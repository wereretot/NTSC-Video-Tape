#include "ntsc_simulator.h"
#include <SDL2/SDL.h>
#include <omp.h>
#include <algorithm>
#include <cmath>

std::atomic<double> g_wallTimeSec{0.0};

static inline uint32_t xorshift32(uint32_t& s) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
static inline float xorshift_f(uint32_t& s) {
    return (float)(xorshift32(s) >> 8) * (1.f / 16777216.f) - 1.f;
}

void NTSCSimulator::initBuffers(int w, int h) {
    W_ = w;
    H_ = h;
    H_BLANK = 144;
    W_TOTAL = w + H_BLANK;
    kSC_PX = (2.0f * kPI * kSC_FREQ) / (float)W_;
    for (int i = 0; i < 4; ++i) prev_line_luma_[i] = 0.0f;

    noise_buf_.resize(W_TOTAL * H_);
    sc_cos_.resize(W_TOTAL);
    sc_sin_.resize(W_TOTAL);
    comp_frame_.resize(W_TOTAL * H_);
    
    for (int x = 0; x < W_TOTAL; ++x) {
        float base_theta = float(x) * kSC_PX;
        sc_cos_[x] = std::cos(base_theta);
        sc_sin_[x] = std::sin(base_theta);
    }

    field_buf_[0].create(H_ / 2, W_, CV_8UC3);
    field_buf_[1].create(H_ / 2, W_, CV_8UC3);
    field_out_[0].create(H_ / 2, W_, CV_8UC3);
    field_out_[1].create(H_ / 2, W_, CV_8UC3);
    phosphor_buffer_.create(H_, W_, CV_32FC3);
    phosphor_buffer_.setTo(cv::Scalar(0,0,0));
}

float NTSCSimulator::generateSnow(float time, int x, int y, std::mt19937& rng) {
    // No longer heavily used now that we have noise_buf_, but keeping for compatibility.
    float snow = 0.0f;
    for (int i = 0; i < 4; ++i) {
        snow_phase_[i] = std::fmod(snow_phase_[i] + 0.016f * (0.5f + i * 0.3f), 1.0f);
        float nx = float(x) * 0.1f + float(y) * 0.07f;
        snow += std::sin(snow_phase_[i] * 2.0f * kPI + nx) * 0.25f;
    }
    snow += std::uniform_real_distribution<float>(-0.5f, 0.5f)(rng);
    return snow * 0.5f;
}

void NTSCSimulator::process(const cv::Mat& in, cv::Mat& out, int frameNum, const EngineParams& ep,
                 const VideoParams& vp, float tapeSpd, float instantSpd, float wallDt,
                 int current_recording_id) {
    const int W = in.cols, H = in.rows;
    if (W == 0 || H == 0) return;
    if (W != W_ || H != H_) initBuffers(W, H);

    wallTime_ = (float)g_wallTimeSec.load();
    tapeTime_ += wallDt * tapeSpd;

    // Transition recording
    if (current_recording_id != last_recording_id_) {
        last_recording_id_ = current_recording_id;
        recording_transition_time_ = 1.0f;
    }
    if (recording_transition_time_ > 0.0f) {
        recording_transition_time_ -= wallDt * 2.0f;
        if (recording_transition_time_ < 0.0f) recording_transition_time_ = 0.0f;
        recording_blend_ = recording_transition_time_;
    }

    uint32_t ns = uint32_t(frameNum) * 2654435761u ^ uint32_t(recording_id_) * 40503u;
    for (float& v : noise_buf_) v = xorshift_f(ns);

    float drum_angle = std::fmod(tapeTime_ * (kVHS_DRUM_RPM / 60.f), 1.f);
    int active_head = (drum_angle >= 0.5f) ? 1 : 0;
    
    // Split into even and odd fields
    for (int y = 0; y < H / 2; ++y) {
        in.row(y * 2).copyTo(field_buf_[0].row(y));
        in.row(y * 2 + 1).copyTo(field_buf_[1].row(y));
    }

    processField(field_buf_[0], field_out_[0], frameNum, ep, vp, tapeSpd, instantSpd, wallDt, 0, active_head);
    processField(field_buf_[1], field_out_[1], frameNum, ep, vp, tapeSpd, instantSpd, wallDt, 1, 1 - active_head);

    out.create(H, W, CV_8UC3);
    for (int y = 0; y < H / 2; ++y) {
        field_out_[0].row(y).copyTo(out.row(y * 2));
        field_out_[1].row(y).copyTo(out.row(y * 2 + 1));
    }
}

void NTSCSimulator::processField(const cv::Mat& inField, cv::Mat& outField, int frameNum, const EngineParams& ep,
                      const VideoParams& vp, float tapeSpd, float instantSpd, float wallDt, int fieldIdx, int active_head) {
    const int W = inField.cols;
    const int H = inField.rows;
    outField.create(H, W, CV_8UC3);

    // Derived variables from implementation plan
    float lp_factor = 1.f - (ep.ips_base / 15.f);
    float preemph_coeff = dsp::lerp(0.18f, 0.26f, lp_factor);
    float lpfA = dsp::lerp(0.32f, 0.22f, lp_factor);
    int chroma_radius = int(float(W) * 0.5f / 4.4f);

    const float kHeadAngleRad = kVHS_HEAD_ANGLE_DEG * kPI / 180.f;
    const float tape_speed_mm_s = ep.ips_base * 25.4f;
    const float H_period_s = 1.f / kNTSC_HSYNC;
    float tbe_mm_per_line = tape_speed_mm_s * std::cos(kHeadAngleRad) * H_period_s;
    float head_sweep_grad_derived = (tbe_mm_per_line / kVHS_TRACK_PITCH_SP) / float(W);

    float rf = vp.base_rf_level * (1.f - vp.tape_age * 0.5f) * (1.f - vp.oxide_shedding * 0.3f);
    float snow_prob = std::max(0.f, (0.4f - rf) * 3.f);
    float snow_density = snow_prob * (1.f + vp.dropout_rate * 5.f);

    float switch_proximity = std::min(std::abs((float)active_head - 0.0f), std::abs((float)active_head - 0.5f));
    float switch_dropout = std::max(0.f, 1.f - switch_proximity * 60.f);

    std::vector<float> comp_field(W_TOTAL * H, 0.0f);
    
    // Pass 1: Encode (parallel)
    #pragma omp parallel for schedule(static)
    for (int y = 0; y < H; ++y) {
        float h_displacement = std::sin(h_roll_phase_ + y * 0.1f) * vp.tracking_error; 
        float Y_prev_encoded = 0.0f;
        
        for (int x = 0; x < W_TOTAL; ++x) {
            float Y = 0.0f, U = 0.0f, V = 0.0f;
            if (x >= H_BLANK) {
                int px = x - H_BLANK;
                int sx = std::max(0, std::min(W - 1, px + (int)h_displacement));
                const auto& color = inField.at<cv::Vec3b>(y, sx);
                float b = color[0] / 255.f;
                float g = color[1] / 255.f;
                float r = color[2] / 255.f;
                
                Y =  0.299f*r + 0.587f*g + 0.114f*b;
                U = -0.147f*r - 0.289f*g + 0.436f*b;
                V =  0.615f*r - 0.515f*g - 0.100f*b;
            }
            
            // FM Pre-emphasis
            float Y_preemph = Y + (Y - Y_prev_encoded) * preemph_coeff;
            Y_prev_encoded  = Y;
            Y_preemph       = std::tanh(Y_preemph * 1.8f) / 1.8f;
            
            // FM noise floor
            float fm_noise_a = noise_buf_[(y * 2 + fieldIdx) * W_TOTAL + x];
            float fm_noise_b = noise_buf_[(y * 2 + fieldIdx) * W_TOTAL + std::max(0, x - 1)];
            float fm_noise   = (fm_noise_a - fm_noise_b) * vp.luma_noise * 0.5f;
            
            Y_preemph += fm_noise;
            
            // Modulate onto composite
            float theta = float(x) * kSC_PX + field_chroma_phase_accum_;
            comp_field[y * W_TOTAL + x] = Y_preemph + U * std::sin(theta) + V * std::cos(theta);
        }
    }
    
    // Pass 2: Decode (parallel)
    #pragma omp parallel for schedule(static)
    for (int y = 0; y < H; ++y) {
        float lpY = 0.f, si = 0.f, sq = 0.f;
        
        // tracking error HSW
        float hsw_offset = (y > H - kVHS_HSW_LINE) ? vp.head_switch_jitter * 5.0f : 0.0f;
        
        for (int x = 0; x < W; ++x) {
            int cx = x + H_BLANK + hsw_offset + head_sweep_grad_derived * x;
            if (cx < 0) cx = 0;
            if (cx >= W_TOTAL) cx = W_TOTAL - 1;
            
            float sig = comp_field[y * W_TOTAL + cx];
            
            lpY += lpfA * (sig - lpY);
            
            float chromaSig = sig - lpY;
            float theta = float(cx) * kSC_PX + field_chroma_phase_accum_;
            
            float rawU = 2.0f * chromaSig * std::sin(theta);
            float rawV = 2.0f * chromaSig * std::cos(theta);
            
            // Simple IIR chroma 
            si += 0.12f * (rawU - si);
            sq += 0.12f * (rawV - sq);
            
            // Heterodyne phase error
            float heterodyne_phase = field_chroma_phase_accum_ + vp.inter_field_phase_error * adjacent_track_phase_ * kPI * 0.5f;
            float U_out = si * std::cos(heterodyne_phase) - sq * std::sin(heterodyne_phase);
            float V_out = si * std::sin(heterodyne_phase) + sq * std::cos(heterodyne_phase);
            
            // YUV back to RGB
            float r = lpY + 1.140f * V_out;
            float g = lpY - 0.395f * U_out - 0.581f * V_out;
            float b = lpY + 2.032f * U_out;
            
            // Noise & Snow
            float snow = noise_buf_[((H - 1 - y) * 2 + fieldIdx) * W_TOTAL + (W_TOTAL - 1 - x)];
            if (std::abs(snow) < snow_density) {
                r += snow * 0.5f + 0.3f;
                g += snow * 0.5f + 0.3f;
                b += snow * 0.5f + 0.3f;
            }
            
            // TV Controls (Brightness, contrast)
            r = (r - 0.5f) * tvParams_.tv_contrast + 0.5f + (tvParams_.tv_brightness - 0.5f) * 0.3f;
            g = (g - 0.5f) * tvParams_.tv_contrast + 0.5f + (tvParams_.tv_brightness - 0.5f) * 0.3f;
            b = (b - 0.5f) * tvParams_.tv_contrast + 0.5f + (tvParams_.tv_brightness - 0.5f) * 0.3f;
            
            auto& pixel = outField.at<cv::Vec3b>(y, x);
            pixel[0] = (uchar)std::clamp(b * 255.f, 0.f, 255.f);
            pixel[1] = (uchar)std::clamp(g * 255.f, 0.f, 255.f);
            pixel[2] = (uchar)std::clamp(r * 255.f, 0.f, 255.f);
        }
    }
}
