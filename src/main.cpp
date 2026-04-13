// ============================================================
//  ADO-8500  Digital Video Effects Processor
//  Analog Tape Simulation  –  NTSC Composite Encoder/Decoder
//  Multi-threaded C++17  ·  SDL2 / OpenCV / Dear ImGui
//  Audio engine: CapstanVar (libCapstanVar.a)
//
//  ── Audio/Video Sync Architecture ───────────────────────────
//
//  g_audioSamplePos is the SINGLE clock, incremented every
//  main-loop tick by wall-clock × sampleRate × tapeSpd.
//
//  tape_speed_mult scales BOTH audio pitch (CapstanVar) AND
//  NTSC geometric distortion (NTSCSimulator). They share the
//  same scalar — they cannot desync.
//
//  All NTSC distortion oscillators run off `simTime` which is
//  frameNum × (1/29.97) × tapeSpd — i.e. they move at exactly
//  the same speed as the audio.
//
//  ── NTSC-RS Composite Signal Chain ──────────────────────────
//
//    ENCODE (per scanline):
//      RGB → YIQ
//      composite[x] = Y + I·cos(θ) − Q·sin(θ)
//      θ = fieldPhase + line·π + x·(2π·227.5/W)
//
//    VHS RECORD:
//      FM pre-emphasis → Luma lowpass IIR → Chroma demod
//      Narrow chroma IIR → VHS vertical chroma blend → Hiss
//
//    DECODE → ARTIFACT LAYER:
//      YIQ → RGB
//      Head-switch noise bar, dropouts, crosstalk,
//      print-through, sync-loss V-roll + hue spin,
//      timebase-error H warp (tapeSpd ≠ 1)
//
//  ── NTSC Timing Constants ──────────────────────────────────
//  Field rate:  59.94 Hz  (interlaced → 29.97 fps)
//  H-sync rate: 15734.26 Hz  (525 lines × 29.97 × 2 fields)
//  There is no configurable framerate — the pipeline runs at
//  native NTSC timing only.
// ============================================================

// ── CapstanVar ──────────────────────────────────────────────
#define _CMATH_
#include "engine.hpp"
#include "audio_io.hpp"
#include "dsp_types.hpp"
#undef _CMATH_

// ── SDL2 ────────────────────────────────────────────────────
#include <SDL2/SDL.h>
#include <SDL2/SDL_audio.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

// ── OpenCV ──────────────────────────────────────────────────
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// ── FFmpeg ──────────────────────────────────────────────────
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

// ── STL ─────────────────────────────────────────────────────
#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#ifdef ADO_OPENMP
#  include <omp.h>
#endif

// ============================================================
//  §1  Constants
// ============================================================

static constexpr int   FW            = 640;
static constexpr int   FH            = 480;
static constexpr float ASPECT        = float(FW)/float(FH);
static constexpr int   Q_DEPTH       = 6;
static constexpr int   AUDIO_SR     = 44100;
static const     float kPI           = 3.14159265f;
// NTSC subcarrier: 227.5 cycles per active line at W pixels
static constexpr float kSC_PX        = 2.f * 3.14159265f * 227.5f / float(FW);
// NTSC timing — hard-locked, not configurable
static constexpr float kNTSC_FPS     = 29.97f;       // 59.94 Hz field / 2
static constexpr float kNTSC_FRAME_S = 1.f / kNTSC_FPS; // ~33.367 ms
static constexpr float kNTSC_HSYNC   = 15734.26f;    // Hz
// Active lines of a 525-line NTSC frame (visible ≈ 480–486)
static constexpr int   kNTSC_TOTAL_LINES = 525;
static constexpr int   kNTSC_ACTIVE_LINES = 480;
// VBI head-switching: occurs in last ~15 lines of 525
static constexpr float kHS_VBI_LINES = 15.f;

// ============================================================
//  §1b  DSP helpers
// ============================================================
namespace dsp {
inline float clamp(float v, float lo=0.f, float hi=1.f){return v<lo?lo:v>hi?hi:v;}
inline uint8_t clamp8(float v){return uint8_t(std::max(0.f,std::min(255.f,v)));}
inline float lerp(float a,float b,float t){return a+(b-a)*t;}

inline cv::Vec3b bsample(const cv::Mat& s, float x, float y) {
    x=std::max(0.f,std::min(float(s.cols-1.001f),x));
    y=std::max(0.f,std::min(float(s.rows-1.001f),y));
    int ix=int(x),iy=int(y); float fx=x-ix,fy=y-iy;
    int ix1=std::min(ix+1,s.cols-1), iy1=std::min(iy+1,s.rows-1);
    const auto& p00=s.at<cv::Vec3b>(iy,ix);
    const auto& p10=s.at<cv::Vec3b>(iy,ix1);
    const auto& p01=s.at<cv::Vec3b>(iy1,ix);
    const auto& p11=s.at<cv::Vec3b>(iy1,ix1);
    return cv::Vec3b{
        clamp8((p00[0]*(1-fx)+p10[0]*fx)*(1-fy)+(p01[0]*(1-fx)+p11[0]*fx)*fy),
        clamp8((p00[1]*(1-fx)+p10[1]*fx)*(1-fy)+(p01[1]*(1-fx)+p11[1]*fx)*fy),
        clamp8((p00[2]*(1-fx)+p10[2]*fx)*(1-fy)+(p01[2]*(1-fx)+p11[2]*fx)*fy)};
}
} // dsp

// ============================================================
//  §2  Audio extraction
// ============================================================
static std::string extractAudioToWav(const std::string& vp) {
    // Use a unique name to avoid collision during re-import
    static std::atomic<int> counter{0};
    std::string wav = "temp_ado_" + std::to_string(SDL_GetTicks64()) + "_" + std::to_string(counter++) + ".wav";
    char cmd[2048];
    std::snprintf(cmd,sizeof(cmd),
        "ffmpeg -y -i \"%s\" -vn -acodec pcm_s16le -ar %d -ac 2 \"%s\" 2>/dev/null",
        vp.c_str(),AUDIO_SR,wav.c_str());
    if(std::system(cmd)!=0)return "";
    FILE* f=std::fopen(wav.c_str(),"rb"); if(!f)return "";
    std::fseek(f,0,SEEK_END); long sz=std::ftell(f); std::fclose(f);
    return sz>=44?wav:"";
}

// ============================================================
//  §2b  SMPTE test-pattern generator
// ============================================================
class DemoSource {
public:
    cv::Mat generate(int w, int h) {
        cv::Mat f(h,w,CV_8UC3); phase_+=.016f;
        static const cv::Scalar bars[]={{192,192,192},{192,192,0},{0,192,192},
            {0,192,0},{192,0,192},{192,0,0},{0,0,192}};
        int bw=w/7;
        for(int i=0;i<7;++i) cv::rectangle(f,{i*bw,0},{(i+1)*bw,(int)(h*.75)},bars[i],-1);
        cv::rectangle(f,{0,(int)(h*.75)},{(int)(w*.16),h},{192,0,0},-1);
        cv::rectangle(f,{(int)(w*.16),(int)(h*.75)},{(int)(w*.26),h},{255,255,255},-1);
        cv::rectangle(f,{(int)(w*.26),(int)(h*.75)},{(int)(w*.36),h},{192,0,192},-1);
        cv::rectangle(f,{(int)(w*.36),(int)(h*.75)},{(int)(w*.66),h},{12,12,12},-1);
        int cx=(int)(w*.5f+std::sin(phase_)*w*.2f);
        int cy=(int)(h*.45f+std::cos(phase_*.7f)*h*.12f);
        cv::circle(f,{cx,cy},36,{255,255,255},-1);
        cv::circle(f,{cx,cy},30,{0,200,64},-1);
        using namespace std::chrono;
        long ms=duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
        char tc[48]; std::snprintf(tc,sizeof(tc),"ADO-8500  %02ld:%02ld:%02ld:%02ld",
            (ms/3600000)%24,(ms/60000)%60,(ms/1000)%60,(ms%1000)/33);
        cv::rectangle(f,{4,4},{286,26},{0,0,0},-1);
        cv::putText(f,tc,{8,20},cv::FONT_HERSHEY_PLAIN,1.,{0,255,64},1,cv::LINE_AA);
        return f;
    }
private: float phase_=0.f;
};

// ============================================================
//  §3  Thread-safe frame queue
// ============================================================
template<typename T>
class FrameQueue {
public:
    explicit FrameQueue(size_t cap=Q_DEPTH):cap_(cap){}
    void push(T v){
        {std::lock_guard<std::mutex> lk(mu_);
         if(q_.size()>=cap_)q_.pop_front();
         q_.push_back(std::move(v));}
        cv_.notify_one();}
    bool pop(T& out,int ms=50){
        std::unique_lock<std::mutex> lk(mu_);
        if(!cv_.wait_for(lk,std::chrono::milliseconds(ms),[this]{return !q_.empty();}))return false;
        out=std::move(q_.front());q_.pop_front();return true;}
    bool try_pop(T& out){
        std::lock_guard<std::mutex> lk(mu_);
        if(q_.empty())return false;
        out=std::move(q_.front());q_.pop_front();return true;}
    void clear(){std::lock_guard<std::mutex> lk(mu_); q_.clear();}
    size_t size()const{std::lock_guard<std::mutex> lk(mu_);return q_.size();}
private:
    mutable std::mutex mu_; std::condition_variable cv_;
    std::deque<T> q_; size_t cap_;
};

// NTSC Constants
static constexpr float kSC_FREQ = 227.5f; // Cycles per line

struct VideoParams {
    float tracking_error = 0.0f;
    float tape_crease = 0.0f;
    float sync_hold_failure = 0.0f;
};

class NTSCSimulator {
public:
    float v_roll_accum_ = 0.0f;
    float noise_bar_phase_ = 0.0f;
    float tapeTime_ = 0.0f;
    float wallTime_ = 0.0f;
    float h_roll_phase_ = 0.0f;
    float tracking_error_lpf_ = 0.0f;
    void initBuffers(int w, int h) {
        W_ = w;
        H_BLANK = 144;
        W_TOTAL = w + H_BLANK;
        kSC_PX = (2.0f * kPI * kSC_FREQ) / (float)W_;
    }

    void process(const cv::Mat& in, cv::Mat& out, int frameNum, const EngineParams& ep, const VideoParams& vp, float tapeSpd, float instantSpd) {
        const int W = in.cols, H = in.rows;
        if (W != W_) initBuffers(W, H);

        const float spd = std::clamp(tapeSpd, 0.1f, 4.0f);
        wallTime_ += kNTSC_FRAME_S;
        tapeTime_ += kNTSC_FRAME_S * instantSpd;
        
        const float fPhase = float(frameNum % 4) * kPI * 0.5f;

        // --- V-SYNC LOSS (Rolling) & TAPE SPEED ERROR ---
        float speed_error = std::abs(1.0f - instantSpd);
        float target_error = speed_error + vp.tracking_error; // Uses VideoParams
        if (target_error > 2.0f) target_error = 2.0f;
        tracking_error_lpf_ += (target_error - tracking_error_lpf_) * 0.1f;

        if (target_error > 0.05f || vp.sync_hold_failure > 0.0f) {
            v_roll_accum_ += (target_error * 50.0f) + (vp.sync_hold_failure * 150.0f); 
        } else {
            v_roll_accum_ *= 0.9f;
        }
        int v_roll = int(v_roll_accum_) % H;

        // --- TRACKING ENVELOPE (Noise Bar Drift) ---
        noise_bar_phase_ += target_error * 0.25f;
        if (noise_bar_phase_ > 1.0f) noise_bar_phase_ -= 1.0f;
        float bar_y = noise_bar_phase_ * float(H);
        float bar_width = 30.0f + tracking_error_lpf_ * 100.0f;

        // --- Massive True H-SYNC Roll ---
        float h_freq_err = (target_error * 15.0f) + (vp.sync_hold_failure * 150.0f);
        h_roll_phase_ -= h_freq_err * kNTSC_FRAME_S; 
        if (h_roll_phase_ < 0.0f) h_roll_phase_ += 1.0f;
        
        // Organic AFC Snap-back when fast/locked
        if (h_freq_err < 0.01f && h_roll_phase_ != 0.0f) {
            float dist = h_roll_phase_;
            if (dist > 0.5f) dist -= 1.0f; 
            dist *= 0.8f; 
            if (std::abs(dist) < 0.001f) h_roll_phase_ = 0.0f;
            else h_roll_phase_ = dist > 0.0f ? dist : 1.0f + dist;
        }

        // --- Tape Crease phase ---
        float crease_y = std::fmod(tapeTime_ * 0.1f, 1.0f) * float(H);

        out.create(H, W, CV_8UC3);

#ifdef ADO_OPENMP
#pragma omp parallel
        {
            // Per-thread composite line buffer to avoid cross-line data races.
            std::vector<float> comp_line;
            comp_line.resize(W_TOTAL);
#pragma omp for schedule(dynamic, 16)
            for (int y = 0; y < H; ++y) {
                std::mt19937 rng(uint32_t(frameNum) * 65537u + uint32_t(y) * 1013u);
                int src_y = (y + v_roll) % H;
                const uchar* sr = in.ptr(src_y);
                uchar* dr = out.ptr(y);

                // 1. PHYSICAL TAPE GEOMETRY (The "Source" signal)
                // --- A. Wow (Tension Flagging at Top of Screen) ---
                float tension_error = std::sin(tapeTime_ * 1.3f) * ep.wow_dep;
                // TV AFC exponential catch-up creating the classic "Flag" bend from the top
                float topFlagging   = std::exp(-float(y) * 0.04f) * tension_error * 15.0f;

                // --- B. Flutter (High freq jagged horizontal tracking) ---
                float flutterVal = 0.0f;
                if (ep.flutter_dep > 0.001f) {
                    flutterVal = (std::sin(wallTime_ * 50.0f + y * 0.15f) * 0.3f 
                               + std::sin(wallTime_ * 23.0f + y * 0.8f) * 0.1f 
                               + std::normal_distribution<float>(0, 1)(rng) * 0.2f) * ep.flutter_dep * 5.0f;
                }

                // --- C. Motor Drag & Belt Slips (Mid-screen AFC Snapping) ---
                float drag_hz = 0.5f + ep.motor_health * 2.0f;
                // Locate the 'snap' point vertically based on phase
                float drag_phase = std::fmod(tapeTime_ * drag_hz, 1.0f) * H;
                float dy = float(y) - drag_phase;
                float beltSlip = 0.0f;
                if (dy > 0.0f && dy < 100.0f && ep.motor_drag > 0.01f) {
                    // Underdamped ringing as AFC tries to catch the slip
                    beltSlip = std::exp(-dy * 0.05f) * std::sin(dy * 0.2f) * (ep.motor_drag * 20.0f);
                }

                // --- D. Head Switch Noise (VHS Tearing at bottom of VBI) ---
                // Approaching the bottom of the visible frame, the head switch causes a phase jump.
                float hsTearing = 0.0f;
                if (y >= H - kHS_VBI_LINES) {
                    float hsDepth = (y - (H - kHS_VBI_LINES)) / kHS_VBI_LINES;
                    hsTearing = std::pow(hsDepth, 2.0f) * 12.0f; 
                    // Add some noise to the head switch
                    hsTearing += std::normal_distribution<float>(0, 1)(rng) * 2.0f;
                }

                // --- Massive H-SYNC Roll & Tracking Shear ---
                // Tracking RF Mask
                float dist_rf = std::abs(float(src_y) - bar_y);
                if (dist_rf > (float)H / 2.0f) dist_rf = (float)H - dist_rf; 
                float tracking_rf = 1.0f;
                if (dist_rf < bar_width) {
                    float min_rf = dist_rf / bar_width;
                    float fade = std::clamp(tracking_error_lpf_ * 10.0f, 0.0f, 1.0f);
                    tracking_rf = 1.0f - fade * (1.0f - min_rf); 
                }

                // Add tape crease disturbance
                float dist_crease = std::abs(float(src_y) - crease_y);
                if (dist_crease > (float)H / 2.0f) dist_crease = (float)H - dist_crease;
                float crease_shear = 0.0f;
                if (vp.tape_crease > 0.01f && dist_crease < 15.0f) {
                    crease_shear = (15.0f - dist_crease) * vp.tape_crease * 5.0f;
                    tracking_rf *= std::max(0.1f, 1.0f - vp.tape_crease * (1.0f - dist_crease/15.0f));
                }

                float current_h_roll = h_roll_phase_ + (float(y) / float(H)) * h_freq_err * 0.033f;
                float h_sync_roll = current_h_roll * float(W_TOTAL);

                float h_sync_shear = 0.0f;
                if (tracking_error_lpf_ > 0.0f || vp.tape_crease > 0.0f) {
                     float chaotic_shear = std::sin(src_y * 0.05f + wallTime_ * 10.0f) * 0.5f 
                                         + std::cos(src_y * 0.12f - wallTime_ * 15.0f) * 0.3f
                                         + std::normal_distribution<float>(0, 1)(rng) * 0.2f;
                     float rf_penalty = (1.0f - tracking_rf) * 3.0f + 1.0f; 
                     h_sync_shear = (tracking_error_lpf_ * 50.0f) * chaotic_shear * rf_penalty;
                }

                float tbe_start = topFlagging + flutterVal + beltSlip + hsTearing + h_sync_roll + h_sync_shear + crease_shear;
                
                // Stretch line internally based on current derivative
                float tbe_slope = std::normal_distribution<float>(0, 1)(rng) * ep.flutter_dep * 0.5f;
                if (dy > 0.0f && dy < 100.0f) {
                    // Huge stretch/squash during the snap
                    tbe_slope += std::exp(-dy * 0.05f) * std::cos(dy * 0.2f) * (ep.motor_drag * 15.0f);
                }

                const float linePhase = fPhase + float(y) * kPI;

                // --- ENCODE ---
                for (int x = 0; x < W_TOTAL; ++x) {
                    float current_tbe = tbe_start + (float(x) / float(W_TOTAL)) * tbe_slope;
                    float xSrc = (float)x - current_tbe;
                    xSrc = std::fmod(xSrc, float(W_TOTAL));
                    if (xSrc < 0.0f) xSrc += float(W_TOTAL);
                    float theta = linePhase + xSrc * kSC_PX / spd;

                    float Y = 0.0f, I = 0.0f, Q = 0.0f;

                    if (xSrc < 60) {
                        Y = -0.3f; // Sync
                    } else if (xSrc >= 70 && xSrc < 110) {
                        Y = 0.0f;  // Burst
                        I = 0.0f; Q = 0.35f;
                    } else if (xSrc >= H_BLANK) {
                        float vX = std::clamp(xSrc - H_BLANK, 0.f, float(W - 1.001f));
                        int x0 = (int)vX;
                        float r = sr[x0 * 3 + 2] / 255.f;
                        float g = sr[x0 * 3 + 1] / 255.f;
                        float b = sr[x0 * 3 + 0] / 255.f;

                        Y = 0.299f * r + 0.587f * g + 0.114f * b;
                        I = 0.596f * r - 0.274f * g - 0.321f * b;
                        Q = 0.211f * r - 0.523f * g + 0.311f * b;
                    }
                    if (tracking_rf < 0.95f && xSrc >= H_BLANK) {
                         Y *= tracking_rf; I *= tracking_rf; Q *= tracking_rf;
                         float noise_amp = 1.0f - tracking_rf;
                         Y += std::uniform_real_distribution<float>(-0.5f, 0.5f)(rng) * noise_amp;
                         if (noise_amp > 0.3f) { I = 0.f; Q = 0.f; } // color completely fails in deepest tracking
                    }
                    comp_line[x] = Y + I * std::cos(theta) - Q * std::sin(theta);
                }

                // --- DECODE (Closed-Loop PLL) ---
                float phase_accum = 0.0f;

                // Burst is encoded as Q=0.35, so the measured burst angle is always
                // atan2(positive,0) = π/2. Subtract the expected reference so the
                // net correction is 0 when encode and decode are synchronized.
                float sumI = 0.f, sumQ = 0.f;
                for (int x = 70; x < 110; ++x) {
                    float refTheta = linePhase + float(x) * kSC_PX;
                    sumI += comp_line[x] * std::cos(refTheta);
                    sumQ -= comp_line[x] * std::sin(refTheta);
                }
                phase_accum = std::atan2(sumQ, sumI) - (kPI / 2.0f);

                // --- TBC FAILURE (Loss of Color Lock) ---
                float pll_instability = std::max(0.0f, std::abs(beltSlip) - 2.0f) * 0.05f + ep.flutter_dep * 0.15f;
                if (ep.dropout_rate > 0.01f) {
                     if (std::uniform_real_distribution<float>(0, 1)(rng) < ep.dropout_rate * 0.5f) {
                         pll_instability += 2.0f;
                     }
                }
                float pll_drift_accum = 0.0f;

                // Demodulation variables
                float lpY = 0.f, si = 0.f, sq = 0.f;
                const float lpfA = 0.35f;
                const float chrA = 0.12f;

                for (int out_x = 0; out_x < W; ++out_x) {
                    int x = out_x + H_BLANK;
                    float sig = comp_line[x];

                    // 1. Adaptive Phase Tracking Simulation
                    pll_drift_accum += std::normal_distribution<float>(0, 1)(rng) * pll_instability;
                    float decTheta = linePhase + (float(x) / spd) * kSC_PX + phase_accum + pll_drift_accum;

                    // 2. Extract Luma
                    lpY += lpfA * (sig - lpY);

                    // 3. Demodulate Chroma
                    float chromaSig = sig - lpY;
                    float rawI = 2.0f * chromaSig * std::cos(decTheta);
                    float rawQ = -2.0f * chromaSig * std::sin(decTheta);

                    si += chrA * (rawI - si);
                    sq += chrA * (rawQ - sq);

                    // 4. Transform back to RGB
                    float r = lpY + 0.956f * si + 0.621f * sq;
                    float g = lpY - 0.272f * si - 0.647f * sq;
                    float b = lpY - 1.106f * si + 1.703f * sq;

                    dr[out_x * 3 + 0] = (uchar)std::clamp(b * 255.f, 0.f, 255.f);
                    dr[out_x * 3 + 1] = (uchar)std::clamp(g * 255.f, 0.f, 255.f);
                    dr[out_x * 3 + 2] = (uchar)std::clamp(r * 255.f, 0.f, 255.f);
                }
            }
        }
#else
        // Single-threaded fallback uses a single composite line buffer.
        std::vector<float> comp_line;
        comp_line.resize(W_TOTAL);
        for (int y = 0; y < H; ++y) {
            std::mt19937 rng(uint32_t(frameNum) * 65537u + uint32_t(y) * 1013u);
            int src_y = (y + v_roll) % H;
            const uchar* sr = in.ptr(src_y);
            uchar* dr = out.ptr(y);

            // 1. PHYSICAL TAPE GEOMETRY (The "Source" signal)
            // --- A. Wow (Tension Flagging at Top of Screen) ---
            float tension_error = std::sin(tapeTime_ * 1.3f) * ep.wow_dep;
            // TV AFC exponential catch-up creating the classic "Flag" bend from the top
            float topFlagging   = std::exp(-float(y) * 0.04f) * tension_error * 15.0f;

            // --- B. Flutter (High freq jagged horizontal tracking) ---
            float flutterVal = 0.0f;
            if (ep.flutter_dep > 0.001f) {
                flutterVal = (std::sin(wallTime_ * 50.0f + y * 0.15f) * 0.3f 
                           + std::sin(wallTime_ * 23.0f + y * 0.8f) * 0.1f 
                           + std::normal_distribution<float>(0, 1)(rng) * 0.2f) * ep.flutter_dep * 5.0f;
            }

            // --- C. Motor Drag & Belt Slips (Mid-screen AFC Snapping) ---
            float drag_hz = 0.5f + ep.motor_health * 2.0f;
            // Locate the 'snap' point vertically based on phase
            float drag_phase = std::fmod(tapeTime_ * drag_hz, 1.0f) * H;
            float dy = float(y) - drag_phase;
            float beltSlip = 0.0f;
            if (dy > 0.0f && dy < 100.0f && ep.motor_drag > 0.01f) {
                // Underdamped ringing as AFC tries to catch the slip
                beltSlip = std::exp(-dy * 0.05f) * std::sin(dy * 0.2f) * (ep.motor_drag * 20.0f);
            }

            // --- D. Head Switch Noise (VHS Tearing at bottom of VBI) ---
            // Approaching the bottom of the visible frame, the head switch causes a phase jump.
            float hsTearing = 0.0f;
            if (y >= H - kHS_VBI_LINES) {
                float hsDepth = (y - (H - kHS_VBI_LINES)) / kHS_VBI_LINES;
                hsTearing = std::pow(hsDepth, 2.0f) * 12.0f; 
                // Add some noise to the head switch
                hsTearing += std::normal_distribution<float>(0, 1)(rng) * 2.0f;
            }

            // --- Massive H-SYNC Roll & Tracking Shear ---
            // Tracking RF Mask
            float dist_rf = std::abs(float(src_y) - bar_y);
            if (dist_rf > (float)H / 2.0f) dist_rf = (float)H - dist_rf; 
            float tracking_rf = 1.0f;
            if (tracking_error_lpf_ > 0.05f && dist_rf < bar_width) {
                tracking_rf = dist_rf / bar_width; 
            }

            // Add tape crease disturbance
            float dist_crease = std::abs(float(src_y) - crease_y);
            if (dist_crease > (float)H / 2.0f) dist_crease = (float)H - dist_crease;
            float crease_shear = 0.0f;
            if (vp.tape_crease > 0.01f && dist_crease < 15.0f) {
                crease_shear = (15.0f - dist_crease) * vp.tape_crease * 5.0f;
                tracking_rf *= std::max(0.1f, 1.0f - vp.tape_crease * (1.0f - dist_crease/15.0f));
            }

            float current_h_roll = h_roll_phase_ + (float(y) / float(H)) * h_freq_err * 0.033f;
            float h_sync_roll = current_h_roll * float(W_TOTAL);

            float h_sync_shear = 0.0f;
            if (tracking_error_lpf_ > 0.0f || vp.tape_crease > 0.0f) {
                 float chaotic_shear = std::sin(src_y * 0.05f + wallTime_ * 10.0f) * 0.5f 
                                     + std::cos(src_y * 0.12f - wallTime_ * 15.0f) * 0.3f
                                     + std::normal_distribution<float>(0, 1)(rng) * 0.2f;
                 float rf_penalty = (1.0f - tracking_rf) * 3.0f + 1.0f; 
                 h_sync_shear = (tracking_error_lpf_ * 50.0f) * chaotic_shear * rf_penalty;
            }

            float tbe_start = topFlagging + flutterVal + beltSlip + hsTearing + h_sync_roll + h_sync_shear + crease_shear;
            
            // Stretch line internally based on current derivative
            float tbe_slope = std::normal_distribution<float>(0, 1)(rng) * ep.flutter_dep * 0.5f;
            if (dy > 0.0f && dy < 100.0f) {
                // Huge stretch/squash during the snap
                tbe_slope += std::exp(-dy * 0.05f) * std::cos(dy * 0.2f) * (ep.motor_drag * 15.0f);
            }

            const float linePhase = fPhase + float(y) * kPI;

            // --- ENCODE ---
            for (int x = 0; x < W_TOTAL; ++x) {
                float current_tbe = tbe_start + (float(x) / float(W_TOTAL)) * tbe_slope;
                float xSrc = (float)x - current_tbe;
                float theta = linePhase + xSrc * kSC_PX / spd;

                float Y = 0.0f, I = 0.0f, Q = 0.0f;

                if (xSrc < 60) {
                    Y = -0.3f; // Sync
                } else if (xSrc >= 70 && xSrc < 110) {
                    Y = 0.0f;  // Burst
                    I = 0.0f; Q = 0.35f;
                } else if (xSrc >= H_BLANK) {
                    float vX = std::clamp(xSrc - H_BLANK, 0.f, float(W - 1.001f));
                    int x0 = (int)vX;
                    float r = sr[x0 * 3 + 2] / 255.f;
                    float g = sr[x0 * 3 + 1] / 255.f;
                    float b = sr[x0 * 3 + 0] / 255.f;

                    Y = 0.299f * r + 0.587f * g + 0.114f * b;
                    I = 0.596f * r - 0.274f * g - 0.321f * b;
                    Q = 0.211f * r - 0.523f * g + 0.311f * b;
                }
                if (tracking_rf < 0.95f && xSrc >= H_BLANK) {
                     float noise_thresh = tracking_rf * tracking_rf;
                     if (std::uniform_real_distribution<float>(0, 1)(rng) > noise_thresh) {
                          Y = std::uniform_real_distribution<float>(-0.2f, 1.0f)(rng);
                          I = 0.0f; Q = 0.0f; // Pure uncolored noise
                     }
                }
                comp_line[x] = Y + I * std::cos(theta) - Q * std::sin(theta);
            }

            // --- DECODE (Closed-Loop PLL) ---
            float phase_accum = 0.0f;

            // Same burst reference correction for single-threaded path.
            float sumI = 0.f, sumQ = 0.f;
            for (int x = 70; x < 110; ++x) {
                float refTheta = linePhase + float(x) * kSC_PX;
                sumI += comp_line[x] * std::cos(refTheta);
                sumQ -= comp_line[x] * std::sin(refTheta);
            }
            phase_accum = std::atan2(sumQ, sumI) - (kPI / 2.0f);

            // --- TBC FAILURE (Loss of Color Lock) ---
            float pll_instability = std::max(0.0f, std::abs(beltSlip) - 2.0f) * 0.05f + ep.flutter_dep * 0.15f;
            if (ep.dropout_rate > 0.01f) {
                 if (std::uniform_real_distribution<float>(0, 1)(rng) < ep.dropout_rate * 0.5f) {
                     pll_instability += 2.0f;
                 }
            }
            float pll_drift_accum = 0.0f;

            // Demodulation variables
            float lpY = 0.f, si = 0.f, sq = 0.f;
            const float lpfA = 0.35f;
            const float chrA = 0.12f;

            for (int out_x = 0; out_x < W; ++out_x) {
                int x = out_x + H_BLANK;
                float sig = comp_line[x];

                // 1. Adaptive Phase Tracking Simulation
                pll_drift_accum += std::normal_distribution<float>(0, 1)(rng) * pll_instability;
                float decTheta = linePhase + (float(x) / spd) * kSC_PX + phase_accum + pll_drift_accum;

                // 2. Extract Luma
                lpY += lpfA * (sig - lpY);

                // 3. Demodulate Chroma
                float chromaSig = sig - lpY;
                float rawI = 2.0f * chromaSig * std::cos(decTheta);
                float rawQ = -2.0f * chromaSig * std::sin(decTheta);

                si += chrA * (rawI - si);
                sq += chrA * (rawQ - sq);

                // 4. Transform back to RGB
                float r = lpY + 0.956f * si + 0.621f * sq;
                float g = lpY - 0.272f * si - 0.647f * sq;
                float b = lpY - 1.106f * si + 1.703f * sq;

                dr[out_x * 3 + 0] = (uchar)std::clamp(b * 255.f, 0.f, 255.f);
                dr[out_x * 3 + 1] = (uchar)std::clamp(g * 255.f, 0.f, 255.f);
                dr[out_x * 3 + 2] = (uchar)std::clamp(r * 255.f, 0.f, 255.f);
            }
        }
#endif
    }

private:
    int W_ = 0, H_BLANK = 0, W_TOTAL = 0;
    float kSC_PX = 0.f;
};

// ============================================================
//  §5  Video Effects
// ============================================================
namespace Effects {
enum class Type:int{None=0,Tumble,PageTurn,Ripple,Sphere,
    Squeeze,Mosaic,Trails,Mirror,CubeSpin,Kaleidoscope,Shatter,COUNT};
static const char* kNames[]={"BYPASS","TUMBLE","PAGE TURN","RIPPLE","SPHERE",
    "SQUEEZE","MOSAIC","TRAILS","MIRROR","CUBE SPIN","KALEID.","SHATTER"};
struct Params{Type type=Type::None;float amount=.5f,speed=.3f,depth=.6f,phase=0.f,mix=1.f;};
static cv::Mat s_trail;

static void fx_none(const cv::Mat& s,cv::Mat& d,const Params&){s.copyTo(d);}

static void fx_tumble(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3,cv::Scalar(0,0,0));
    float ang=p.phase*kPI*2.f,cA=std::cos(ang),sA=std::sin(ang);
    float scX=std::abs(cA); if(scX<.02f)scX=.02f; bool bk=cA<0.f;
    for(int y=0;y<H;++y){uchar* dr=d.ptr(y);float fy=dsp::clamp(float(y)-sA*20.f,0.f,float(H-1));
        for(int x=0;x<W;++x){float sx=(x-W*.5f)/scX+W*.5f;if(bk)sx=W-1-sx;
            auto px=dsp::bsample(s,sx,fy);float sh=1.f-(1.f-scX)*p.depth*.6f;if(bk)sh*=.5f;
            dr[x*3+0]=dsp::clamp8(px[0]*sh);dr[x*3+1]=dsp::clamp8(px[1]*sh);dr[x*3+2]=dsp::clamp8(px[2]*sh);}}}

static void fx_pageturn(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; s.copyTo(d);
    float cx2=W-(std::sin(p.phase*kPI*2.f)*.5f+.5f)*W*p.amount;int cx=int(cx2);if(cx>=W)return;
    cv::rectangle(d,{cx,0},{W-1,H-1},{16,16,16},-1);
    for(int x=std::max(0,cx-24);x<std::min(W,cx+6);++x){
        float tt=float(x-(cx-24))/30.f,hl=std::sin(tt*kPI)*75.f;
        for(int y=0;y<H;++y){uchar*p2=d.ptr(y)+x*3;
            p2[0]=dsp::clamp8(p2[0]+hl);p2[1]=dsp::clamp8(p2[1]+hl);p2[2]=dsp::clamp8(p2[2]+hl);}}
    for(int y=0;y<H;++y){const uchar*sr=s.ptr(y);uchar*dr=d.ptr(y);
        for(int x=cx;x<W;++x){int sx=W-1-(x-cx);if(sx<0)continue;
            float sh=.52f-float(x-cx)/float(W-cx)*.28f;
            dr[x*3+0]=dsp::clamp8(sr[sx*3+0]*sh);dr[x*3+1]=dsp::clamp8(sr[sx*3+1]*sh);
            dr[x*3+2]=dsp::clamp8(sr[sx*3+2]*sh);}}}

static void fx_ripple(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3);
    float amp=p.amount*20.f,freq=p.depth*.09f,spd2=p.phase*kPI*4.f;
    for(int y=0;y<H;++y){uchar*dr=d.ptr(y);
        for(int x=0;x<W;++x){float dx=x-W*.5f,dy=y-H*.5f,dist=std::sqrt(dx*dx+dy*dy);
            float off=std::sin(dist*freq+spd2)*amp;
            auto px=dsp::bsample(s,x+off*(dx/(dist+1.f)),y+off*(dy/(dist+1.f)));
            dr[x*3+0]=px[0];dr[x*3+1]=px[1];dr[x*3+2]=px[2];}}}

static void fx_sphere(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3);
    float R=std::min(W,H)*.44f*(.3f+p.amount*.7f),cx=W*.5f,cy=H*.5f,R2=R*R;
    for(int y=0;y<H;++y){uchar*dr=d.ptr(y);
        for(int x=0;x<W;++x){float dx=x-cx,dy=y-cy,r2=dx*dx+dy*dy;
            if(r2<R2){float r=std::sqrt(r2),th=std::atan2(r/R,std::sqrt(1.f-r2/R2));
                float ss=R*std::sin(th);auto px=dsp::bsample(s,cx+dx*ss/(r+.001f),cy+dy*ss/(r+.001f));
                float sh=std::cos(th);dr[x*3+0]=dsp::clamp8(px[0]*sh);
                dr[x*3+1]=dsp::clamp8(px[1]*sh);dr[x*3+2]=dsp::clamp8(px[2]*sh);}
            else{dr[x*3+0]=18;dr[x*3+1]=18;dr[x*3+2]=18;}}}}

static void fx_squeeze(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3);
    float sq=1.f-p.amount*.8f,bg=1.f+p.depth*2.f;
    for(int y=0;y<H;++y){uchar*dr=d.ptr(y);
        for(int x=0;x<W;++x){float nx=(x-W*.5f)/(W*.5f),ny=(y-H*.5f)/(H*.5f);
            float r2=nx*nx+ny*ny,warp=1.f+(bg-1.f)*std::exp(-r2*4.f);
            auto px=dsp::bsample(s,(nx*sq*warp+1.f)*W*.5f,(ny*sq*warp+1.f)*H*.5f);
            dr[x*3+0]=px[0];dr[x*3+1]=px[1];dr[x*3+2]=px[2];}}}

static void fx_mosaic(const cv::Mat& s,cv::Mat& d,const Params& p){
    int bs=std::max(2,int(p.amount*60.f)+2);
    cv::Mat sm; cv::resize(s,sm,{s.cols/bs,s.rows/bs},0,0,cv::INTER_NEAREST);
    cv::resize(sm,d,s.size(),0,0,cv::INTER_NEAREST);}

static void fx_trails(const cv::Mat& s,cv::Mat& d,const Params& p){
    float bl=1.f-p.amount*.92f;
    if(s_trail.empty()||s_trail.size()!=s.size()) s.copyTo(s_trail);
    cv::addWeighted(s,bl,s_trail,1.f-bl,0.,d); d.copyTo(s_trail);}

static void fx_mirror(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; int spx=std::clamp(int(W*(.5f+p.amount*.5f)),1,W-1);
    s.copyTo(d);
    for(int y=0;y<H;++y)for(int x=spx;x<W;++x){
        int from=spx-(x-spx);if(from<0)from=0;
        const uchar*sp=s.ptr(y)+from*3;uchar*dp=d.ptr(y)+x*3;
        dp[0]=sp[0];dp[1]=sp[1];dp[2]=sp[2];}}

static void fx_cubespin(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3,cv::Scalar(0,0,0));
    float ang=p.phase*kPI*2.f,cA=std::cos(ang),scX=std::abs(cA);
    if(scX<.01f)scX=.01f; bool fl=cA<0.f; int hW=W/2;
    float shift=std::sin(ang)*hW*.5f*p.amount;
    for(int y=0;y<H;++y){uchar*dr=d.ptr(y);
        for(int x=0;x<W;++x){int sx=int((x-hW)/scX+shift+hW);
            if(fl)sx=W-1-sx;
            if(sx>=0&&sx<W){auto px=s.at<cv::Vec3b>(y,sx);float sh=.5f+.5f*scX;
                dr[x*3+0]=dsp::clamp8(px[0]*sh);dr[x*3+1]=dsp::clamp8(px[1]*sh);dr[x*3+2]=dsp::clamp8(px[2]*sh);}}}}

static void fx_kaleidoscope(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3);
    int seg=6+int(p.depth*14); float segA=2.f*kPI/seg,cx=W*.5f,cy=H*.5f;
    for(int y=0;y<H;++y){uchar*dr=d.ptr(y);
        for(int x=0;x<W;++x){float dx=x-cx,dy=y-cy,r=std::sqrt(dx*dx+dy*dy);
            float th=std::atan2(dy,dx),s2=std::fmod(th,segA);
            if(s2<0)s2+=segA;if(s2>segA*.5f)s2=segA-s2;
            auto px=dsp::bsample(s,cx+r*std::cos(s2+p.phase*kPI),cy+r*std::sin(s2+p.phase*kPI));
            dr[x*3+0]=px[0];dr[x*3+1]=px[1];dr[x*3+2]=px[2];}}}

static void fx_shatter(const cv::Mat& s,cv::Mat& d,const Params& p){
    int W=s.cols,H=s.rows; d=cv::Mat(H,W,CV_8UC3,cv::Scalar(0,0,0));
    int gs=std::max(8,64-int(p.amount*56));float sa=p.depth*30.f;
    std::mt19937 rg(42);std::uniform_real_distribution<float> uu(-sa,sa);
    for(int gy=0;gy<H;gy+=gs)for(int gx=0;gx<W;gx+=gs){
        float ox=uu(rg),oy=uu(rg);int ew=std::min(gs,W-gx),eh=std::min(gs,H-gy);
        int dx=std::clamp(int(gx+ox),0,W-ew),dy=std::clamp(int(gy+oy),0,H-eh);
        cv::Mat(s,{gx,gy,ew,eh}).copyTo(cv::Mat(d,{dx,dy,ew,eh}));}}

static void apply(const cv::Mat& s,cv::Mat& d,const Params& p){
    switch(p.type){
        case Type::Tumble:       fx_tumble(s,d,p);break;
        case Type::PageTurn:     fx_pageturn(s,d,p);break;
        case Type::Ripple:       fx_ripple(s,d,p);break;
        case Type::Sphere:       fx_sphere(s,d,p);break;
        case Type::Squeeze:      fx_squeeze(s,d,p);break;
        case Type::Mosaic:       fx_mosaic(s,d,p);break;
        case Type::Trails:       fx_trails(s,d,p);break;
        case Type::Mirror:       fx_mirror(s,d,p);break;
        case Type::CubeSpin:     fx_cubespin(s,d,p);break;
        case Type::Kaleidoscope: fx_kaleidoscope(s,d,p);break;
        case Type::Shatter:      fx_shatter(s,d,p);break;
        default:                 fx_none(s,d,p);break;}
    if(p.mix<.999f&&p.type!=Type::None)
        cv::addWeighted(d,p.mix,s,1.f-p.mix,0.f,d);}
} // Effects

// ============================================================
//  §6  Global audio clock
// ============================================================
static std::atomic<int64_t> g_audioSamplePos{0};
static std::atomic<float>   g_sourceFPS{kNTSC_FPS};  // default to NTSC 29.97
static std::atomic<bool>    g_videoReset{false};

// ============================================================
//  §7  ProcessParams & Pipeline
// ============================================================
enum class ExportFormat { H264_MP4, FFV1_MKV, FFV1_AVI };
struct ExportContext {
    bool              active = false;
    ExportFormat      format = ExportFormat::H264_MP4;
    std::string       path;
    cv::VideoWriter   writer;
    int               frameCount = 0;
    std::string       tempVideoPath;
    std::string       tempAudioPath;
};

struct ProcessParams {
    std::mutex        mu;
    Effects::Params   fx;
    std::atomic<bool> ntscEnabled{true};
    EngineParams      epSnap;
    VideoParams       vpSnap;
    float             tapeSpd{1.f};
    float             instantSpd{1.f};
    bool              engValid{false};
    std::atomic<int>  spdIdx{0};
    ExportContext*    exPtr=nullptr; // Pointer to AppState.exportCtx
};

class ProcessingPipeline {
public:
    FrameQueue<cv::Mat> rawQ{Q_DEPTH};
    FrameQueue<cv::Mat> outQ{Q_DEPTH};
    NTSCSimulator       ntsc_;

    void start(ProcessParams& pp){running_=true;thread_=std::thread([this,&pp]{loop(pp);});}
    void stop(){running_=false;rawQ.push(cv::Mat());if(thread_.joinable())thread_.join();}
    float fps()const{return fps_.load();}

private:
    std::atomic<bool>  running_{false};
    std::thread        thread_;
    std::atomic<float> fps_{0.f};
    int                frameNum_{0};

    void loop(ProcessParams& pp){
        auto tPrev=std::chrono::steady_clock::now();int fpsCount=0;
        ntsc_.initBuffers(FW,FH);
        cv::Mat last_raw;
        auto loop_timer = std::chrono::steady_clock::now();
        while(running_){
            auto now_time = std::chrono::steady_clock::now();
            long long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - loop_timer).count();
            if (elapsed < 33) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            loop_timer = now_time;

            cv::Mat raw; 
            if(!rawQ.try_pop(raw)) {
                if(last_raw.empty()) continue;
                raw = last_raw;
            } else {
                last_raw = raw;
                if(raw.empty()) continue;
            }
            // ── Crop to 4:3 (center-crop) ─────────────────────────────────
            {
                int sw = raw.cols, sh = raw.rows;
                float srcAR = float(sw) / float(sh);
                constexpr float kAR43 = 4.f / 3.f;
                if (std::abs(srcAR - kAR43) > 0.01f) {
                    int cw, ch;
                    if (srcAR > kAR43) { // wider — crop sides
                        ch = sh;
                        cw = int(sh * kAR43);
                    } else {             // taller — crop top/bottom
                        cw = sw;
                        ch = int(sw / kAR43);
                    }
                    int ox = (sw - cw) / 2;
                    int oy = (sh - ch) / 2;
                    raw = raw(cv::Rect(ox, oy, cw, ch)).clone();
                }
            }
            Effects::Params fx; bool ntscOn; EngineParams ep{}; VideoParams vp{}; float spd=1.f; float iSpd=1.f;
            bool hasEng=false; int si=0;
            {std::lock_guard<std::mutex> lk(pp.mu);
             fx=pp.fx;ntscOn=pp.ntscEnabled.load();ep=pp.epSnap;vp=pp.vpSnap;spd=pp.tapeSpd;iSpd=pp.instantSpd;
             hasEng=pp.engValid;si=pp.spdIdx.load();}
            if(!hasEng){switch(si){case 1:ep.ips_base=7.5f;break;case 2:ep.ips_base=3.75f;break;default:ep.ips_base=15.f;}}
            static float ph=0.f; ph+=.01f; if(ph>1.f)ph-=1.f; fx.phase=ph;
            cv::Mat eff,proc;
            Effects::apply(raw,eff,fx);
            if(ntscOn){
                if(!hasEng){EngineParams def{};def.ips_base=15.f;def.hiss=.003f;def.wow_dep=.3f;def.flutter_dep=.08f;
                    ntsc_.process(eff,proc,frameNum_,def,vp,1.f,1.f);}
                else ntsc_.process(eff,proc,frameNum_,ep,vp,spd,iSpd);}
            else proc=eff;
            
            // ── EXPORT ENGINE ──────────────────────────────────────────
            if (pp.exPtr && pp.exPtr->active && pp.exPtr->writer.isOpened()) {
                pp.exPtr->writer.write(proc);
                pp.exPtr->frameCount++;
            }

            outQ.push(std::move(proc));++frameNum_;++fpsCount;
            auto now=std::chrono::steady_clock::now();float dt=std::chrono::duration<float>(now-tPrev).count();
            if(dt>=1.f){fps_=fpsCount/dt;fpsCount=0;tPrev=now;}}
    }
};

// ============================================================
//  §8  Capture Thread — NTSC-locked, audio-clock-synced
// ============================================================
enum class SourceType{Demo=0,Camera,File};

class CaptureThread {
public:
    std::atomic<SourceType> sourceType{SourceType::Demo};
    std::mutex*  fileMuPtr=nullptr;
    std::string* filePathPtr=nullptr;
    std::function<void(const std::string&)> onAudioFileReady;

    void start(FrameQueue<cv::Mat>& q){running_=true;thread_=std::thread([this,&q]{loop(q);});}
    void stop(){running_=false;if(thread_.joinable())thread_.join();}
    float fps()const{return fps_.load();}

private:
    std::atomic<bool>  running_{false};
    std::thread        thread_;
    std::atomic<float> fps_{0.f};
    DemoSource         demo_;

    void loop(FrameQueue<cv::Mat>& outQ){
        cv::VideoCapture cap; SourceType lastSrc=SourceType::Demo; std::string lastFile;
        auto tPrev=std::chrono::steady_clock::now(); int fpsCount=0;
        // Source FPS: demo runs at NTSC, camera/file use detected rate
        double srcFPS = kNTSC_FPS;
        auto wallClock = std::chrono::steady_clock::now();

        while(running_){
            SourceType src=sourceType.load(); std::string fp;
            if(fileMuPtr&&filePathPtr){std::lock_guard<std::mutex> lk(*fileMuPtr);fp=*filePathPtr;}

            if(src!=lastSrc||(src==SourceType::File&&fp!=lastFile)){
                cap.release();
                if(src==SourceType::Camera){
                    cap.open(0);
                    if(!cap.isOpened()){sourceType=SourceType::Demo;src=SourceType::Demo;}
                    else{cap.set(cv::CAP_PROP_FRAME_WIDTH,FW);cap.set(cv::CAP_PROP_FRAME_HEIGHT,FH);
                         srcFPS=cap.get(cv::CAP_PROP_FPS);if(srcFPS<=0)srcFPS=kNTSC_FPS;}
                }else if(src==SourceType::File&&!fp.empty()){
                    // Extract audio BEFORE opening the video device to avoid locking
                    std::string wav=extractAudioToWav(fp);
                    if(!running_.load()) return; // App shutting down
                    
                    cap.open(fp);
                    if(!cap.isOpened()){
                        if(fileMuPtr){std::lock_guard<std::mutex> lk(*fileMuPtr);}
                        sourceType.store(SourceType::Demo); src=SourceType::Demo;
                    } else {
                         if(!running_.load()) return;
                         // ONLY NOW start the audio engine, after the video device is definitely locked and ready
                         if(!wav.empty()&&onAudioFileReady) onAudioFileReady(wav);
                         
                         srcFPS=cap.get(cv::CAP_PROP_FPS);if(srcFPS<=0)srcFPS=kNTSC_FPS;
                         g_sourceFPS.store(float(srcFPS));g_videoReset.store(true);
                    }
                }
                lastSrc=src;lastFile=fp;wallClock=std::chrono::steady_clock::now();}

            if(g_videoReset.exchange(false)){
                if(cap.isOpened())cap.set(cv::CAP_PROP_POS_FRAMES,0);
                g_audioSamplePos.store(0);}

            cv::Mat frame;
            if(src==SourceType::Demo){
                // NTSC-locked wall-clock pacing: generate at 29.97 fps
                auto now=std::chrono::steady_clock::now();
                float el=std::chrono::duration<float,std::milli>(now-wallClock).count();
                float iv=1000.f/float(srcFPS);
                if(el<iv) std::this_thread::sleep_for(std::chrono::microseconds(int((iv-el)*1000)));
                wallClock=std::chrono::steady_clock::now();
                frame=demo_.generate(FW,FH);
            }else if(cap.isOpened()){
                // Audio-clock-driven video seek
                int64_t apos=g_audioSamplePos.load();
                double spf=AUDIO_SR/srcFPS;
                int64_t targetFrame=int64_t(double(apos)/spf);
                int64_t curFrame=int64_t(cap.get(cv::CAP_PROP_POS_FRAMES));
                int64_t delta=targetFrame-curFrame;
                if(delta>4) cap.set(cv::CAP_PROP_POS_FRAMES,double(targetFrame));
                else if(delta<-1){std::this_thread::sleep_for(std::chrono::milliseconds(4));continue;}
                if(!cap.read(frame)||frame.empty()){
                    cap.set(cv::CAP_PROP_POS_FRAMES,0);g_audioSamplePos.store(0);continue;}
                cv::resize(frame,frame,{FW,FH});
            }else{std::this_thread::sleep_for(std::chrono::milliseconds(10));continue;}

            outQ.push(frame);++fpsCount;
            auto now2=std::chrono::steady_clock::now();float dt=std::chrono::duration<float>(now2-tPrev).count();
            if(dt>=1.f){fps_=fpsCount/dt;fpsCount=0;tPrev=now2;}}
    }
};

// ============================================================
//  §9  SDL Texture wrapper
// ============================================================
struct SDLTexture{
    SDL_Texture*tex=nullptr;int w=0,h=0;
    void create(SDL_Renderer*r,int W,int H){
        if(tex)SDL_DestroyTexture(tex);w=W;h=H;
        tex=SDL_CreateTexture(r,SDL_PIXELFORMAT_BGR24,SDL_TEXTUREACCESS_STREAMING,W,H);}
    void update(const cv::Mat&f){
        if(!tex||f.empty())return;cv::Mat rs;
        if(f.cols!=w||f.rows!=h)cv::resize(f,rs,{w,h});else rs=f;
        void*px;int pitch;SDL_LockTexture(tex,nullptr,&px,&pitch);
        for(int y=0;y<h;++y)memcpy((uchar*)px+y*pitch,rs.ptr(y),w*3);
        SDL_UnlockTexture(tex);}
    ~SDLTexture(){if(tex)SDL_DestroyTexture(tex);}
};

// ============================================================
//  §10  AppState
// ============================================================
struct AppState{
    CaptureThread      capture;
    ProcessingPipeline pipeline;
    ProcessParams      pp;
    SDLTexture         outTex,rawTex;
    std::mutex         frameMu;
    cv::Mat            lastOut,lastRaw;
    std::mutex         fileMu;
    std::string        filePath;
    std::mutex         audioMu; // Protects tapeEngine and audioIO
    std::unique_ptr<TapeEngine> tapeEngine;
    std::unique_ptr<AudioIO>    audioIO;
    std::string        audioWavPath;
    ExportContext      exportCtx;
    int   selectedFx=0,tapeSpeedIdx=0;
    bool  ntscEnabled=true;
    float vuL=0.f,vuR=0.f;
    uint64_t startMs=SDL_GetTicks64();
    VideoParams videoParams;
    EngineParams baseParams; // UI state

    void setParam(std::function<void(EngineParams&)> fn){
        fn(baseParams);
    }
};

// ============================================================
//  §11  80s Broadcast-Centre UI
// ============================================================
static void styleBroadcast(){
    ImGuiStyle& s=ImGui::GetStyle();
    s.WindowRounding=s.FrameRounding=s.GrabRounding=s.PopupRounding=0.f;
    s.ScrollbarRounding=s.TabRounding=0.f;
    s.FrameBorderSize=s.WindowBorderSize=1.f;
    s.ItemSpacing={5,3};s.FramePadding={5,2};
    s.WindowPadding={8,6};s.ScrollbarSize=10.f;s.GrabMinSize=8.f;
    ImVec4*c=s.Colors;
    c[ImGuiCol_WindowBg]           ={.08f,.08f,.08f,1.f};
    c[ImGuiCol_ChildBg]            ={.06f,.06f,.06f,1.f};
    c[ImGuiCol_PopupBg]            ={.10f,.10f,.10f,1.f};
    c[ImGuiCol_Border]             ={.30f,.30f,.30f,1.f};
    c[ImGuiCol_FrameBg]            ={.12f,.12f,.12f,1.f};
    c[ImGuiCol_FrameBgHovered]     ={.18f,.18f,.18f,1.f};
    c[ImGuiCol_FrameBgActive]      ={.08f,.08f,.08f,1.f};
    c[ImGuiCol_TitleBg]            ={.06f,.06f,.06f,1.f};
    c[ImGuiCol_TitleBgActive]      ={.10f,.10f,.10f,1.f};
    c[ImGuiCol_SliderGrab]         ={1.f,.70f,.00f,1.f};
    c[ImGuiCol_SliderGrabActive]   ={1.f,.85f,.20f,1.f};
    c[ImGuiCol_CheckMark]          ={.00f,.90f,.46f,1.f};
    c[ImGuiCol_Button]             ={.18f,.18f,.18f,1.f};
    c[ImGuiCol_ButtonHovered]      ={.26f,.26f,.26f,1.f};
    c[ImGuiCol_ButtonActive]       ={.10f,.10f,.10f,1.f};
    c[ImGuiCol_Header]             ={.16f,.16f,.16f,1.f};
    c[ImGuiCol_HeaderHovered]      ={.22f,.22f,.22f,1.f};
    c[ImGuiCol_HeaderActive]       ={.10f,.10f,.10f,1.f};
    c[ImGuiCol_ScrollbarBg]        ={.05f,.05f,.05f,1.f};
    c[ImGuiCol_ScrollbarGrab]      ={.28f,.28f,.28f,1.f};
    c[ImGuiCol_Tab]                ={.12f,.12f,.12f,1.f};
    c[ImGuiCol_TabHovered]         ={.22f,.22f,.22f,1.f};
    c[ImGuiCol_TabActive]          ={.20f,.20f,.20f,1.f};
    c[ImGuiCol_Text]               ={.90f,.88f,.82f,1.f};
    c[ImGuiCol_TextDisabled]       ={.38f,.38f,.38f,1.f};
    c[ImGuiCol_Separator]          ={.30f,.30f,.30f,1.f};
}

static void sectionHdr(const char* lbl){
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4{1.f,.70f,.00f,1.f});
    ImGui::Separator(); ImGui::PopStyleColor();
    ImGui::TextColored({1.f,.70f,.00f,1.f},"▶ %s",lbl);
    ImGui::Separator(); ImGui::Spacing();}

static void vuBar(float lv,float w,float h,bool clip=false){
    ImVec2 p=ImGui::GetCursorScreenPos(); ImDrawList*dl=ImGui::GetWindowDrawList();
    dl->AddRectFilled(p,{p.x+w,p.y+h},IM_COL32(20,20,20,255));
    float f=std::clamp(lv,0.f,1.f)*w;
    ImU32 col=clip?IM_COL32(255,30,30,255):lv>.7f?IM_COL32(255,160,0,255):IM_COL32(0,200,80,255);
    if(f>.01f)dl->AddRectFilled(p,{p.x+f,p.y+h},col);
    dl->AddRect(p,{p.x+w,p.y+h},IM_COL32(90,90,90,255));
    ImGui::Dummy({w,h});}

static void monitorFrame(SDL_Texture*tex,float w,float h,const char*label,ImU32 ledCol){
    ImVec2 p=ImGui::GetCursorScreenPos(); ImDrawList*dl=ImGui::GetWindowDrawList();
    dl->AddRectFilled(p,{p.x+w+8,p.y+h+22},IM_COL32(18,18,18,255));
    dl->AddRect(p,{p.x+w+8,p.y+h+22},IM_COL32(60,60,60,255),0,0,2.f);
    dl->AddRectFilled({p.x+4,p.y+4},{p.x+w+4,p.y+h+4},IM_COL32(0,0,0,255));
    ImGui::SetCursorScreenPos({p.x+4,p.y+4});
    if(tex) ImGui::Image((ImTextureID)(intptr_t)tex,{w,h});
    else{ImGui::Dummy({w,h});
         dl->AddText({p.x+w*.5f-22,p.y+h*.5f-6},IM_COL32(50,50,50,255),"NO SIGNAL");}
    ImVec2 lb={p.x,p.y+h+6};
    dl->AddRectFilled(lb,{lb.x+w+8,lb.y+16},IM_COL32(12,12,12,255));
    dl->AddCircleFilled({lb.x+8,lb.y+8},4.f,ledCol);
    dl->AddText({lb.x+16,lb.y+3},IM_COL32(180,160,60,255),label);
    ImGui::SetCursorScreenPos(p); ImGui::Dummy({w+8,h+24});}

static std::string timecode(uint64_t s0){
    uint64_t ms=SDL_GetTicks64()-s0; char b[32];
    std::snprintf(b,sizeof(b),"%02llu:%02llu:%02llu:%02llu",
        (unsigned long long)(ms/3600000)%24,(unsigned long long)(ms/60000)%60,
        (unsigned long long)(ms/1000)%60,(unsigned long long)(ms%1000)/33);
    return b;}

static void drawUI(AppState& app, SDL_Renderer* ren){
    const ImGuiViewport*vp=ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(vp->Pos); ImGui::SetNextWindowSize(vp->Size);
    ImGui::Begin("##MAIN",nullptr,
        ImGuiWindowFlags_NoDecoration|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoBringToFrontOnFocus|ImGuiWindowFlags_NoScrollbar);
    const float W=vp->Size.x, H=vp->Size.y;

    // ── HEADER BAR ───────────────────────────────────────────
    ImGui::PushStyleColor(ImGuiCol_ChildBg,ImVec4{.04f,.04f,.04f,1.f});
    ImGui::BeginChild("##hdr",{0,30},false,ImGuiWindowFlags_NoScrollbar);
    ImGui::SetCursorPosY(5);
    ImGui::TextColored({.20f,.90f,1.f,1.f},"■ ADO-8500"); ImGui::SameLine();
    ImGui::TextColored({1.f,.70f,0.f,1.f},"BROADCAST VIDEO EFFECTS PROCESSOR"); ImGui::SameLine();
    ImGui::TextDisabled("  |  "); ImGui::SameLine();
    ImGui::TextColored({1.f,.70f,0.f,1.f},"%s",timecode(app.startMs).c_str()); ImGui::SameLine();
    float pls=.5f+.5f*std::sin(SDL_GetTicks64()*.003f);
    ImGui::TextColored({0.f,pls*.9f,0.f,1.f},"  ◉ PGM");
    ImGui::EndChild(); ImGui::PopStyleColor();
    ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4{.55f,.45f,.00f,1.f});
    ImGui::Separator(); ImGui::PopStyleColor();

    // ── MONITOR WALL ─────────────────────────────────────────
    const float monH=H*.36f;
    ImGui::BeginChild("##mons",{0,monH},false,ImGuiWindowFlags_NoScrollbar);
    {
        cv::Mat outF,rawF;
        {std::lock_guard<std::mutex> lk(app.frameMu); outF=app.lastOut; rawF=app.lastRaw;}
        if(!outF.empty())app.outTex.update(outF);

        float pgW=W*.40f, pgH=monH-32;
        float lpls=.4f+.6f*pls;
        monitorFrame(app.outTex.tex,pgW,pgH,"PGM  OUTPUT",IM_COL32(0,int(lpls*220),0,255));
        ImGui::SameLine(0,12);

        float srcW=pgW*.52f, srcH=srcW/ASPECT;
        if(!rawF.empty())app.rawTex.update(rawF);
        monitorFrame(app.rawTex.tex,srcW,srcH,"SRC  INPUT",IM_COL32(50,150,255,255));
        ImGui::SameLine(0,12);

        // ── NTSC WAVEFORM SCOPE ──────────────────────────────
        // Reconstructs the NTSC composite signal for multiple
        // scanlines of the output frame and draws them as a
        // phosphor-persistence waveform monitor trace.
        // Regions: front-porch | H-sync | burst | active video
        // Graticule: −40 / 0 / 7.5 / 75 / 100 IRE
        {ImVec2 cp=ImGui::GetCursorScreenPos(); ImDrawList*dl=ImGui::GetWindowDrawList();
         const float aw=srcW*1.05f, ah=srcH;
         // Bezel + screen
         dl->AddRectFilled(cp,{cp.x+aw+8,cp.y+ah+22},IM_COL32(14,14,14,255));
         dl->AddRect(cp,{cp.x+aw+8,cp.y+ah+22},IM_COL32(55,55,55,255),0,0,2.f);
         const float sx0=cp.x+4, sy0=cp.y+4, sx1=sx0+aw, sy1=sy0+ah;
         dl->AddRectFilled({sx0,sy0},{sx1,sy1},IM_COL32(0,8,3,255));

         // IRE → screen Y  (top = high IRE)
         constexpr float kIlo=-50.f, kIhi=120.f, kIspan=170.f;
         auto ireY=[&](float ire)->float{
             return sy1-(ire-kIlo)/kIspan*ah;};

         // ── Graticule lines ──────────────────────────────
         struct GL{float ire;const char*lbl;};
         static constexpr GL kG[]={{100.f,"100"},{75.f,"75"},{7.5f,"7.5"},{0.f,"0"},{-40.f,"-40"}};
         for(auto&g:kG){
             float gy=ireY(g.ire);
             dl->AddLine({sx0,gy},{sx1,gy},IM_COL32(0,50,16,255),1.f);
             dl->AddText({sx0+2,gy-9},IM_COL32(0,100,36,220),g.lbl);}

         // ── Timing fractions (63.556 µs line period) ─────
         constexpr float kT=63.556f;
         constexpr float fFP  = 1.500f/kT;
         constexpr float fSYE = (1.500f+4.700f)/kT;
         constexpr float fBRE = (1.500f+4.700f+0.600f)/kT;
         constexpr float fBUE = (1.500f+4.700f+0.600f+2.500f)/kT;
         constexpr float fACT = (1.500f+4.700f+0.600f+2.500f+1.600f)/kT;
         auto xf=[&](float f)->float{return sx0+f*aw;};
         // Region dividers
         for(float fx:{fFP,fSYE,fBRE,fBUE,fACT})
             dl->AddLine({xf(fx),sy0},{xf(fx),sy1},IM_COL32(0,32,10,200),1.f);
         // Region labels
         const float ly=sy0+1.f;
         dl->AddText({sx0+1,ly},     IM_COL32(0,75,28,200),"FP");
         dl->AddText({xf(fFP)+1,ly}, IM_COL32(0,75,28,200),"SYNC");
         dl->AddText({xf(fBRE)+1,ly},IM_COL32(0,75,28,200),"BST");
         dl->AddText({xf(fACT)+1,ly},IM_COL32(0,75,28,200),"ACTIVE");

         // ── Build & draw waveform traces ──────────────────
         // Sample N_LINES evenly-spaced rows from outF.
         // Each row is encoded as a full NTSC composite line
         // (sync + burst + active video) and drawn as a trace.
         if(!outF.empty()){
             constexpr int N_LINES=16, N_PX=480;
             const int FH2=outF.rows, FW2=outF.cols;
             // Subcarrier step over active region:
             // 188.5 cycles / (N_PX * fACT fraction) pixels
             const int activePx=std::max(1,int((1.f-fACT)*N_PX));
             const float scStep=2.f*3.14159265f*188.5f/float(activePx);
             // Front-porch/sync/burst pixel boundaries
             const int pFP =int(fFP *N_PX);
             const int pSYE=int(fSYE*N_PX);
             const int pBRE=int(fBRE*N_PX);
             const int pBUE=int(fBUE*N_PX);
             const int pACT=int(fACT*N_PX);
             for(int li=0;li<N_LINES;++li){
                 int srcY=int(float(li)/float(N_LINES)*float(FH2));
                 srcY=std::clamp(srcY,0,FH2-1);
                 const uchar* row=outF.ptr(srcY);
                 // brightness: middle lines brightest
                 float mid=float(li)/float(N_LINES-1)-0.5f;
                 uint8_t br=uint8_t(70+140*(1.f-std::abs(mid)*1.6f));
                 ImU32 col=IM_COL32(0,br,uint8_t(br*0.3f),200);
                 float prevX=sx0, prevY=ireY(0.f);
                 for(int px=0;px<N_PX;++px){
                     float ire=0.f;
                     if(px<pFP){
                         ire=0.f;                        // front porch
                     }else if(px<pSYE){
                         ire=-40.f;                      // H-sync tip
                     }else if(px<pBRE){
                         ire=0.f;                        // breezeway
                     }else if(px<pBUE){
                         // colour burst ±20 IRE
                         float bp=float(px-pBRE)/float(pBUE-pBRE)*2.f*3.14159265f*9.f;
                         ire=20.f*std::sin(bp);
                     }else if(px<pACT){
                         ire=0.f;                        // back porch
                     }else{
                         // active video: Y + I·cos(θ) − Q·sin(θ) → IRE
                         float xSrc=float(px-pACT)/float(activePx)*float(FW2);
                         xSrc=std::max(0.f,std::min(float(FW2-1.001f),xSrc));
                         int x0=int(xSrc); float xfr=xSrc-x0;
                         int x1=std::min(x0+1,FW2-1);
                         float r=(row[x0*3+2]*(1-xfr)+row[x1*3+2]*xfr)/255.f;
                         float g2=(row[x0*3+1]*(1-xfr)+row[x1*3+1]*xfr)/255.f;
                         float b2=(row[x0*3+0]*(1-xfr)+row[x1*3+0]*xfr)/255.f;
                         float Y=.2990f*r+.5870f*g2+.1140f*b2;
                         float I=.5957f*r-.2744f*g2-.3213f*b2;
                         float Q=.2115f*r-.5227f*g2+.3112f*b2;
                         float theta=float(px-pACT)*scStep;
                         float comp=Y+I*std::cos(theta)-Q*std::sin(theta);
                         ire=comp*100.f+7.5f;
                         ire=std::clamp(ire,-40.f,120.f);
                     }
                     float nx=sx0+float(px)/float(N_PX)*aw;
                     float ny=std::clamp(ireY(ire),sy0,sy1);
                     if(px>0) dl->AddLine({prevX,prevY},{nx,ny},col,1.f);
                     prevX=nx; prevY=ny;
                 }
             }
         }
         // Bright 0-IRE blanking reference
         {float gy=ireY(0.f);
          dl->AddLine({sx0,gy},{sx1,gy},IM_COL32(0,80,26,255),1.f);}

         // Bezel label
         ImVec2 lb={cp.x,cp.y+ah+6};
         dl->AddRectFilled(lb,{lb.x+aw+8,lb.y+16},IM_COL32(12,12,12,255));
         dl->AddCircleFilled({lb.x+8,lb.y+8},4.f,IM_COL32(0,200,80,255));
         dl->AddText({lb.x+16,lb.y+3},IM_COL32(180,160,60,255),"WFM  NTSC COMPOSITE");
         ImGui::SetCursorScreenPos(cp);ImGui::Dummy({aw+8,ah+24});}
        ImGui::SameLine(0,12);

        // System status readout monitor
        {ImVec2 cp=ImGui::GetCursorScreenPos(); ImDrawList*dl=ImGui::GetWindowDrawList();
         float sw2=std::max(80.f,W-cp.x+vp->Pos.x-24), sh2=srcH;
         dl->AddRectFilled(cp,{cp.x+sw2+8,cp.y+sh2+22},IM_COL32(18,18,18,255));
         dl->AddRect(cp,{cp.x+sw2+8,cp.y+sh2+22},IM_COL32(60,60,60,255),0,0,2.f);
         dl->AddRectFilled({cp.x+4,cp.y+4},{cp.x+sw2+4,cp.y+sh2+4},IM_COL32(0,0,10,255));
         ImGui::SetCursorScreenPos({cp.x+8,cp.y+6});
         ImGui::BeginChild("##sysstat",{sw2-6,sh2-8},false,ImGuiWindowFlags_NoScrollbar);
         ImGui::PushStyleColor(ImGuiCol_ChildBg,ImVec4{0,0,.04f,1.f});
         ImGui::TextColored({1.f,.70f,0.f,1.f},"── SYSTEM ──");
         ImGui::Text("CAP  %5.1f fps", (double)app.capture.fps());
         ImGui::Text("PROC %5.1f fps", (double)app.pipeline.fps());
         ImGui::Text("RAW-Q  %2zu  OUT-Q  %2zu",app.pipeline.rawQ.size(),app.pipeline.outQ.size());
         ImGui::Spacing();
         ImGui::TextColored({1.f,.70f,0.f,1.f},"── A/V SYNC ──");
         int64_t ap=g_audioSamplePos.load();
         float sf=g_sourceFPS.load();
         int64_t vf=sf>0?int64_t(double(ap)/(AUDIO_SR/sf)):0;
         ImGui::Text("APOS  %8lld smp",(long long)ap);
         ImGui::Text("VFRM  %8lld",(long long)vf);
         ImGui::Text("SFPS  %6.2f  (NTSC %.2f)",(double)sf,kNTSC_FPS);
         
         bool locked = false;
         { std::lock_guard<std::mutex> a_lk(app.audioMu);
           locked=app.tapeEngine&&app.audioIO&&app.audioIO->is_open(); }
         
         ImGui::Spacing();
         if(locked) ImGui::TextColored({0.f,.9f,.4f,1.f},"● A/V LOCKED");
         else        ImGui::TextColored({.9f,.3f,.3f,1.f},"○ FREE-RUN");
         ImGui::PopStyleColor(); ImGui::EndChild();
         ImVec2 lb={cp.x,cp.y+sh2+6};
         dl->AddRectFilled(lb,{lb.x+sw2+8,lb.y+16},IM_COL32(12,12,12,255));
         dl->AddCircleFilled({lb.x+8,lb.y+8},4.f,locked?IM_COL32(0,220,100,255):IM_COL32(200,50,50,255));
         dl->AddText({lb.x+16,lb.y+3},IM_COL32(180,160,60,255),"SYS  STATUS");
         ImGui::SetCursorScreenPos(cp);ImGui::Dummy({0,0});}
    }
    ImGui::EndChild();

    ImGui::PushStyleColor(ImGuiCol_Separator,ImVec4{.55f,.45f,.00f,1.f});
    ImGui::Separator(); ImGui::PopStyleColor();

    // ── CONTROL SURFACE ──────────────────────────────────────
    const float ctrlH=H-monH-72;
    ImGui::BeginChild("##ctrl",{0,ctrlH});
    const float colW=(W-24)/3.f;

    // ─── COL 1: INPUT + EFFECTS ──────────────────────────────
    ImGui::BeginChild("##c1",{colW,ctrlH-4},true);
    sectionHdr("INPUT SOURCE");
    auto stopAudioAndClear = [&app](){
        std::lock_guard<std::mutex> lk(app.audioMu);
        std::string oldWav;
        { std::lock_guard<std::mutex> flk(app.fileMu); oldWav = app.audioWavPath; app.audioWavPath=""; }
        
        if(app.audioIO){app.audioIO->stop(); app.audioIO->close();}
        app.audioIO.reset(); app.tapeEngine.reset();
        
        if(!oldWav.empty()) std::remove(oldWav.c_str());
        
        app.pipeline.rawQ.clear(); app.pipeline.outQ.clear();
        std::lock_guard<std::mutex> lk2(app.pp.mu); app.pp.engValid=false;
    };

    auto srcBtnWithStop=[&](const char* lbl,SourceType st){
        SourceType cur = app.capture.sourceType.load();
        if(cur==st) ImGui::PushStyleColor(ImGuiCol_Button,ImVec4{0.f,.22f,.08f,1.f});
        if(ImGui::Button(lbl,{colW-18,0})){
            if(cur!=st){ stopAudioAndClear(); app.capture.sourceType=st; }
        }
        if(cur==st) ImGui::PopStyleColor();};

    srcBtnWithStop("◉  SMPTE TEST SIGNAL",SourceType::Demo);
    srcBtnWithStop("◉  CAMERA INPUT",SourceType::Camera);
    if(ImGui::Button("◉  OPEN VIDEO FILE ...",{colW-18,0})){
        stopAudioAndClear();
        std::string cmd;
#ifdef _WIN32
        cmd="powershell -Command \"Add-Type -AssemblyName System.Windows.Forms;$d=New-Object Windows.Forms.OpenFileDialog;$d.Filter='Video|*.mp4;*.avi;*.mkv;*.mov;*.webm|All|*.*';if($d.ShowDialog()-eq'OK'){Write-Output $d.FileName}\"";
#elif defined(__APPLE__)
        cmd="osascript -e 'POSIX path of (choose file of type {\"mp4\",\"avi\",\"mkv\",\"mov\",\"webm\"})'";
#else
        cmd="zenity --file-selection --title='Open Video File' --file-filter='*.mp4 *.avi *.mkv *.mov *.webm' 2>/dev/null";
#endif
        FILE*pipe=popen(cmd.c_str(),"r");
        if(pipe){char buf[512];if(fgets(buf,sizeof(buf),pipe)){
            std::string path=buf;path.erase(path.find_last_not_of(" \n\r\t")+1);
            if(!path.empty()){std::lock_guard<std::mutex> lk(app.fileMu);
                app.filePath=path;app.capture.sourceType=SourceType::File;}}
            pclose(pipe);}
    }
    sectionHdr("EFFECT SELECT");
    for(int i=0;i<(int)Effects::Type::COUNT;++i){
        bool sel=app.selectedFx==i;
        if(sel)ImGui::PushStyleColor(ImGuiCol_Button,ImVec4{.25f,.17f,0.f,1.f});
        if(ImGui::Button(Effects::kNames[i],{(colW-18)*.5f-3,0})){
            app.selectedFx=i;std::lock_guard<std::mutex> lk(app.pp.mu);
            app.pp.fx.type=(Effects::Type)i;Effects::s_trail=cv::Mat();}
        if(sel)ImGui::PopStyleColor();
        if(i%2==0)ImGui::SameLine();}
    sectionHdr("EFFECT PARAMS");
    {Effects::Params fxL;{std::lock_guard<std::mutex> lk(app.pp.mu);fxL=app.pp.fx;}
     bool ch=false;
     auto sl=[&](const char*n,float*v,float lo,float hi){
         ImGui::SetNextItemWidth(colW-100);ch|=ImGui::SliderFloat(n,v,lo,hi);};
     sl("AMOUNT",&fxL.amount,0,1);sl("SPEED",&fxL.speed,0,1);
     sl("DEPTH",&fxL.depth,0,1);sl("MIX",&fxL.mix,0,1);
     if(ch){std::lock_guard<std::mutex> lk(app.pp.mu);
         app.pp.fx.amount=fxL.amount;app.pp.fx.speed=fxL.speed;
         app.pp.fx.depth=fxL.depth;app.pp.fx.mix=fxL.mix;}}
    ImGui::EndChild();ImGui::SameLine();

    // ─── COL 2: VIDEO PROCESSING ─────────────────────────────
    ImGui::BeginChild("##c2",{colW,ctrlH-4},true);
    sectionHdr("VIDEO PROCESSING");
    ImGui::Checkbox("NTSC/VHS SIMULATION",&app.ntscEnabled);
    {std::lock_guard<std::mutex> lk(app.pp.mu);app.pp.ntscEnabled=app.ntscEnabled;}
    ImGui::Text("TAPE SPEED:"); ImGui::SameLine();
    bool spCh=false;
    if(ImGui::RadioButton("SP",&app.tapeSpeedIdx,0))spCh=true; ImGui::SameLine();
    if(ImGui::RadioButton("LP",&app.tapeSpeedIdx,1))spCh=true; ImGui::SameLine();
    if(ImGui::RadioButton("EP",&app.tapeSpeedIdx,2))spCh=true;
    if(spCh){
        // Auto-set IPS from tape speed preset (SP=15, LP=7.5, EP=3.75)
        static const float kIPS[] = {15.f, 7.5f, 3.75f};
        app.baseParams.ips_base = kIPS[app.tapeSpeedIdx];
        std::lock_guard<std::mutex> lk(app.pp.mu);app.pp.spdIdx=app.tapeSpeedIdx;
    }
    { std::lock_guard<std::mutex> a_lk(app.audioMu);
    if(app.tapeEngine){
        bool tCh=false;
        auto sl3=[&](const char*n,float*v,float lo,float hi){
            ImGui::SetNextItemWidth(colW-130);tCh|=ImGui::SliderFloat(n,v,lo,hi);};
        EngineParams& bp=app.baseParams;
        sectionHdr("TRANSPORT");
        sl3("WOW",&bp.wow_dep,0,5);sl3("FLUTTER",&bp.flutter_dep,0,1);
        sl3("MOTOR HLTH",&bp.motor_health,0,.9f);
        sl3("MOTOR DRAG",&bp.motor_drag,0,.9f);sl3("DROPOUT",&bp.dropout_rate,0,.1f);
        sectionHdr("VCR GEOMETRY WARPING");
        sl3("TRACKING OFF", &app.videoParams.tracking_error, 0.f, 1.f);
        sl3("TAPE CREASE",  &app.videoParams.tape_crease, 0.f, 1.f);
        sl3("H/V HOLD FAIL", &app.videoParams.sync_hold_failure, 0.f, 1.f);
        sectionHdr("MAGNETIC");
        sl3("DEMAGNETIZE",&bp.demagnetization,0,1);sl3("PRINT-THRU",&bp.print_through,0,.5f);
        sl3("AZIMUTH",&bp.azimuth_drift,0,1);sl3("CROSSTALK",&bp.crosstalk,0,.3f);
        sectionHdr("ELECTRONICS");
        sl3("TAPE HISS",&bp.hiss,0,.02f);sl3("HEAD BUMP",&bp.head_bump,0,1);
        sl3("HF CUTOFF",&bp.cutoff_base,1000,20000);
        if(tCh){
            std::lock_guard<std::mutex> lk2(app.pp.mu);
            app.pp.epSnap=bp;app.pp.tapeSpd=bp.tape_speed_mult;app.pp.engValid=true;}
        
        float vL=0, vR=0;
        if(app.audioIO){ vL=app.audioIO->get_level_left(); vR=app.audioIO->get_level_right(); }
        app.vuL = vL; app.vuR = vR;
    }else{ImGui::Spacing();ImGui::TextColored({.9f,.3f,.3f,1.f},"Load video file to enable tape DSP");}
    }
    
    // Cleanup old audio files if they accumulate?
    // Let's do a simple cleanup on switch in stopAudioAndClear instead.
    if(!app.audioWavPath.empty()){
        ImGui::SetCursorPos({15,ctrlH-55});
        std::string wavName;
        { std::lock_guard<std::mutex> flk(app.fileMu); wavName = app.audioWavPath; }
        ImGui::TextColored({.5f,.5f,.5f,1.f},"AUDIO: %s",wavName.c_str());
    }
    ImGui::EndChild();ImGui::SameLine();

    // ─── COL 3: PRESETS + METERS ─────────────────────────────
    ImGui::BeginChild("##c3",{colW,ctrlH-4},true);
    sectionHdr("TAPE PRESETS");
    struct PR{const char*n;std::function<void(EngineParams&)>fn;};
    static const PR prs[]={
        {"CLEAN DIGITAL",[](EngineParams&p){p={};p.ips_base=15;p.hiss=.001f;p.cutoff_base=20000;}},
        {"VHS  SP",      [](EngineParams&p){p={};p.ips_base=15;p.wow_dep=.5f;p.flutter_dep=.15f;p.head_bump=.4f;p.hiss=.003f;p.cutoff_base=12000;}},
        {"VHS  LP",      [](EngineParams&p){p={};p.ips_base=7.5f;p.wow_dep=1.5f;p.flutter_dep=.3f;p.head_bump=.6f;p.hiss=.006f;p.cutoff_base=8000;p.print_through=.1f;}},
        {"VHS  EP",      [](EngineParams&p){p={};p.ips_base=3.75f;p.wow_dep=3.f;p.flutter_dep=.6f;p.motor_health=.6f;p.head_bump=.8f;p.hiss=.01f;p.cutoff_base=5000;p.demagnetization=.4f;p.print_through=.2f;}},
        {"WORN  EP",     [](EngineParams&p){p={};p.ips_base=3.75f;p.wow_dep=4.f;p.flutter_dep=.8f;p.motor_health=.75f;p.sticky_shed=.08f;p.dropout_rate=.05f;p.demagnetization=.7f;p.cutoff_base=3000;p.hiss=.015f;}},
        {"METAL TAPE",   [](EngineParams&p){p={};p.ips_base=15;p.bias=1.5f;p.drive=1.5f;p.hiss=.0005f;p.cutoff_base=18000;p.oxide_type="Metal";}},
        {"DEGRADED",     [](EngineParams&p){p={};p.ips_base=7.5f;p.wow_dep=4.f;p.flutter_dep=.8f;p.motor_health=.8f;p.sticky_shed=.08f;p.oxide_shedding=.3f;p.dropout_rate=.05f;p.demagnetization=.7f;p.cutoff_base=3000;p.hiss=.015f;}},
    };
    for(auto&pr:prs){
        if(ImGui::Button(pr.n,{colW-18,0})){
            app.setParam(pr.fn);
            std::lock_guard<std::mutex> lk2(app.pp.mu);app.pp.epSnap=app.baseParams;app.pp.tapeSpd=app.baseParams.tape_speed_mult;app.pp.engValid=true;}}
    
    sectionHdr("AUDIO METERS");
    ImGui::Text("L:"); ImGui::SameLine(); vuBar(app.vuL,colW-46,10,app.vuL>.95f);
    ImGui::Text("R:"); ImGui::SameLine(); vuBar(app.vuR,colW-46,10,app.vuR>.95f);
    
    sectionHdr("RENDER EXPORT");
    static int fmtIdx = 0;
    const char* fmts[] = {"MP4 (H.264 Lossy)", "MKV (FFV1 Lossless)", "AVI (FFV1 Lossless)"};
    ImGui::SetNextItemWidth(colW-18);
    ImGui::Combo("##fmt", &fmtIdx, fmts, 3);
    
    if (!app.exportCtx.active) {
        if (ImGui::Button("START RECORDING TO DISK", {colW-18, 0})) {
            app.exportCtx.format = (ExportFormat)fmtIdx;
            std::string ext = (fmtIdx == 0) ? ".mp4" : (fmtIdx == 1 ? ".mkv" : ".avi");
            app.exportCtx.tempVideoPath = "temp_render" + ext;
            app.exportCtx.tempAudioPath = "temp_audio_proc.wav";
            
            int fourcc;
            if (fmtIdx == 0) fourcc = cv::VideoWriter::fourcc('a','v','c', '1');
            else fourcc = cv::VideoWriter::fourcc('F','F','V','1');
            
            if (app.exportCtx.writer.open(app.exportCtx.tempVideoPath, fourcc, kNTSC_FPS, {FW, FH})) {
                app.exportCtx.active = true;
                app.exportCtx.frameCount = 0;
                
                // Start capturing processed audio!
                std::lock_guard<std::mutex> a_lk(app.audioMu);
                if (app.audioIO) app.audioIO->start_capture(app.exportCtx.tempAudioPath);
                
                SDL_Log("Export: Started recording to %s", app.exportCtx.tempVideoPath.c_str());
            } else {
                SDL_Log("Export: Failed to open VideoWriter");
            }
        }
    } else {
        ImGui::PushStyleColor(ImGuiCol_Button, {.8f, 0, 0, 1});
        if (ImGui::Button("STOP & FINISH EXPORT", {colW-18, 40})) {
            app.exportCtx.active = false;
            app.exportCtx.writer.release();
            
            // Stop audio capture
            { std::lock_guard<std::mutex> a_lk(app.audioMu);
              if (app.audioIO) app.audioIO->stop_capture(); }
            
            std::string outName = "render_export_" + std::to_string(SDL_GetTicks()) + 
                                  ((app.exportCtx.format == ExportFormat::H264_MP4) ? ".mp4" : 
                                   (app.exportCtx.format == ExportFormat::FFV1_MKV ? ".mkv" : ".avi"));
            
            char muxCmd[4096];
            // Use the processed RAW audio now!
            bool hasAudio = false;
            std::FILE* test = std::fopen(app.exportCtx.tempAudioPath.c_str(), "rb");
            if (test) {
                std::fseek(test, 0, SEEK_END);
                long sz = std::ftell(test);
                std::fclose(test);
                if (sz > 1024) hasAudio = true;
                else SDL_Log("Export: Audio capture file too small (%ld bytes)", sz);
            }
            
            if (hasAudio) {
                std::snprintf(muxCmd, sizeof(muxCmd),
                    "ffmpeg -y -i \"%s\" -i \"%s\" -c:v copy -c:a aac -b:a 192k -aspect 4:3 \"%s\"",
                    app.exportCtx.tempVideoPath.c_str(), app.exportCtx.tempAudioPath.c_str(), outName.c_str());
            } else {
                std::snprintf(muxCmd, sizeof(muxCmd),
                    "ffmpeg -y -i \"%s\" -c:v copy -aspect 4:3 \"%s\"",
                    app.exportCtx.tempVideoPath.c_str(), outName.c_str());
            }
            
            SDL_Log("Export: Muxing final output (with processed audio)...");
            if (std::system(muxCmd) == 0) {
                SDL_Log("Export: SUCCESS -> %s", outName.c_str());
                std::remove(app.exportCtx.tempVideoPath.c_str());
                std::remove(app.exportCtx.tempAudioPath.c_str());
            } else {
                SDL_Log("Export: Muxing error. Raw files preserved.");
            }
        }
        ImGui::PopStyleColor();
        ImGui::ProgressBar(float(app.exportCtx.frameCount % 100) / 100.f, {colW-18, 0}, "RECORDING...");
    }
    
    ImGui::EndChild();
    
    // ─── BOTTOM READOUTS ─────────────────────────────────────
    sectionHdr("SYNC STATUS");
    {int64_t ap=g_audioSamplePos.load();float sf=g_sourceFPS.load();
     int64_t vf=sf>0?int64_t(double(ap)/(AUDIO_SR/sf)):0;
     ImGui::Text("AUDIO POS: %8lld smp",(long long)ap);
     ImGui::Text("VIDEO FRM: %8lld",(long long)vf);
     ImGui::Text("SRC FPS:   %6.2f  (NTSC %.2f)",(double)sf,kNTSC_FPS);
     ImGui::Text("PROC FPS:  %6.1f",(double)app.pipeline.fps());
     ImGui::Text("H-SYNC:    %.2f Hz",kNTSC_HSYNC);
     ImGui::Text("V-SYNC:    %.2f Hz",kNTSC_FPS*2.f);
     bool locked = false;
     { std::lock_guard<std::mutex> a_lk(app.audioMu);
       locked=app.tapeEngine&&app.audioIO&&app.audioIO->is_open(); }
     if(locked)ImGui::TextColored({0.f,.9f,.4f,1.f},"● A/V LOCKED");
     else       ImGui::TextColored({.9f,.3f,.3f,1.f},"○ FREE-RUN");}
    ImGui::EndChild();
    ImGui::EndChild();
    ImGui::End();
}

// ============================================================
//  §12  main()
// ============================================================
int main(int,char**){
    if(SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER|SDL_INIT_AUDIO)<0){
        SDL_Log("SDL_Init: %s",SDL_GetError());return 1;}
    SDL_Window*win=SDL_CreateWindow(
        "ADO-8500  ·  BROADCAST VIDEO EFFECTS PROCESSOR",
        SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
        1520,900,SDL_WINDOW_RESIZABLE|SDL_WINDOW_ALLOW_HIGHDPI);
    if(!win){SDL_Log("Window: %s",SDL_GetError());return 1;}
    SDL_Renderer*ren=SDL_CreateRenderer(win,-1,SDL_RENDERER_ACCELERATED);
    if(!ren){SDL_Log("Renderer: %s",SDL_GetError());return 1;}

    IMGUI_CHECKVERSION(); ImGui::CreateContext();
    ImGuiIO&io=ImGui::GetIO();
    io.ConfigFlags|=ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename=nullptr; io.Fonts->AddFontDefault();
    styleBroadcast();
    ImGui_ImplSDL2_InitForSDLRenderer(win,ren);
    ImGui_ImplSDLRenderer2_Init(ren);

    AppState app;
    app.outTex.create(ren,FW,FH);
    app.rawTex.create(ren,FW/2,FH/2);

    app.capture.onAudioFileReady=[&app](const std::string&wav){
        auto eng=std::make_unique<TapeEngine>(42);
        if(!eng->load_file(wav)){ SDL_Log("TapeEngine: Failed to load %s", wav.c_str()); return; }
        
        auto aio=std::make_unique<AudioIO>(*eng);
        if(!aio->open()){ SDL_Log("AudioIO: Failed to open"); return; }
        aio->play_forward(); eng->is_playing.store(true);
        
        {
            std::lock_guard<std::mutex> lk(app.audioMu);
            // Safely stop and replace the current engine/IO
            if(app.audioIO){ app.audioIO->stop(); app.audioIO->close(); }
            
            { std::lock_guard<std::mutex> flk(app.fileMu); app.audioWavPath=wav; }
            app.tapeEngine=std::move(eng); 
            app.audioIO=std::move(aio);
            g_audioSamplePos.store(0);
            app.baseParams=app.tapeEngine->params;

            std::lock_guard<std::mutex> lk2(app.pp.mu);
            app.pp.epSnap=app.baseParams;
            app.pp.tapeSpd=app.baseParams.tape_speed_mult;
            app.pp.engValid=true;
            
            // Cleanup the temporary WAV file AFTER we're sure the engine is running
            // Actually, we can't delete it while TapeEngine is using it via StreamBuffer.
            // StreamBuffer keeps the file open.
        }
        SDL_Log("Audio: engine started, %d samples",app.tapeEngine->total_samples);};

    app.capture.fileMuPtr=&app.fileMu;
    app.capture.filePathPtr=&app.filePath;
    app.pp.exPtr=&app.exportCtx; 
    app.capture.start(app.pipeline.rawQ);
    app.pipeline.start(app.pp);

    std::atomic<bool> drainRun{true};
    std::thread drainThread([&]{
        while(drainRun){
            cv::Mat f; if(app.pipeline.outQ.pop(f,20)){
                std::lock_guard<std::mutex> lk(app.frameMu);
                app.lastOut=std::move(f);}}});

    bool running=true; SDL_Event evt;
    auto lastPoll=std::chrono::steady_clock::now();

    while(running){
        // Advance audio clock from wall-clock
        {auto now=std::chrono::steady_clock::now();
         lastPoll=now;
         
         std::lock_guard<std::mutex> a_lk(app.audioMu);
         if(app.tapeEngine&&app.audioIO&&app.audioIO->is_open()){
             EngineParams active = app.baseParams;
             // ─── Diegetic HiFi Loss ───────────────────────────────────────
             // Compute signal quality from the RAW base params only — NOT from
             // `active`, which is already modified by prior physics steps.
             // This breaks the feedback loop that caused runaway dropout.
             float signal_loss = std::clamp(
                 app.baseParams.dropout_rate * 4.0f
                 + app.baseParams.demagnetization * 0.6f
                 + (app.videoParams.tracking_error * 2.0f),
                 0.0f, 1.0f);
             static float hifi_blend_lpf = 0.0f;
             hifi_blend_lpf += (signal_loss - hifi_blend_lpf) * 0.04f;
             float hifi_blend = hifi_blend_lpf;

             // ── Phase 1: Drum Interference Buzz (partial HiFi loss) ────────
             if (hifi_blend > 0.05f) {
                 float buzz_t = (float)g_audioSamplePos.load() / AUDIO_SR;
                 float buzz_env = hifi_blend * (1.0f - hifi_blend) * 4.0f;
                 active.print_through += buzz_env * 0.15f;
                 active.hiss += buzz_env * 0.006f;
                 // Tiny rhythmic dropout on every 60Hz drum cycle beat.
                 // Kept very small (0.015) to avoid flooding CapstanVar's
                 // dropout scheduler and causing audio silence.
                 float buzz_phase = std::fmod(buzz_t * 60.0f, 1.0f);
                 if (buzz_phase > 0.85f)
                     active.dropout_rate = std::min(active.dropout_rate + buzz_env * 0.015f, 0.09f);
             }

             // ── Phase 2: HiFi → Linear Mono crossfade ─────────────────────
             if (hifi_blend > 0.1f) {
                 float t = std::clamp((hifi_blend - 0.1f) / 0.9f, 0.0f, 1.0f);
                 active.azimuth_drift = std::max(active.azimuth_drift, t);
                 float linear_cutoff = 8000.0f - t * 3000.0f;
                 active.cutoff_base = std::min(active.cutoff_base, linear_cutoff);
                 active.hiss += t * 0.010f;
             }
             float trk = app.videoParams.tracking_error;
             if (trk > 0.05f) {
                 active.hiss += trk * 0.015f; 
                 active.cutoff_base *= std::max(0.02f, 1.0f - trk * 0.95f);
                 active.azimuth_drift += trk; 
                 // Cap tracking dropout to avoid flooding CapstanVar's scheduler
                 active.dropout_rate = std::min(active.dropout_rate + trk * 0.05f, 0.09f);
             }
             float crease = app.videoParams.tape_crease;
             if (crease > 0.01f) {
                 float t_sec = (float)g_audioSamplePos.load() / AUDIO_SR;
                 float cp = std::fmod(t_sec * 0.1f, 1.0f);
                 if (cp > 0.4f && cp < 0.6f) { 
                     active.dropout_rate += crease * 0.5f;
                     active.flutter_dep += crease * 2.0f;
                 }
             }

             float mh = app.baseParams.motor_health;
             if (mh > 0.1f) {
                 active.wow_dep += mh * 2.5f; // Drastic pitch sag 
                 active.flutter_dep += mh * 0.3f; // Grinding motor bearings
             }

             float spd=1.f;
             {std::lock_guard<std::mutex> lk(app.tapeEngine->lock);
              // Preserve transport state from the AudioIO thread
              float m_spd = app.tapeEngine->params.tape_speed_mult;
              float m_eng = app.tapeEngine->params.motor_engage;
              bool  m_rev = app.tapeEngine->params.is_reversed;
              app.tapeEngine->params = active;
              app.tapeEngine->params.tape_speed_mult = m_spd;
              app.tapeEngine->params.motor_engage    = m_eng;
              app.tapeEngine->params.is_reversed     = m_rev;
              
              spd=std::clamp(m_spd,.01f,4.f);}
             g_audioSamplePos.store(int64_t(app.tapeEngine->play_head));
             int64_t total=app.tapeEngine->total_samples;
             if(total>0&&g_audioSamplePos.load()>total) g_audioSamplePos.store(0);
             {std::lock_guard<std::mutex> lk(app.pp.mu);
              app.pp.epSnap=active;
              app.pp.vpSnap=app.videoParams;
              app.pp.tapeSpd=spd; app.pp.instantSpd=app.tapeEngine->transport.last_instant_speed; app.pp.engValid=true;}}}

        // Copy lastOut to lastRaw for source monitor
        {std::lock_guard<std::mutex> lk(app.frameMu);
         if(!app.lastOut.empty()) app.lastRaw=app.lastOut;}

        while(SDL_PollEvent(&evt)){
            ImGui_ImplSDL2_ProcessEvent(&evt);
            if(evt.type==SDL_QUIT)running=false;
            if(evt.type==SDL_WINDOWEVENT&&evt.window.event==SDL_WINDOWEVENT_CLOSE
               &&evt.window.windowID==SDL_GetWindowID(win)) running=false;}

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI(app,ren);
        ImGui::Render();
        SDL_SetRenderDrawColor(ren,8,8,8,255);
        SDL_RenderClear(ren);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(ren);
    }

    drainRun=false; drainThread.join();
    app.pipeline.stop(); app.capture.stop();
    if(app.audioIO){app.audioIO->stop();app.audioIO->close();}
    if(!app.audioWavPath.empty()) std::remove(app.audioWavPath.c_str());
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(ren); SDL_DestroyWindow(win); SDL_Quit();
    return 0;
}
