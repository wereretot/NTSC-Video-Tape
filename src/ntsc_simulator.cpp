#include "ntsc_simulator.h"
#include <SDL2/SDL.h>

std::atomic<double> g_wallTimeSec{0.0};

void NTSCSimulator::initBuffers(int w, int h) {
    W_ = w;
    H_BLANK = 144;
    W_TOTAL = w + H_BLANK;
    kSC_PX = (2.0f * kPI * kSC_FREQ) / (float)W_;
    for (int i = 0; i < 4; ++i) prev_line_luma_[i] = 0.0f;
}

float NTSCSimulator::generateSnow(float time, int x, int y, std::mt19937& rng) {
    float snow = 0.0f;
    for (int i = 0; i < 4; ++i) {
        snow_phase_[i] = std::fmod(snow_phase_[i] + 0.016f * (0.5f + i * 0.3f), 1.0f);
        float nx = float(x) * 0.1f + float(y) * 0.07f;
        snow += std::sin(snow_phase_[i] * 2.0f * kPI + nx) * 0.25f;
    }
    snow += std::uniform_real_distribution<float>(-0.5f, 0.5f)(rng);
    return snow * 0.5f;
}

void NTSCSimulator::process(const cv::Mat& in, cv::Mat& out, int frameNum,
                            const EngineParams& ep, const VideoParams& vp,
                            float tapeSpd, float instantSpd, float wallDt,
                            int current_recording_id) {
    const int W = in.cols, H = in.rows;
    if (W == 0 || H == 0) {
        SDL_Log("[NTSC] CRASH GUARD: empty input frame %d", frameNum);
        return;
    }
    if (W != W_) initBuffers(W, H);

    wallTime_ = (float)g_wallTimeSec.load();

    if (current_recording_id != last_recording_id_) {
        last_recording_id_ = current_recording_id;
        recording_transition_time_ = 1.0f;
    }
    if (recording_transition_time_ > 0.0f) {
        recording_transition_time_ -= wallDt * 2.0f;
        if (recording_transition_time_ < 0.0f) recording_transition_time_ = 0.0f;
        recording_blend_ = recording_transition_time_;
    }

    float signal_strength = vp.signal_strength;
    float snow_amount = 0.0f;

    float rf_level = 1.0f;
    float chroma_attenuation = 1.0f;
    float luma_noise_level = 0.0f;
    float chroma_noise_level = 0.0f;

    float tracking_deg = std::clamp(vp.tracking_error, 0.0f, 1.0f);
    float dropout_deg = std::clamp(vp.dropout_rate * 10.0f, 0.0f, 1.0f);
    float motor_deg = std::clamp(vp.motor_health, 0.0f, 1.0f);
    float crease_deg = std::clamp(vp.tape_crease, 0.0f, 1.0f);
    float oxide_deg = std::clamp(vp.oxide_shedding, 0.0f, 1.0f);
    float demag_deg = std::clamp(vp.demagnetization, 0.0f, 1.0f);
    float sticky_deg = std::clamp(vp.sticky_shed, 0.0f, 1.0f);
    float age_deg = std::clamp(vp.tape_age, 0.0f, 1.0f);
    float sync_deg = std::clamp(vp.sync_hold_failure, 0.0f, 1.0f);

    float helical_sweep_deg = std::clamp(vp.helical_sweep, 0.0f, 1.0f);
    float head_switch_jitter_deg = std::clamp(vp.head_switch_jitter, 0.0f, 1.0f);
    float fm_carrier_noise_deg = std::clamp(vp.fm_carrier_noise, 0.0f, 1.0f);
    float chroma_crosstalk_deg = std::clamp(vp.chroma_crosstalk, 0.0f, 1.0f);
    float inter_field_phase_deg = std::clamp(vp.inter_field_phase_error, 0.0f, 1.0f);
    float head_pre_echo_deg = std::clamp(vp.head_pre_echo, 0.0f, 1.0f);
    float drum_ecc_deg = std::clamp(vp.drum_eccentricity, 0.0f, 1.0f);

    rf_level = 1.0f - tracking_deg * 0.7f
                     - dropout_deg * 0.3f
                     - oxide_deg * 0.4f
                     - crease_deg * 0.5f
                     - age_deg * 0.3f;

    chroma_attenuation = 1.0f - tracking_deg * 0.6f
                               - motor_deg * 0.3f
                               - demag_deg * 0.4f
                               - oxide_deg * 0.2f;

    luma_noise_level = tracking_deg * 0.5f
                      + dropout_deg * 0.3f
                      + oxide_deg * 0.4f
                      + age_deg * 0.3f
                      + sticky_deg * 0.2f;

    chroma_noise_level = tracking_deg * 0.6f
                        + motor_deg * 0.4f
                        + demag_deg * 0.5f
                        + oxide_deg * 0.3f;

    if (rf_level < 0.99f) {
        snow_amount = (1.0f - rf_level) * 1.2f;
        snow_amount += sync_deg * 0.6f;
        snow_amount += luma_noise_level * 0.4f;
        snow_amount = std::clamp(snow_amount, 0.0f, 0.95f);
    }

    if (signal_strength < 0.99f) {
        snow_amount = std::max(snow_amount, (1.0f - signal_strength) * 1.0f);
    }

    float tracking_rf_mask = rf_level;
    if (tracking_deg > 0.1f) {
        float rf_penalty = tracking_deg * tracking_deg * 0.5f;
        tracking_rf_mask = rf_level * (1.0f - rf_penalty);
    }

    int currentField = (frameNum % 2);

    if (currentField != prev_field_) {
        std::mt19937 fieldRng(uint32_t(frameNum) * 13u);
        field_chroma_phase_accum_ = std::fmod(
            field_chroma_phase_accum_ + inter_field_phase_deg * 0.15f
            + std::normal_distribution<float>(0, inter_field_phase_deg * 0.08f)(fieldRng),
            2.0f * kPI);
        if (head_switch_jitter_deg > 0.01f) {
            head_switch_offset_ = std::normal_distribution<float>(0, head_switch_jitter_deg * 3.0f)(fieldRng);
        } else {
            head_switch_offset_ = 0.0f;
        }
        prev_field_ = currentField;
    }

    drum_ecc_phase_ = std::fmod(drum_ecc_phase_ + wallDt * (kVHS_DRUM_RPM / 60.0f), 1.0f);
    float drum_ecc_wobble = drum_ecc_deg * std::sin(drum_ecc_phase_ * 2.0f * kPI) * 2.0f;

    adjacent_track_phase_ = std::fmod(adjacent_track_phase_ + wallDt * 0.1f, 1.0f);

    const float fPhase = std::fmod(wallTime_ * (kNTSC_FPS * 2.f), 4.f) * (kPI * 0.5f);

    static constexpr float WOW_HZ[]    = {0.3f, 0.8f, 2.1f};
    static constexpr float FLUTTER_HZ[] = {15.0f, 23.0f, 30.0f, 46.0f, 67.0f};
    static constexpr float WOW_AMP[]    = {0.5f, 0.3f, 0.2f};
    static constexpr float FLUTTER_AMP[] = {0.25f, 0.20f, 0.20f, 0.18f, 0.17f};

    float wow_sum = 0.0f;
    for (int i = 0; i < 3; ++i) {
        wow_phase_[i] = std::fmod(wow_phase_[i] + wallDt * WOW_HZ[i], 1.0f);
        wow_sum += WOW_AMP[i] * std::sin(wow_phase_[i] * 2.0f * kPI);
    }

    float flutter_sum = 0.0f;
    for (int i = 0; i < 5; ++i) {
        flutter_phase_[i] = std::fmod(flutter_phase_[i] + wallDt * FLUTTER_HZ[i], 1.0f);
        flutter_sum += FLUTTER_AMP[i] * std::sin(flutter_phase_[i] * 2.0f * kPI);
    }

    float wow_dev = wow_sum * ep.wow_dep * 0.01f;
    float flutter_dev = flutter_sum * ep.flutter_dep * 0.008f;

    float speed_error = std::abs(1.0f - instantSpd);
    float target_error = speed_error + vp.tracking_error;
    if (target_error > 2.0f) target_error = 2.0f;

    float dt = wallDt;
    float lpfA = 1.f - std::exp(-dt * 3.f);
    tracking_error_lpf_ += (target_error - tracking_error_lpf_) * lpfA;

    if (target_error > 0.01f || vp.sync_hold_failure > 0.0f) {
        float roll_hz = (target_error * 7.5f) + (vp.sync_hold_failure * 10.f);
        v_roll_accum_ = std::sin(wallTime_ * roll_hz * kPI * 2.f) * (target_error * 480.f + vp.sync_hold_failure * 600.f);
    } else {
        v_roll_accum_ *= std::exp(-dt * 10.f);
        if (std::abs(v_roll_accum_) < 1.0f) v_roll_accum_ = 0.0f;
    }

    float v_jitter = 0.0f;
    if (target_error > 0.1f) {
        std::mt19937 jRng(uint32_t(frameNum) * 999u);
        v_jitter = std::normal_distribution<float>(0, target_error * 10.0f)(jRng);
    }

    int v_roll = (int(v_roll_accum_ + v_jitter)) % H;
    if (v_roll < 0) v_roll += H;

    float noise_bar_rate = target_error * 0.8f * kNTSC_FPS;
    noise_bar_phase_ = std::fmod(wallTime_ * noise_bar_rate, 1.0f);
    if (noise_bar_phase_ < 0.0f) noise_bar_phase_ += 1.0f;
    float bar_y = noise_bar_phase_ * float(H);
    bar_y += adjacent_track_phase_ * tracking_deg * 8.0f;
    float bar_width = 20.0f + tracking_error_lpf_ * 80.0f;

    float h_freq_err = (target_error * 12.0f) + (vp.sync_hold_failure * 60.0f);
    if (h_freq_err > 0.01f) {
        h_roll_phase_ = std::fmod(h_roll_phase_ - wallDt * h_freq_err, 1.0f);
        if (h_roll_phase_ < 0.0f) h_roll_phase_ += 1.0f;
    } else {
        h_roll_phase_ *= std::exp(-wallDt * 8.0f);
        if (std::abs(h_roll_phase_) < 0.001f) h_roll_phase_ = 0.0f;
    }

    float safeTapeSpd = std::max(tapeSpd, 0.001f);
    float speed_dev = std::abs(instantSpd - safeTapeSpd) / safeTapeSpd;

    float motor_defect = std::clamp(ep.motor_health, 0.0f, 1.0f);

    float motor_cog = 0.0f;
    if (motor_defect > 0.05f) {
        float cog_hz = 3.7f;
        float cog_phase = std::fmod(tapeTime_ * cog_hz, 1.0f);
        float pole_frac = std::fmod(cog_phase * 8.0f, 1.0f);
        motor_cog = std::pow(pole_frac, 0.3f) * motor_defect * 8.0f;
    }

    float drum_wobble_phase = 0.0f;
    if (motor_defect > 0.05f) {
        drum_wobble_phase = std::fmod(tapeTime_ * 30.0f, 1.0f) * 2.0f * kPI;
    }

    float motor_vibration = 0.0f;
    if (motor_defect > 0.05f) {
        float motor_rpm_hz = 1800.0f / 60.0f;
        motor_vibration = motor_defect * 2.0f * (
            std::sin(tapeTime_ * motor_rpm_hz * 2.0f * kPI) * 0.6f +
            std::sin(tapeTime_ * motor_rpm_hz * 4.0f * kPI) * 0.3f);
    }

    float fast_tbe = speed_dev * 18.0f;
    mechanical_wow_ = speed_dev * 120.0f;

    float tape_tension_impulse = 0.0f;
    {
        float impulse_rate = speed_dev * 4.0f + 0.5f;
        float impulse_phase = std::fmod(tapeTime_ * impulse_rate, 1.0f);
        if (impulse_phase < 0.05f) {
            tape_tension_impulse = (impulse_phase / 0.05f) * speed_dev * 35.0f;
        } else {
            tape_tension_impulse = std::exp(-(impulse_phase - 0.05f) * 8.0f) * speed_dev * 35.0f;
        }
    }

    afc_error_ = mechanical_wow_ * 0.4f
               + (speed_dev * 400.0f)
               + fast_tbe
               + tape_tension_impulse
               + motor_cog * 50.0f
               + motor_vibration * 15.0f;

    float crease_y = std::fmod(tapeTime_ * 0.1f, 1.0f) * float(H);

    float head_sweep_px_per_line = helical_sweep_deg * 1.5f;
    float head_sweep_grad = head_sweep_px_per_line / float(W);

    float hsw_boundary_y = float(H) - kVHS_HSW_LINE * float(H) / float(kLINES_PER_FIELD);
    hsw_boundary_y += head_switch_offset_;

    out.create(H, W, CV_8UC3);

    std::vector<float> prev_comp_line;
    prev_comp_line.resize(W_TOTAL, 0.0f);
    std::vector<float> curr_comp_line;
    curr_comp_line.resize(W_TOTAL, 0.0f);
    bool first_line = true;

    for (int y = 0; y < H; ++y) {
        std::mt19937 rng(uint32_t(frameNum) * 65537u + uint32_t(y) * 1013u);
        int src_y = (y + v_roll) % H;
        const uchar* sr = in.ptr(src_y);
        uchar* dr = out.ptr(y);

        float scrape_flutter = std::normal_distribution<float>(0, ep.flutter_dep * 0.4f)(rng);

        float line_afc = afc_error_ * std::exp(-float(y) * 0.08f);

        float guide_bounce = 0.0f;
        for (int bounce_idx = 0; bounce_idx < 3; ++bounce_idx) {
            float bounce_freq = 2.3f + bounce_idx * 1.7f;
            float bounce_y = std::fmod(tapeTime_ * bounce_freq + float(frameNum) * 0.07f * (bounce_idx + 1), 1.0f) * float(H);
            float dist_bounce = std::abs(float(y) - bounce_y);
            if (dist_bounce > (float)H / 2.0f) dist_bounce = (float)H - dist_bounce;
            if (dist_bounce < 15.0f && (speed_dev > 0.005f || ep.flutter_dep > 0.05f)) {
                float intensity = std::pow(1.0f - dist_bounce / 15.0f, 2.5f);
                float attack = intensity * (speed_dev * 30.0f + ep.flutter_dep * 8.0f);
                guide_bounce += attack * std::exp(-dist_bounce * 0.4f);
            }
        }

        float total_h_warp = line_afc + mechanical_wow_ + scrape_flutter + guide_bounce;

        float drag_hz = 0.5f + ep.motor_drag * 2.0f;
        float drag_phase = std::fmod(tapeTime_ * drag_hz, 1.0f) * H;
        float dy = float(y) - drag_phase;
        float beltSlip = 0.0f;
        if (dy > 0.0f && dy < 100.0f && ep.motor_drag > 0.01f) {
            beltSlip = std::exp(-dy * 0.05f) * std::sin(dy * 0.2f) * (ep.motor_drag * 20.0f);
        }

        float drum_wobble_tbe = 0.0f;
        if (motor_defect > 0.05f) {
            float drum_wobble_amp = motor_defect * 1.5f;
            drum_wobble_tbe = drum_wobble_amp * std::sin(drum_wobble_phase + float(y) * 0.05f);
        }
        total_h_warp += drum_wobble_tbe;

        float drum_ecc_offset = drum_ecc_wobble * std::sin(float(y) * 0.03f + drum_ecc_phase_ * 2.0f * kPI);
        total_h_warp += drum_ecc_offset;

        float hsTearing = 0.0f;
        float hs_line_offset = motor_defect * 12.0f * std::sin(tapeTime_ * 1.5f * 2.0f * kPI);
        int hs_start = std::max(0, H - int(kHS_VBI_LINES) + int(hs_line_offset));

        float dist_to_hsw = float(y) - hsw_boundary_y;
        if (dist_to_hsw > -3.0f && dist_to_hsw < 8.0f) {
            float hsw_intensity = 0.0f;
            if (dist_to_hsw < 0.0f) {
                hsw_intensity = std::pow(1.0f + dist_to_hsw / 3.0f, 2.0f);
            } else {
                hsw_intensity = std::exp(-dist_to_hsw * 0.5f);
            }
            hsw_intensity *= head_switch_jitter_deg;
            float hsw_shear = hsw_intensity * 25.0f * std::sin(float(y) * 1.7f + tapeTime_ * 120.0f);
            float hsw_noise = hsw_intensity * std::normal_distribution<float>(0, 0.4f)(rng);
            hsTearing += hsw_shear + hsw_noise * 10.0f;
            total_h_warp += hsw_shear * 0.3f;
        }

        if (y >= hs_start) {
            float hsDepth = (float(y) - float(hs_start)) / float(kHS_VBI_LINES);
            float hs_gain = 12.0f + motor_defect * 20.0f;
            hsTearing += std::pow(hsDepth, 2.0f) * hs_gain;
            hsTearing += std::normal_distribution<float>(0, 1 + motor_defect * 8)(rng);
        }

        float motor_rf_noise = 0.0f;
        if (motor_defect > 0.05f) {
            motor_rf_noise = motor_vibration * 0.05f;
            tracking_rf *= std::max(0.7f, 1.0f - motor_rf_noise);
        }

        float dist_rf = std::abs(float(src_y) - bar_y);
        if (dist_rf > (float)H / 2.0f) dist_rf = (float)H - dist_rf;
        float tracking_rf = 1.0f;
        if (tracking_error_lpf_ > 0.05f && dist_rf < bar_width) {
            tracking_rf = dist_rf / bar_width;
        }

        float dist_crease = std::abs(float(src_y) - crease_y);
        if (dist_crease > (float)H / 2.0f) dist_crease = (float)H - dist_crease;
        float crease_shear = 0.0f;
        float crease_dropout_prob = 0.0f;

        if (vp.tape_crease > 0.01f && dist_crease < 35.0f) {
            float intensity = std::pow(1.0f - (dist_crease / 35.0f), 1.5f);
            float shear_wave = std::sin(float(y) * 0.3f + wallTime_ * 28.0f) * 15.0f
                             + std::cos(float(y) * 0.5f - wallTime_ * 43.0f) * 8.0f;
            crease_shear = intensity * vp.tape_crease * (50.0f + shear_wave);
            tracking_rf *= std::max(0.05f, 1.0f - intensity * vp.tape_crease * 3.0f);
            crease_dropout_prob = intensity * vp.tape_crease * 0.6f;
        }

        float current_h_roll = h_roll_phase_ * 0.1f;
        float h_sync_roll = current_h_roll * float(W_TOTAL) * (target_error + vp.sync_hold_failure);

        float h_sync_shear = 0.0f;
        if (tracking_error_lpf_ > 0.01f || vp.tape_crease > 0.01f) {
             float chaotic_shear = std::sin(src_y * 0.22f + wallTime_ * 35.0f) * 0.7f
                                 + std::cos(src_y * 0.35f - wallTime_ * 52.0f) * 0.5f
                                 + std::sin(src_y * 0.11f + wallTime_ * 19.0f) * 0.4f
                                 + std::sin(src_y * 0.47f - wallTime_ * 71.0f) * 0.25f
                                 + std::normal_distribution<float>(0, 1)(rng) * 0.6f;
             float rf_penalty = (1.0f - tracking_rf) * 5.0f + 1.5f;
             h_sync_shear = (tracking_error_lpf_ * 100.0f) * chaotic_shear * rf_penalty;
        }

        float abe_start = total_h_warp + beltSlip + hsTearing + h_sync_roll + h_sync_shear + crease_shear;

        float bend_fraction = std::exp(-float(y) * 0.012f);
        float wow_bend = wow_dev * float(W) * 0.3f * bend_fraction;
        float flutter_bend = flutter_dev * float(W) * 0.08f * bend_fraction;
        abe_start += wow_bend + flutter_bend;

        float abe_slope = scrape_flutter * 0.25f;
        if (dy > 0.0f && dy < 100.0f) {
            abe_slope += std::exp(-dy * 0.05f) * std::cos(dy * 0.2f) * (ep.motor_drag * 25.0f);
        }

        abe_slope += head_sweep_grad;

        const float linePhase = fPhase + float(y) * kPI;

        float line_wow_offset = wow_dev * 0.02f * std::sin(float(y) * 0.08f + wallTime_ * 1.2f);
        float line_flutter_base = flutter_dev * 0.008f *
            std::sin(float(y) * 0.3f + wallTime_ * 47.0f) *
            std::normal_distribution<float>(0.0f, 1.0f)(rng);

        float d_tail = 0.0f;

        bool dropout_active = false;
        float dropout_duration = 0.0f;
        float dropout_x_pos = 0.0f;

        if (ep.dropout_rate > 0.005f) {
            float line_dropout_prob = ep.dropout_rate * 0.4f;
            std::uniform_real_distribution<float> dropout_rng(0, 1);
            std::uniform_real_distribution<float> duration_rng(10, 180);
            std::uniform_real_distribution<float> pos_rng(H_BLANK + 10, H_BLANK + W - 30);
            if (dropout_rng(rng) < line_dropout_prob) {
                dropout_active = true;
                dropout_duration = 10.0f + dropout_rng(rng) * duration_rng(rng);
                dropout_x_pos = pos_rng(rng);
            }
        }

        float field_phase_offset = currentField * kPI;
        float field_chroma_error = field_chroma_phase_accum_;

        float fm_noise_lpf = 0.0f;

        for (int x = 0; x < W_TOTAL; ++x) {
            float current_tbe = abe_start + (float(x) / float(W_TOTAL)) * abe_slope;
            float xSrc = (float)x - current_tbe;
            xSrc = std::fmod(xSrc, float(W_TOTAL));
            if (xSrc < 0.0f) xSrc += float(W_TOTAL);

            float theta = linePhase + xSrc * kSC_PX + line_wow_offset + field_phase_offset;

            float Y = 0.0f, I = 0.0f, Q = 0.0f;

            if (xSrc < 60) {
                Y = -0.3f;
            } else if (xSrc >= 70 && xSrc < 110) {
                Y = 0.0f;
                I = 0.0f; Q = 0.35f;
            } else if (xSrc >= H_BLANK) {
                if (std::uniform_real_distribution<float>(0, 1)(rng) < crease_dropout_prob * 0.05f) {
                    d_tail = 1.0f;
                }

                float vX = std::clamp(xSrc - H_BLANK, 0.f, float(W - 1.001f));
                int x0 = (int)vX;
                float r = sr[x0 * 3 + 2] / 255.f;
                float g = sr[x0 * 3 + 1] / 255.f;
                float b = sr[x0 * 3 + 0] / 255.f;

                Y = 0.299f * r + 0.587f * g + 0.114f * b;
                I = 0.596f * r - 0.274f * g - 0.321f * b;
                Q = 0.211f * r - 0.523f * g + 0.311f * b;

                if (fm_carrier_noise_deg > 0.005f) {
                    float fm_dev = fm_carrier_noise_deg * 0.012f;
                    fm_noise_lpf += fm_dev * (std::uniform_real_distribution<float>(-1, 1)(rng) - fm_noise_lpf);
                    Y += fm_noise_lpf * 0.3f;
                    float fm_chroma_mod = fm_noise_lpf * 0.15f;
                    I += fm_chroma_mod * std::cos(theta * 0.5f);
                    Q += fm_chroma_mod * std::sin(theta * 0.5f);
                }

                if (chroma_crosstalk_deg > 0.005f && !first_line) {
                    float crosstalk_amp = chroma_crosstalk_deg * 0.12f;
                    float crosstalk_offset = kVHS_CHROMA_SHIFT / kSC_FREQ;
                    int adj_x = std::clamp(x + int(crosstalk_offset * 4.0f), 0, W_TOTAL - 1);
                    float adj_sig = prev_comp_line[adj_x];
                    float adj_chroma = adj_sig - prev_line_luma_[std::min(3, x0 % 4)];
                    float ct_phase = theta + crosstalk_offset * kSC_PX;
                    float ct_I = 2.0f * adj_chroma * std::cos(ct_phase) * crosstalk_amp;
                    float ct_Q = -2.0f * adj_chroma * std::sin(ct_phase) * crosstalk_amp;

                    float beat_freq = std::fmod(float(y) / 40.0f, 1.0f);
                    float beat_env = 0.5f + 0.5f * std::cos(beat_freq * 2.0f * kPI);
                    I += ct_I * beat_env;
                    Q += ct_Q * beat_env;
                }

                if (d_tail > 0.01f) {
                    Y = Y * (1.0f - d_tail) + (d_tail > 0.8f ? 0.8f : -0.2f);
                    I *= (1.0f - d_tail); Q *= (1.0f - d_tail);
                    d_tail *= 0.92f;
                }
            }
            if (tracking_rf < 0.95f && xSrc >= H_BLANK) {
                 Y *= tracking_rf; I *= tracking_rf; Q *= tracking_rf;
                 float noise_amp = 1.0f - tracking_rf;
                 Y += std::uniform_real_distribution<float>(-0.8f, 0.8f)(rng) * noise_amp;
                 if (noise_amp > 0.35f) { I *= 0.2f; Q *= 0.2f; }
            }

            if (dropout_active) {
                float dist_to_dropout = std::abs(float(x) - dropout_x_pos);
                if (dist_to_dropout < dropout_duration * 0.5f) {
                    float edge_dist = std::min(
                        std::abs(dist_to_dropout - dropout_duration * 0.5f + 3.0f),
                        std::abs(dist_to_dropout + dropout_duration * 0.5f - 3.0f)
                    );
                    if (edge_dist < 3.0f) {
                        Y = 1.0f + std::uniform_real_distribution<float>(-0.3f, 0.5f)(rng);
                        I = 0.0f; Q = 0.0f;
                    } else {
                        Y = -0.1f + std::uniform_real_distribution<float>(0, 1)(rng) * 0.15f;
                        I = std::uniform_real_distribution<float>(-0.05f, 0.05f)(rng);
                        Q = std::uniform_real_distribution<float>(-0.05f, 0.05f)(rng);
                    }
                }
            }

            if (head_pre_echo_deg > 0.005f && xSrc >= H_BLANK && !first_line) {
                float echo_delay = 3.0f + helical_sweep_deg * 4.0f;
                int echo_x = std::clamp(x - int(echo_delay), 0, W_TOTAL - 1);
                float echo_sig = prev_comp_line[echo_x];
                float echo_amp = head_pre_echo_deg * 0.06f;
                Y += (echo_sig - 0.3f) * echo_amp;
            }

            curr_comp_line[x] = Y + I * std::cos(theta) - Q * std::sin(theta);
        }

        {
            float sumLuma = 0.0f;
            for (int x = H_BLANK; x < W_TOTAL; ++x) sumLuma += curr_comp_line[x];
            prev_line_luma_[0] = prev_line_luma_[1];
            prev_line_luma_[1] = prev_line_luma_[2];
            prev_line_luma_[2] = prev_line_luma_[3];
            prev_line_luma_[3] = sumLuma / float(W_TOTAL - H_BLANK);
        }

        float phase_accum = 0.0f;

        float sumI = 0.f, sumQ = 0.f;
        for (int x = 70; x < 110; ++x) {
            float refTheta = linePhase + float(x) * kSC_PX;
            sumI += curr_comp_line[x] * std::cos(refTheta);
            sumQ -= curr_comp_line[x] * std::sin(refTheta);
        }
        phase_accum = std::atan2(sumQ, sumI) - (kPI / 2.0f);

        phase_accum += field_chroma_error;

        float wow_flutter_phase_error = std::abs(line_wow_offset) + std::abs(line_flutter_base);
        float pll_instability = std::max(0.0f, std::abs(beltSlip) - 2.0f) * 0.05f
                              + ep.flutter_dep * 0.15f
                              + wow_flutter_phase_error * 0.2f;
        if (ep.dropout_rate > 0.01f) {
             if (std::uniform_real_distribution<float>(0, 1)(rng) < ep.dropout_rate * 0.5f) {
                 pll_instability += 2.0f;
             }
        }
        float pll_drift_accum = 0.0f;

        float lpY = 0.f, si = 0.f, sq = 0.f;
        const float lpfA = 0.35f;
        const float chrA = 0.12f;

        std::mt19937 snowRng(uint32_t(frameNum) * 7777u + uint32_t(y) * 31u);

        for (int out_x = 0; out_x < W; ++out_x) {
            int x = out_x + H_BLANK;
            float sig = curr_comp_line[x];

            pll_drift_accum += std::normal_distribution<float>(0, 1)(rng) * pll_instability;

            float x_norm = float(out_x) / float(W);
            float within_line_flutter = line_flutter_base *
                (0.5f + 0.5f * std::sin(x_norm * 12.0f + wallTime_ * 37.0f));

            float decTheta = linePhase + float(x) * kSC_PX + phase_accum + pll_drift_accum
                           + within_line_flutter + field_phase_offset * 0.5f;

            lpY += lpfA * (sig - lpY);

            float chromaSig = sig - lpY;
            float rawI = 2.0f * chromaSig * std::cos(decTheta);
            float rawQ = -2.0f * chromaSig * std::sin(decTheta);

            si += chrA * (rawI - si);
            sq += chrA * (rawQ - sq);

            float r = lpY + 0.956f * si + 0.621f * sq;
            float g = lpY - 0.272f * si - 0.647f * sq;
            float b = lpY - 1.106f * si + 1.703f * sq;

            float snow = 0.0f;
            if (snow_amount > 0.01f) {
                snow = generateSnow(wallTime_, out_x, y, snowRng) * snow_amount;
            }

            float rf_final = tracking_rf * signal_strength;
            if (rf_final < 0.9f && snow_amount > 0.0f) {
                float blend = (0.9f - rf_final) * 10.0f * snow_amount;
                blend = std::clamp(blend, 0.0f, 1.0f);
                float snow_luma = snow * 0.5f + 0.3f;
                float snow_chroma = std::uniform_real_distribution<float>(-0.1f, 0.1f)(snowRng) * snow_amount;
                r = r * (1.0f - blend) + snow_luma * blend + snow_chroma;
                g = g * (1.0f - blend) + snow_luma * blend;
                b = b * (1.0f - blend) + snow_luma * blend - snow_chroma;
            }

            if (recording_blend_ > 0.01f) {
                float rec_snow = generateSnow(wallTime_ + 100.0f, out_x, y, snowRng) * 0.6f;
                float rec_luma = rec_snow * 0.5f + 0.2f;
                r = r * (1.0f - recording_blend_) + rec_luma * recording_blend_;
                g = g * (1.0f - recording_blend_) + rec_luma * recording_blend_;
                b = b * (1.0f - recording_blend_) + rec_luma * recording_blend_;
            }

            dr[out_x * 3 + 0] = (uchar)std::clamp(b * 255.f, 0.f, 255.f);
            dr[out_x * 3 + 1] = (uchar)std::clamp(g * 255.f, 0.f, 255.f);
            dr[out_x * 3 + 2] = (uchar)std::clamp(r * 255.f, 0.f, 255.f);
        }

        std::swap(prev_comp_line, curr_comp_line);
        first_line = false;
    }
}
