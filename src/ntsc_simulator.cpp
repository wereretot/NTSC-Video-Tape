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

static inline float xgauss_(uint32_t& s) {
    float sum = 0.0f;
    for (int i=0; i<4; ++i) sum += xorshift_f(s);
    return sum * 0.866f; // approximate gaussian using sum of uniform logic
}

void NTSCSimulator::initBuffers(int w, int h) {
    W_ = w;
    H_ = h;
    H_BLANK = 144;
    W_TOTAL = w + H_BLANK;
    W_TOTAL_ = W_TOTAL;
    kSC_PX = (2.0f * kPI * kSC_FREQ) / (float)W_;
    for (int i = 0; i < 4; ++i) prev_line_luma_[i] = 0.0f;

    // Initialize VCR brand profile and head config if not yet set
    if (brand_profile_.name == nullptr) {
        brand_profile_ = ::getBrandProfile(brand_);
    }
    if (head_config_.count == HeadCount::TwoHead && head_config_.head_gap_width_um == 0.0f) {
        head_config_ = ::getHeadConfig(HeadCount::TwoHead);
    }

    noise_buf_.resize(W_TOTAL * H_);
    sc_cos_.resize(W_TOTAL);
    sc_sin_.resize(W_TOTAL);
    comp_frame_.resize(W_TOTAL * H_);

    // FM Luma encode/decode buffers
    fm_encode_buf_.resize(W_TOTAL * H_);
    fm_decode_buf_.resize(W_TOTAL * H_);

    // Configure FM processor based on tape format
    float fm_sync = getFMSyncTipFreq(tape_format_) * 1e6f;
    float fm_peak = getFMPeakWhiteFreq(tape_format_) * 1e6f;
    float luma_bw = getLumaBandwidth(tape_format_);
    fm_luma_.configure(fm_sync, fm_peak, luma_bw, tape_format_ == TapeFormat::SVHS);
    
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
    
    if (current_recording_id != last_recording_id_) {
        last_recording_id_ = current_recording_id;
        recording_transition_time_ = 1.0f;
    }
    if (recording_transition_time_ > 0.0f) {
        recording_transition_time_ -= wallDt * 2.0f;
        if (recording_transition_time_ < 0.0f) recording_transition_time_ = 0.0f;
        recording_blend_ = recording_transition_time_;
    }


        const int W = in.cols, H = in.rows;
        if (W == 0 || H == 0) { SDL_Log("[NTSC] CRASH GUARD: empty input frame %d", frameNum); return; }
        if (W != W_ || H != H_) initBuffers(W, H);

        // Fill noise buffer once per frame — eliminates ~960 mt19937 seedings
        // and all per-pixel normal_distribution calls in the hot path.
        {
            uint32_t ns = uint32_t(frameNum) * 2654435761u ^ 40503u;
            for (float& v : noise_buf_) v = xgauss_(ns);
        }

        // Read absolute wall-clock time directly. All oscillators are computed
        // from this value — NOTHING is accumulated per frame, so tape speed
        // and pipeline frame rate cannot affect effect timing.
        wallTime_ = (float)g_wallTimeSec.load();

        // Smooth brand transition: blend parameters over 0.5 seconds
        if (brand_transition_time_ > 0.0f) {
            brand_transition_time_ -= wallDt;
            if (brand_transition_time_ < 0.0f) brand_transition_time_ = 0.0f;
        }

        // Tape time advances at the rate of tape speed. Wow/flutter oscillators
        // are mechanical phenomena on the tape transport, so they must scale
        // with tape speed. When tape slows down (EP mode), wow/flutter also
        // slows proportionally, maintaining correct chroma phase relationships.
        float tapeDt = wallDt * std::max(tapeSpd, 0.001f);

        float tracking_deg = std::clamp(vp.tracking_error, 0.0f, 1.0f);
        float dropout_deg = std::clamp(vp.dropout_rate * 10.0f, 0.0f, 1.0f);
        float motor_deg = std::clamp(vp.motor_health, 0.0f, 1.0f);
        float crease_deg = std::clamp(vp.tape_crease, 0.0f, 1.0f);
        float metal_deg = std::clamp(vp.tape_metal_loss, 0.0f, 1.0f);
        float binder_deg = std::clamp(vp.tape_binder_decay, 0.0f, 1.0f);
        float head_wear_deg = std::clamp(vp.tape_head_wear, 0.0f, 1.0f);

        float helical_sweep_deg = std::clamp(vp.helical_sweep, 0.0f, 1.0f);
        float head_switch_jitter_deg = std::clamp(vp.head_switch_jitter, 0.0f, 1.0f);
        float fm_carrier_noise_deg = std::clamp(vp.fm_carrier_noise, 0.0f, 1.0f);
        float chroma_crosstalk_deg = std::clamp(vp.chroma_crosstalk, 0.0f, 1.0f);
        // 6-head VCRs have Hi-Fi heads with narrower gaps, reducing azimuth crosstalk by 25%
        if (head_config_.has_hifi_heads) {
            chroma_crosstalk_deg *= 0.75f;
        }
        float inter_field_phase_deg = std::clamp(vp.inter_field_phase_error, 0.0f, 1.0f);
        float head_pre_echo_deg [[maybe_unused]] = std::clamp(vp.head_pre_echo, 0.0f, 1.0f);
        float drum_ecc_deg = std::clamp(vp.drum_eccentricity, 0.0f, 1.0f);

        float rf_level = 1.0f - tracking_deg * 0.7f
                               - dropout_deg * 0.3f
                               - metal_deg * 0.55f
                               - crease_deg * 0.5f
                               - binder_deg * 0.25f;
        rf_level = std::clamp(rf_level, 0.05f, 1.0f);

        float chroma_attenuation = 1.0f - tracking_deg * 0.6f
                                         - motor_deg * 0.3f
                                         - head_wear_deg * 0.55f
                                         - metal_deg * 0.20f
                                         - binder_deg * 0.18f;
        chroma_attenuation = std::clamp(chroma_attenuation, 0.03f, 1.0f);

        float luma_noise_level = tracking_deg * 0.5f
                               + dropout_deg * 0.3f
                               + metal_deg * 0.45f
                               + binder_deg * 0.25f
                               + head_wear_deg * 0.15f;
        // Brand modifier: Panasonic has quieter head amp (-52 dB), Sharp is noisier (-46 dB)
        float head_amp_noise_scale = std::pow(10.0f, (-brand_profile_.head_amp_noise_floor_db - 48.0f) / 20.0f);
        luma_noise_level *= head_amp_noise_scale;

        float chroma_noise_level = tracking_deg * 0.6f
                                 + motor_deg * 0.4f
                                 + head_wear_deg * 0.50f
                                 + metal_deg * 0.25f
                                 + binder_deg * 0.15f;
        float wear_noise = std::clamp(
            (1.0f - rf_level) * 0.9f + luma_noise_level * 0.25f + chroma_noise_level * 0.2f,
            0.0f,
            1.2f);

        float dropout_drive = std::clamp(
            vp.dropout_rate + vp.tape_metal_loss * 0.18f + vp.tape_binder_decay * 0.12f + vp.tape_crease * 0.03f,
            0.0f,
            0.25f);

        // S-VHS tape format modifiers: metal particle tape improves signal quality
        if (tape_format_ == TapeFormat::SVHS) {
            // Metal tape reduces dropouts by 50%
            dropout_drive *= 0.5f;
            // +3 dB RF signal improvement (~1.41x voltage gain)
            rf_level = std::min(rf_level * 1.41f, 1.0f);
            // -40% tape hiss (noise reduction)
            luma_noise_level *= 0.6f;
            chroma_noise_level *= 0.6f;
            wear_noise *= 0.6f;
        }

        // FM carrier and bandwidth parameters based on tape format
        // VHS: 3.4-4.4 MHz FM, 3.0 MHz luma BW, 0.5 MHz chroma BW
        // S-VHS: 5.4-7.0 MHz FM, 5.0 MHz luma BW, 1.0 MHz chroma BW
        float fm_sync_tip_mhz [[maybe_unused]] = getFMSyncTipFreq(tape_format_);
        float fm_peak_white_mhz [[maybe_unused]] = getFMPeakWhiteFreq(tape_format_);
        float luma_bw_mhz = getLumaBandwidth(tape_format_);
        float chroma_bw_mhz [[maybe_unused]] = getChromaBandwidth(tape_format_);

        // Bandwidth affects smear/persistence: higher BW (S-VHS) = less smear
        // Normalize to VHS baseline (3.0 MHz luma)
        float bw_sharpness_factor = luma_bw_mhz / 3.0f;  // 1.0 for VHS, 1.67 for S-VHS

        // --- Tracking Pulse & Flyback Lock Model ---
        // Simulates control/sync pulse strength that a CRT flyback/H+V oscillator
        // can lock onto. Worn tape weakens pulse amplitude and causes unstable lock.
        float tracking_pulse_strength = std::clamp(
            rf_level
            - metal_deg * 0.45f
            - head_wear_deg * 0.35f
            - binder_deg * 0.22f
            - motor_deg * 0.10f
            - ep.motor_drag * 0.16f
            - dropout_drive * 1.8f
            - vp.sync_hold_failure * 0.30f,
            0.0f,
            1.0f);

        float lock_target = std::clamp((tracking_pulse_strength - 0.18f) / 0.72f, 0.0f, 1.0f);
        // Brand-specific CTL servo: Panasonic locks faster, Sharp locks slower
        float lock_attack_rate = brand_profile_.ctl_servo_bandwidth_hz * 1.0f;
        float lock_release_rate = brand_profile_.ctl_servo_bandwidth_hz * 0.31f;
        float lock_attack = 1.0f - std::exp(-wallDt * lock_attack_rate);
        float lock_release = 1.0f - std::exp(-wallDt * lock_release_rate);
        float lock_alpha = (lock_target > tracking_lock_) ? lock_attack : lock_release;
        // CTL head sensitivity modulates the effective lock target
        lock_target *= brand_profile_.ctl_head_sensitivity / 0.85f; // normalize to JVC
        tracking_lock_ += (lock_target - tracking_lock_) * lock_alpha;
        tracking_lock_ = std::clamp(tracking_lock_, 0.0f, 1.0f);
        float unlock = 1.0f - tracking_lock_;

        // Field phase: NTSC fields at 59.94 Hz
        const float fPhase = std::fmod(wallTime_ * (kNTSC_FPS * 2.f), 4.f) * (kPI * 0.5f);

        // ── WOW/FLUTTER OSCILLATORS — time-varying tape speed modulation ──
        // Real VHS wow/flutter is a continuous time-varying speed error.
        // This frequency-modulates the NTSC 3.58 MHz subcarrier, creating
        // the characteristic color hue drift and chroma phase errors.
        //
        // Wow: slow speed variation (0.3-2 Hz) from capstan roller
        //      eccentricity and reel irregularities
        // Flutter: fast speed variation (15-67 Hz) from mechanical
        //      resonances, guide rollers, tape tension modes
        //
        static constexpr float WOW_HZ[]    = {0.3f, 0.8f, 2.1f};
        static constexpr float FLUTTER_HZ[] = {15.0f, 23.0f, 30.0f, 46.0f, 67.0f};
        static constexpr float WOW_AMP[]   = {0.5f, 0.3f, 0.2f};     // relative weights
        static constexpr float FLUTTER_AMP[] = {0.25f, 0.20f, 0.20f, 0.18f, 0.17f};

        float wow_sum = 0.0f;
        for (int i = 0; i < 3; ++i) {
            wow_phase_[i] = std::fmod(wow_phase_[i] + tapeDt * WOW_HZ[i], 1.0f);
            wow_sum += WOW_AMP[i] * std::sin(wow_phase_[i] * 2.0f * kPI);
        }

        float flutter_sum = 0.0f;
        for (int i = 0; i < 5; ++i) {
            flutter_phase_[i] = std::fmod(flutter_phase_[i] + tapeDt * FLUTTER_HZ[i], 1.0f);
            flutter_sum += FLUTTER_AMP[i] * std::sin(flutter_phase_[i] * 2.0f * kPI);
        }

        // Scale by user settings: wow_dep (0-5) and flutter_dep (0-1)
        // Realistic VHS wow is ±0.5-3% speed variation, flutter is ±0.1-0.5%
        // Brand modifiers: tape path length affects wow, scrape flutter coeff affects flutter
        float tape_path_factor = brand_profile_.tape_path_length_mm / 420.0f; // normalize to JVC
        float wow_dev = wow_sum * ep.wow_dep * 0.01f * tape_path_factor;  // up to ±5% at max
        float flutter_dev = flutter_sum * ep.flutter_dep * 0.008f * brand_profile_.scrape_flutter_coeff;

        // Wow/flutter and transport drag also disturb sync pulse lock.
        float sync_transport_dev = std::abs(instantSpd - std::max(tapeSpd, 0.001f)) / std::max(tapeSpd, 0.001f);
        float wow_flutter_stress = std::clamp(std::abs(wow_dev) * 10.0f + std::abs(flutter_dev) * 20.0f, 0.0f, 1.0f);
        float drag_stress = std::clamp(ep.motor_drag * 1.2f + sync_transport_dev * 1.5f + motor_deg * 0.5f, 0.0f, 1.0f);
        float lock_disturb = std::clamp(wow_flutter_stress * 0.25f + drag_stress * 0.30f, 0.0f, 1.0f);
        tracking_lock_ -= lock_disturb * (1.0f - std::exp(-wallDt * 4.0f));
        tracking_lock_ = std::clamp(tracking_lock_, 0.0f, 1.0f);
        unlock = 1.0f - tracking_lock_;

        // Note: Real VHS does NOT accumulate subcarrier phase error frame-to-frame.
        // The VCR re-generates the color burst at a clean reference every field,
        // and the TV's color PLL locks to that burst. Residual chroma errors come
        // from tiny per-line phase offsets (mechanical tape bounce) and very
        // subtle within-line flutter noise. These are subtle — VHS color quality
        // is dominated by overall hue offset (the TBC HUE knob), not rainbowing.
        // The per-line offsets are applied in the scanline processing loop below.

        // --- V-SYNC LOSS (Rolling) & TAPE SPEED ERROR ---
        float speed_error = std::abs(1.0f - instantSpd);
        float target_error = speed_error + vp.tracking_error + unlock * (0.18f + dropout_drive * 1.2f);
        if (target_error > 2.0f) target_error = 2.0f;

        // Low-pass filter on tracking error (per-frame → use absolute wall-clock)
        float dt = wallDt;
        float lpfA = 1.f - std::exp(-dt * 3.f);  // 3 Hz cutoff → real seconds
        tracking_error_lpf_ += (target_error - tracking_error_lpf_) * lpfA;

        // V-roll: absolute position from wall-clock
        if (target_error > 0.01f || vp.sync_hold_failure > 0.0f) {
            float roll_hz = (target_error * 7.5f) + (vp.sync_hold_failure * 10.f);
            v_roll_accum_ = std::sin(wallTime_ * roll_hz * kPI * 2.f) * (target_error * 480.f + vp.sync_hold_failure * 600.f);
        } else {
            v_roll_accum_ *= std::exp(-dt * 10.f);
            if (std::abs(v_roll_accum_) < 1.0f) v_roll_accum_ = 0.0f;
        }

        float v_jitter = 0.0f;
        if (target_error > 0.1f || unlock > 0.05f) {
            std::mt19937 jRng(uint32_t(frameNum) * 999u);
            v_jitter = std::normal_distribution<float>(0, std::max<float>(1e-6f, target_error * 10.0f + unlock * 14.0f))(jRng);
        }

        int v_roll = (int(v_roll_accum_ + v_jitter)) % H;
        if (v_roll < 0) v_roll += H;

        // --- TRACKING ENVELOPE (Noise Bar Drift) — absolute phase ---
        // Real VHS tracking errors create a moving noise bar that scrolls
        // vertically. The bar moves at 0.5-8 Hz depending on tracking error.
        float noise_bar_rate = target_error * 0.8f * kNTSC_FPS;  // Faster than before
        noise_bar_phase_ = std::fmod(wallTime_ * noise_bar_rate, 1.0f);
        if (noise_bar_phase_ < 0.0f) noise_bar_phase_ += 1.0f;
        float bar_y = noise_bar_phase_ * float(H);
        float bar_width = 20.0f + tracking_error_lpf_ * 80.0f + unlock * 35.0f;

        // --- H-SYNC Roll — absolute phase ---
        // Real VHS H-sync errors are FAST: the horizontal oscillator
        // in a CRT TV runs at 15,734 Hz and any deviation from the
        // broadcast signal causes immediate phase drift. When tape
        // speed is wrong, H-sync rolls at 2-15 Hz depending on error.
        float h_freq_err = (target_error * 12.0f) + (vp.sync_hold_failure * 60.0f) + unlock * (12.0f + dropout_drive * 42.0f);
        if (h_freq_err > 0.01f) {
            h_roll_phase_ = std::fmod(h_roll_phase_ - wallDt * h_freq_err, 1.0f);
            if (h_roll_phase_ < 0.0f) h_roll_phase_ += 1.0f;
        } else {
            // Decay H-roll phase to zero when signal is clean
            h_roll_phase_ *= std::exp(-wallDt * 8.0f);
            if (std::abs(h_roll_phase_) < 0.001f) h_roll_phase_ = 0.0f;
        }

        // --- TAPE-DRIVEN MECHANICAL MODEL ---
        // Real VHS horizontal warping is characterized by FAST, chaotic
        // micro-transients (tape tension spikes, guide roller bounce,
        // head drum eccentricity) superimposed on slower "wow" drift.
        // Guard against div-by-zero during engine spinup
        float safeTapeSpd = std::max(tapeSpd, 0.001f);
        float speed_dev = std::abs(instantSpd - safeTapeSpd) / safeTapeSpd;

        // --- VCR ALIGNMENT STATE -------------------------------------------
        // Alignment errors affect the video in different ways:
        // - head_azimuth_error: top-of-screen squewing, chroma phase errors
        // - tracking_alignment: head switch boundary shift, noise bar position
        // - drum_height_error: top/bottom asymmetry (tilt)
        // - audio_head_alignment: audio head position error (affects Hi-Fi)
        {
            float align_drift_rate = 1.0f - std::exp(-wallDt * 0.5f);

            // Head azimuth: tracks the set misalignment value
            head_azimuth_shift_ += (vp.head_azimuth_error - head_azimuth_shift_) * align_drift_rate;

            // Tracking alignment: affects head switch position
            tracking_phase_error_ += (vp.tracking_alignment - tracking_phase_error_) * align_drift_rate;

            // Drum height/tilt: causes top/bottom asymmetry
            drum_tilt_error_ += (vp.drum_height_error - drum_tilt_error_) * align_drift_rate;

            // Audio head alignment
            audio_head_shift_ += (vp.audio_head_alignment - audio_head_shift_) * align_drift_rate;
        }

        // --- MOTOR DEGRADATION ARTIFACTS ----------------------------------
        // motor_health = 0.0 means no motor defects. motor_health = 1.0 means
        // worst motor degradation (cogging, belt slip, drum wobble, vibration).
        // A bad motor in a VCR causes unique artifacts that are NOT wow/flutter:
        //   1. Cogging: discrete stepping at rotor notch positions
        //   2. Belt slip: rubber degrades, slips at the same point per rev
        //   3. Drum wobble: bearing wear transmits vibration to tape path
        //   4. Speed surges: motor can't maintain constant RPM
        // These get WORSE as motor_health approaches 1.0 (fully degraded).
        float motor_defect = std::clamp(vp.motor_health, 0.0f, 1.0f);

        // Motor cogging: discrete "notches" as the rotor passes each pole.
        // Creates a staircase-like horizontal displacement that jumps every
        // few milliseconds. Brand-specific: Toshiba has 6 poles, others have 8.
        float motor_cog = 0.0f;
        if (motor_defect > 0.05f) {
            float cog_hz = 3.7f;  // ~222 RPM capstan motor
            float cog_phase = std::fmod(tapeTime_ * cog_hz, 1.0f);
            float num_poles = float(brand_profile_.motor_pole_count);
            float pole_frac = std::fmod(cog_phase * num_poles, 1.0f);
            // Sharp catch, slow release
            motor_cog = std::pow(pole_frac, 0.3f) * motor_defect * 8.0f;
        }

        // Drum wobble: bearing wear causes the rotating drum to vibrate.
        // This creates per-line horizontal jitter that varies with drum
        // position. The drum spins at 1800 RPM (30 rev/sec for NTSC).
        float drum_wobble_phase = 0.0f;
        if (motor_defect > 0.05f) {
            drum_wobble_phase = std::fmod(tapeTime_ * 30.0f, 1.0f) * 2.0f * kPI;
        }

        // Motor vibration: broadband mechanical noise transmitted to tape path
        // Creates fine horizontal "hash" or "fizz" on every scanline
        float motor_vibration = 0.0f;
        if (motor_defect > 0.05f) {
            float motor_rpm_hz = 1800.0f / 60.0f;  // 30 Hz for capstan
            motor_vibration = motor_defect * 2.0f * (
                std::sin(tapeTime_ * motor_rpm_hz * 2.0f * kPI) * 0.6f +
                std::sin(tapeTime_ * motor_rpm_hz * 4.0f * kPI) * 0.3f);
        }

        // Fast timebase error: high-frequency jitter from tape scraping
        // past heads. Realistic VHS TBE is 1-15 pixels peak-to-peak
        // under tracking error, with impulsive spikes up to 40 px.
        float fast_tbe = speed_dev * 18.0f;

        // Slow "wow" drift: large-scale tape tension variations (0.1–2 Hz)
        // Amplitude: 5-30 pixels depending on tape condition
        mechanical_wow_ = speed_dev * 120.0f;

        // Tape tension impulse spikes: sudden "snap" events when tape
        // slips or catches on guide pins. Modeled as stochastic impulses.
        float tape_tension_impulse = 0.0f;
        {
            // Impulse rate scales with speed error (faster = more spikes)
            float impulse_rate = speed_dev * 4.0f + 0.5f;  // 0.5-4 Hz
            float impulse_phase = std::fmod(tapeTime_ * impulse_rate, 1.0f);
            // Sharp attack, slow decay (real tape tension behavior)
            if (impulse_phase < 0.05f) {
                tape_tension_impulse = (impulse_phase / 0.05f) * speed_dev * 35.0f;
            } else {
                tape_tension_impulse = std::exp(-(impulse_phase - 0.05f) * 8.0f) * speed_dev * 35.0f;
            }
        }

        // AFC error: the TV's PLL trying to catch the line sync.
        // Strong at the top of screen, decays through the field.
        // Real VCR AFC has pull-in range of ~±500 Hz, time constant ~3-8 lines.
        //
        // The wow/flutter create TBE through the bend_fraction mechanism in the
        // scanline loop (which creates proper exponential bending). The AFC
        // error here provides the baseline tracking instability from speed error
        // and tape tension events. MOTOR DEGRADATION adds cogging and vibration.
        afc_error_ = mechanical_wow_ * 0.4f
                   + (speed_dev * 400.0f)
                   + fast_tbe
                   + tape_tension_impulse
                   + motor_cog * 50.0f        // motor cogging: up to +400px TBE at worst
                   + motor_vibration * 15.0f;  // motor vibration: up to +30px hash

        // --- Tape Crease phase ---
        float crease_y = std::fmod(tapeTime_ * 0.1f, 1.0f) * float(H);
        crinkle_phase_ = std::fmod(crinkle_phase_ + wallDt * (0.12f + crease_deg * 0.55f), 1.0f);
        float crinkle_phase_rad = crinkle_phase_ * 2.0f * kPI;
        float crinkle_center_target = crease_y
            + std::sin(crinkle_phase_rad * 0.71f + 1.2f) * (float(H) * (0.08f + crease_deg * 0.18f))
            + std::cos(crinkle_phase_rad * 1.33f - 0.6f) * (float(H) * 0.05f);
        while (crinkle_center_target < 0.0f) crinkle_center_target += float(H);
        while (crinkle_center_target >= float(H)) crinkle_center_target -= float(H);
        float crinkle_center_alpha = 1.0f - std::exp(-wallDt * 2.4f);
        if (crinkle_center_lpf_ <= 0.0f) crinkle_center_lpf_ = crease_y;
        crinkle_center_lpf_ += (crinkle_center_target - crinkle_center_lpf_) * crinkle_center_alpha;
        while (crinkle_center_lpf_ < 0.0f) crinkle_center_lpf_ += float(H);
        while (crinkle_center_lpf_ >= float(H)) crinkle_center_lpf_ -= float(H);

        float crinkle_pulse = 0.5f + 0.5f * std::sin(crinkle_phase_rad * 1.9f + 0.8f);
        float crinkle_strength_target = std::pow(crease_deg, 1.15f) * (0.35f + 0.65f * crinkle_pulse);
        float crinkle_strength_alpha = 1.0f - std::exp(-wallDt * (1.8f + crease_deg * 3.0f));
        crinkle_strength_lpf_ += (crinkle_strength_target - crinkle_strength_lpf_) * crinkle_strength_alpha;
        crinkle_strength_lpf_ = std::clamp(crinkle_strength_lpf_, 0.0f, 1.0f);
        float crinkle_center_y = crinkle_center_lpf_;
        float crinkle_band = 10.0f + 55.0f * crease_deg;
        float crinkle_strength = crinkle_strength_lpf_;

        int currentField = (frameNum % 2);
        if (currentField != prev_field_) {
            std::mt19937 fieldRng(uint32_t(frameNum) * 13u);
            float phase_drift = std::normal_distribution<float>(0, std::max<float>(1e-6f, inter_field_phase_deg * 0.06f))(fieldRng);
            field_chroma_phase_accum_ = field_chroma_phase_accum_ * 0.92f + phase_drift;
            if (head_switch_jitter_deg > 0.01f) {
                // Brand modifier: Sony has more jitter, Panasonic less
                float brand_jitter_scale = brand_profile_.head_switch_jitter_lines / 0.3f; // normalize to JVC
                head_switch_offset_ = std::normal_distribution<float>(0, std::max<float>(1e-6f,
                    head_switch_jitter_deg * 3.0f * brand_jitter_scale))(fieldRng);
            } else {
                head_switch_offset_ = 0.0f;
            }
            prev_field_ = currentField;
        }
        drum_ecc_phase_ = std::fmod(drum_ecc_phase_ + wallDt * 30.0f, 1.0f);
        // Brand-specific drum resonance: Sony resonates at 30.5 Hz, others at 30.0 Hz
        float drum_ecc_wobble = drum_ecc_deg * std::sin(drum_ecc_phase_ * 2.0f * kPI) * 2.0f;
        // Add brand-specific bearing resonance (narrow-band oscillation)
        float bearing_res = brand_profile_.drum_bearing_resonance_hz;
        float bearing_q = brand_profile_.drum_bearing_q;
        float bearing_resonance = std::sin(wallTime_ * bearing_res * 2.0f * kPI)
            * drum_ecc_deg * 0.15f * (bearing_q / 3.5f); // normalize Q to JVC
        drum_ecc_wobble += bearing_resonance;
        adjacent_track_phase_ = std::fmod(adjacent_track_phase_ + wallDt * 0.1f, 1.0f);

        float head_sweep_px_per_line = helical_sweep_deg * 1.5f;
        float head_sweep_grad = head_sweep_px_per_line / float(W);
        float hsw_boundary_y = float(H) - 7.0f * float(H) / 240.0f + head_switch_offset_;

        


    out.create(H, W, CV_8UC3);

#ifdef ADO_OPENMP
#pragma omp parallel
        {
            std::vector<float> comp_line;
            comp_line.resize(W_TOTAL);
#pragma omp for schedule(dynamic, 16)
            for (int y = 0; y < H; ++y) {
                // rng only used for rare/conditional distributions not in inner-x loop
                std::mt19937 rng(uint32_t(frameNum) * 65537u + uint32_t(y) * 1013u);
                const float* nb = noise_buf_.data() + y * W_TOTAL_;
                int src_y = (y + v_roll) % H;
                const uchar* sr = in.ptr(src_y);
                uchar* dr = out.ptr(y);

                // 1. PHYSICAL TAPE GEOMETRY (The "Source" signal)
                // Real VHS timebase error is a composite of:
                //   - AFC pull-in at top of screen (flagging)
                //   - Fast scrape flutter (tape/head friction, ~50-200 Hz)
                //   - Guide-roller micro-bounces (impulse-like spikes)
                //   - Motor belt slips (underdamped ringing)
                //   - Head-switch transients (bottom VBI tear)

                // Per-line FAST scrape flutter (high-frequency noise)
                // Real VHS head-to-tape friction creates fine per-line jitter.
                // This is SUBTLE — typically 0.1-0.5px RMS, not whole-pixel shifts.
                float scrape_flutter = nb[0] * ep.flutter_dep * 0.4f;

                // AFC Pull-in: Error decays exponentially as TV catches sync.
                // Real VHS PLL pull-in time constant is ~3-8 scanlines.
                // Creates the classic "flagging" bend at the top of screen.
                // The bending extends ~20-30 lines down from the top.
                float line_afc = afc_error_ * std::exp(-float(y) * 0.08f);  // ~30 line decay

                // ── Head Alignment: Top-of-screen squewing ──────────────────
                // When video heads are misaligned (azimuth angle or drum height),
                // the head-to-tape contact is uneven, causing the top of the
                // screen to skew/squew. This is the classic "head adjustment"
                // symptom — turning the head alignment screw fixes it.
                // Real head misalignment is STABLE — it doesn't wobble or flicker.
                float head_azimuth_deg = head_azimuth_shift_ * 5.967f;  // Max ±5.967°
                float head_squewing = 0.0f;
                if (std::abs(head_azimuth_deg) > 0.5f) {
                    // Squewing: horizontal offset that decays from top
                    float squew_amt = head_azimuth_deg * 8.0f;
                    head_squewing = squew_amt * std::exp(-float(y) * 0.03f);  // ~70 line decay
                }

                // Drum height misalignment: causes top/bottom asymmetry
                float drum_height_ske = 0.0f;
                if (drum_tilt_error_ > 0.01f) {
                    // Tilt causes the top and bottom to skew in opposite directions
                    float y_norm = float(y) / float(H);
                    drum_height_ske = drum_tilt_error_ * 20.0f * (y_norm - 0.5f);
                }

                // Tracking alignment error: shifts the head switch boundary
                float tracking_shift = tracking_phase_error_ * 6.0f;  // Up to ±6 lines
                float head_switch_shift = tracking_shift;  // Stable offset

                line_afc += head_squewing + drum_height_ske;
                float flyback_unlock_jitter = unlock * (6.0f + binder_deg * 16.0f + head_wear_deg * 14.0f)
                    * std::sin(float(y) * 0.42f + wallTime_ * 91.0f);
                float unlock_top_gate = 1.0f - std::exp(-float(y) * 0.06f);
                line_afc += flyback_unlock_jitter * unlock_top_gate;

                // ── Motor Degradation: Realistic Effects ────────────────────
                // Motor degradation causes multiple visible artifacts:
                // 1. Capstan wow/flutter: slow speed variations affecting whole frame
                // 2. Belt slip: horizontal "bites" through the image at belt rev point
                // 3. Drum wobble: fine per-line jitter from worn drum bearings
                // 4. Speed instability: frame rate variation, vertical rolling

                // 1. Capstan wow: slow (0.5-4 Hz) speed variation from motor/belt
                float capstan_wow = 0.0f;
                if (motor_defect > 0.05f) {
                    capstan_wow_phase_ += (0.5f + motor_defect * 3.0f) * wallDt * 2.0f * kPI;
                    if (capstan_wow_phase_ > 2.0f * kPI) capstan_wow_phase_ -= 2.0f * kPI;
                    capstan_wow = std::sin(capstan_wow_phase_) * motor_defect * 3.0f;
                    // Secondary wow component at different frequency
                    capstan_wow += std::sin(capstan_wow_phase_ * 1.7f + 1.3f) * motor_defect * 1.5f;
                }

                // 2. Belt slip: horizontal "bite" at belt revolution point
                // Belt revolves ~every 2 seconds, slip creates a moving distortion band
                float belt_slip = 0.0f;
                if (motor_defect > 0.05f) {
                    motor_belt_slip_phase_ += (0.4f + motor_defect * 1.5f) * wallDt * 2.0f * kPI;
                    if (motor_belt_slip_phase_ > 2.0f * kPI) motor_belt_slip_phase_ -= 2.0f * kPI;

                    // Belt slip position moves through the frame
                    float slip_y = std::fmod(motor_belt_slip_phase_ / (2.0f * kPI), 1.0f) * float(H);
                    float dist_to_slip = std::abs(float(y) - slip_y);
                    if (dist_to_slip > float(H) / 2.0f) dist_to_slip = float(H) - dist_to_slip;

                    if (dist_to_slip < 40.0f) {
                        float slip_intensity = std::pow(1.0f - dist_to_slip / 40.0f, 2.0f);
                        // Belt slip creates a horizontal "bite" — sudden shift then recovery
                        belt_slip = slip_intensity * std::sin(dist_to_slip * 0.3f + tapeTime_ * 8.0f)
                                  * motor_defect * 15.0f;
                        // Add RF level drop during slip (head loses contact momentarily)
                        rf_level *= (1.0f - slip_intensity * motor_defect * 0.3f);
                    }
                }

                // 3. Drum wobble: fine per-line jitter from worn drum bearings
                // This is high-frequency (30 Hz = drum rotation rate), small amplitude
                float drum_wobble = 0.0f;
                if (motor_defect > 0.05f) {
                    drum_wobble_phase += 30.0f * wallDt * 2.0f * kPI;
                    if (drum_wobble_phase > 2.0f * kPI) drum_wobble_phase -= 2.0f * kPI;
                    // Wobble amplitude increases with motor wear
                    float wobble_amp = motor_defect * motor_defect * 4.0f;  // Quadratic for subtle low wear
                    drum_wobble = std::sin(drum_wobble_phase + float(y) * 0.1f) * wobble_amp;
                    // Add secondary harmonic from eccentric bearing
                    drum_wobble += std::sin(drum_wobble_phase * 2.0f + 0.7f) * wobble_amp * 0.3f;
                }

                // 4. Speed instability: causes horizontal stretch/squeeze variation
                // This modulates the tape speed per-frame, affecting image width
                motor_speed_var_ = 0.0f;
                if (motor_defect > 0.05f) {
                    motor_speed_var_ = capstan_wow * 0.02f  // Wow contributes to speed
                                     + belt_slip * 0.005f;   // Belt slip causes momentary speed change
                }

                // Combine all motor effects (will be added to total_h_warp later)
                float motor_tbe = capstan_wow + belt_slip + drum_wobble;

                // Guide-roller micro-bounce: impulsive spike every ~80-200 lines
                // caused by tape physically bouncing off guide rollers.
                // Multiple bounce points for more chaotic behavior.
                float guide_bounce = 0.0f;
                for (int bounce_idx = 0; bounce_idx < 3; ++bounce_idx) {
                    float bounce_freq = 2.3f + bounce_idx * 1.7f;  // Different rates
                    float bounce_y = std::fmod(tapeTime_ * bounce_freq + float(frameNum) * 0.07f * (bounce_idx + 1), 1.0f) * float(H);
                    float dist_bounce = std::abs(float(y) - bounce_y);
                    if (dist_bounce > (float)H / 2.0f) dist_bounce = (float)H - dist_bounce;
                    if (dist_bounce < 15.0f && (speed_dev > 0.005f || ep.flutter_dep > 0.05f)) {
                        float intensity = std::pow(1.0f - dist_bounce / 15.0f, 2.5f);
                        // Sharp impulse: asymmetric attack (< 1 line), exponential decay
                        float attack = intensity * (speed_dev * 30.0f + ep.flutter_dep * 8.0f);
                        guide_bounce += attack * std::exp(-dist_bounce * 0.4f);
                    }
                }

                // Add the immediate physical jitter to the AFC baseline
                float total_h_warp = line_afc + mechanical_wow_ + scrape_flutter + guide_bounce + motor_tbe;

                // --- B. Motor Degradation: Combined TBE Effects ---
                // Motor effects (capstan_wow, belt_slip, drum_wobble) are computed
                // at frame level and applied per-scanline here.

                // Motor drag: legacy belt slip effect (superseded by new model)
                float drag_hz = 0.5f + ep.motor_drag * 2.0f;
                float drag_phase = std::fmod(tapeTime_ * drag_hz, 1.0f) * H;
                float dy = float(y) - drag_phase;
                float beltSlip_legacy = 0.0f;
                if (dy > 0.0f && dy < 100.0f && ep.motor_drag > 0.01f) {
                    beltSlip_legacy = std::exp(-dy * 0.05f) * std::sin(dy * 0.2f) * (ep.motor_drag * 20.0f);
                }

                total_h_warp += beltSlip_legacy;

                float drum_ecc_offset = drum_ecc_wobble * std::sin(float(y) * 0.03f + drum_ecc_phase_ * 2.0f * kPI);
                total_h_warp += drum_ecc_offset;

                // --- D. Head Switch Noise (VHS Tearing at bottom of VBI) ---
                // The head switch boundary position is affected by tracking alignment.
                // Misaligned tracking shifts the boundary up or down.
                float hsTearing = 0.0f;
                float hs_line_offset = motor_defect * 12.0f *
                    std::sin(tapeTime_ * 1.5f * 2.0f * kPI);
                // Tracking alignment shifts the head switch boundary
                hs_line_offset += tracking_phase_error_ * 15.0f;
                int hs_start = std::max(0, H - 24 + int(hs_line_offset) + int(head_switch_shift));

                float dist_to_hsw = float(y) - hsw_boundary_y;
                if (dist_to_hsw > -3.0f && dist_to_hsw < 8.0f) {
                    float hsw_intensity = 0.0f;
                    if (dist_to_hsw < 0.0f) {
                        hsw_intensity = std::pow(1.0f + dist_to_hsw / 3.0f, 2.0f);
                    } else {
                        hsw_intensity = std::exp(-dist_to_hsw * 0.5f);
                    }
                    hsw_intensity *= head_switch_jitter_deg;
                    // Brand modifier: Sharp has larger head switch transients
                    float hsw_transient_scale = brand_profile_.head_switch_transient_amp / 0.12f; // normalize to JVC
                    // 4-head and 6-head VCRs have dedicated slow-mo heads, reducing switch transients
                    if (head_config_.has_slowmo_heads) {
                        hsw_transient_scale *= 0.6f; // 40% reduction
                    }
                    float hsw_shear = hsw_intensity * 25.0f * hsw_transient_scale
                        * std::sin(float(y) * 1.7f + tapeTime_ * 120.0f);
                    float hsw_noise = hsw_intensity * std::normal_distribution<float>(0, 0.4f)(rng);
                    hsTearing += hsw_shear + hsw_noise * 10.0f;
                    total_h_warp += hsw_shear * 0.3f;
                }

                if (y >= hs_start) {
                    float hsDepth = (float(y) - float(hs_start)) / 24.0f;
                    float hs_gain = 35.0f + motor_defect * 25.0f;
                    hsTearing += std::pow(hsDepth, 3.5f) * hs_gain;
                    if (y >= H-3) hsTearing += std::normal_distribution<float>(0, 8 + motor_defect * 20)(rng);
                }

                float motor_rf_noise = 0.0f;
                if (motor_defect > 0.05f) {
                    motor_rf_noise = motor_vibration * 0.05f;
                }

                // --- Massive H-SYNC Roll & Tracking Shear ---
                // Tracking RF Mask
                float dist_rf = std::abs(float(src_y) - bar_y);
                if (dist_rf > (float)H / 2.0f) dist_rf = (float)H - dist_rf; 
                float tracking_rf = rf_level;
                if (dist_rf < bar_width) {
                    float min_rf = dist_rf / bar_width;
                    float fade = std::clamp(tracking_error_lpf_ * 10.0f, 0.0f, 1.0f);
                    tracking_rf *= 1.0f - fade * (1.0f - min_rf);
                }
                tracking_rf = std::clamp(tracking_rf, 0.02f, 1.0f);

                // Add tape crease disturbance (Physical V-shaped crinkle)
                float dist_crease = std::abs(float(src_y) - crease_y);
                if (dist_crease > (float)H / 2.0f) dist_crease = (float)H - dist_crease;
                
                float crease_shear = 0.0f;
                float crease_dropout_prob = 0.0f;
                float crease_slope = 0.0f;
                
                if (vp.tape_crease > 0.01f && dist_crease < 35.0f) {
                    float intensity = std::pow(1.0f - (dist_crease / 35.0f), 1.5f);

                    // A. Geometric Shear: Sharp V-shape displacement
                    // Mimics the tape physically buckling away from the head
                    // High-frequency chaotic oscillation simulates crease
                    // "catching" on the head edge during playback
                    float shear_wave = std::sin(float(y) * 0.3f + wallTime_ * 28.0f) * 15.0f
                                     + std::cos(float(y) * 0.5f - wallTime_ * 43.0f) * 8.0f;
                    crease_shear = intensity * vp.tape_crease * (50.0f + shear_wave);

                    // B. Signal Degradation
                    tracking_rf *= std::max(0.05f, 1.0f - intensity * vp.tape_crease * 3.0f);
                    crease_dropout_prob = intensity * vp.tape_crease * 0.6f;
                }

                if (crinkle_strength > 0.005f) {
                    float dist_crinkle = std::abs(float(src_y) - crinkle_center_y);
                    if (dist_crinkle > (float)H * 0.5f) dist_crinkle = float(H) - dist_crinkle;
                    if (dist_crinkle < crinkle_band) {
                        float ci = std::pow(1.0f - (dist_crinkle / crinkle_band), 2.2f) * crinkle_strength;
                        float fold_wave = std::sin(float(y) * 0.18f + wallTime_ * 17.0f + crinkle_phase_rad);
                        float notch = fold_wave > 0.0f ? 1.0f : -0.6f;
                        crease_shear += ci * (36.0f + fold_wave * 24.0f + notch * 18.0f);
                        crease_slope += ci * (0.4f + 0.45f * std::sin(float(y) * 0.07f + wallTime_ * 11.0f));
                        tracking_rf *= std::max(0.03f, 1.0f - ci * 1.6f);
                        crease_dropout_prob = std::max(crease_dropout_prob, ci * 0.8f);
                    }
                }

                float current_h_roll = h_roll_phase_ * 0.1f;  // Reduced influence
                float h_sync_roll = current_h_roll * float(W_TOTAL) * (target_error + vp.sync_hold_failure + unlock * 1.2f);

                float h_sync_shear = 0.0f;
                if (tracking_error_lpf_ > 0.01f || vp.tape_crease > 0.01f) {
                     // Fast chaotic shear: multiple high-frequency oscillators
                     // Real VHS tracking errors cause rapid line-to-line shifts
                     // with frequencies in the 10-80 Hz range.
                     float chaotic_shear = std::sin(src_y * 0.22f + wallTime_ * 35.0f) * 0.7f
                                         + std::cos(src_y * 0.35f - wallTime_ * 52.0f) * 0.5f
                                         + std::sin(src_y * 0.11f + wallTime_ * 19.0f) * 0.4f
                                         + std::sin(src_y * 0.47f - wallTime_ * 71.0f) * 0.25f
                                         + nb[1] * 0.6f;
                     float rf_penalty = (1.0f - tracking_rf) * 5.0f + 1.5f;
                     h_sync_shear = (tracking_error_lpf_ * 100.0f) * chaotic_shear * (rf_penalty + unlock * 1.4f);
                }

                float tbe_start = total_h_warp + beltSlip_legacy + hsTearing + h_sync_roll + h_sync_shear + crease_shear;

                // ── Wow-induced TBE bending ──────────────────────────────────
                // Real VHS timebase error creates the classic "flagging" bend
                // at the TOP of the image. The VCR's H-sync pulse provides
                // the timing reference at the start of each scanline. Tape
                // speed error (wow/flutter) causes each successive line to
                // start at a slightly wrong horizontal position.
                //
                // The bending is concentrated at the top and decays downward
                // as the TV's AFC circuit catches up. The bend is NOT uniform
                // — only the upper portion of the image curves.
                // Real VHS flagging is typically 5-15px at the top edge.
                float bend_fraction = std::exp(-float(y) * 0.012f);
                // At y=0: bend_fraction = 1.0 (full displacement)
                // At y=100: bend_fraction = 0.30
                // At y=250: bend_fraction = 0.05 (essentially straight)
                float wow_bend = wow_dev * float(W) * 0.3f * bend_fraction;     // up to ±9.6 px at top
                float flutter_bend = flutter_dev * float(W) * 0.08f * bend_fraction;  // up to ±0.4 px
                tbe_start += wow_bend + flutter_bend;
                
                float tbe_slope = scrape_flutter * 0.25f;
                tbe_slope += crease_slope;

                if (dy > 0.0f && dy < 100.0f) {
                    tbe_slope += std::exp(-dy * 0.05f) * std::cos(dy * 0.2f) * (ep.motor_drag * 25.0f);
                }

                tbe_slope += head_sweep_grad;

                const float linePhase = fPhase + float(y) * kPI;

                // ── Per-line wow/flutter chroma phase offsets ────────────────
                // Real VHS wow/flutter creates very subtle chroma phase errors.
                // The VCR's internal chroma converter shifts the subcarrier
                // frequency uniformly during both record and playback, so the
                // TV's color PLL sees a clean, consistent subcarrier.
                //
                // Residual errors come from:
                //   - Mechanical tape bounce causing micro phase shifts (~0.1-1°)
                //   - Flutter exceeding PLL response bandwidth (~0.05-0.5°)
                // These are tiny — VHS color is dominated by overall hue offset
                // (the TBC HUE knob), not per-line rainbowing.
                //
                // Values are in radians. Max settings produce ~1-2° phase error.
                float line_wow_offset = wow_dev * 0.02f * std::sin(float(y) * 0.08f + tapeTime_ * 1.2f);
                line_wow_offset += nb[2] * (head_wear_deg * 0.05f + unlock * 0.08f);

                // --- ENCODE ---
                float field_phase_offset = inter_field_phase_deg * 0.15f * float(currentField);
                float field_chroma_error = field_chroma_phase_accum_;
                float fm_noise_lpf = 0.0f;
                float d_tail = 0.0f;

                // ── Oxide Dropout Scheduling ────────────────────────────────
                // Real VHS dropouts are random missing oxide patches on the tape.
                // They cause brief signal loss — the scanline goes to black/static
                // for a few pixels to ~100 pixels. Dropouts are more frequent on
                // worn tapes and cause the classic "sparkle" or "snow" artifacts.
                // Rate: 0.01 = occasional sparkle, 0.1 = heavy dropout storm
                float dropout_active = false;
                float dropout_duration = 0.0f;
                float dropout_x_pos = 0.0f;

                // Check if a dropout event occurs this line
                if (dropout_drive > 0.002f) {
                    // Probability of at least one dropout per line
                    float line_dropout_prob = dropout_drive * 0.4f;  // scale to visible
                    std::uniform_real_distribution<float> dropout_rng(0, 1);
                    std::uniform_real_distribution<float> duration_rng(10, 180);
                    std::uniform_real_distribution<float> pos_rng(H_BLANK + 10, H_BLANK + W - 30);
                    if (dropout_rng(rng) < line_dropout_prob) {
                        dropout_active = true;
                        dropout_duration = 10.0f + dropout_rng(rng) * duration_rng(rng);
                        dropout_x_pos = pos_rng(rng);
                    }
                }

                for (int x = 0; x < W_TOTAL; ++x) {
                    float current_tbe = tbe_start + (float(x) / float(W_TOTAL)) * tbe_slope;

                    // Horizontal squeeze/stretch based on tape speed deviation.
                    // When tape runs slower than nominal, the image stretches horizontally
                    // (more time per scanline = wider picture). When faster, it compresses.
                    // Motor degradation adds speed instability, causing the squeeze/stretch
                    // to fluctuate per-frame.
                    float h_scale = 1.0f + speed_dev * (instantSpd < safeTapeSpd ? 0.15f : -0.12f);
                    h_scale += motor_speed_var_;  // Motor speed instability
                    h_scale = std::clamp(h_scale, 0.7f, 1.3f);

                    // Tearing: at low tape speeds, the image tears into horizontal bands
                    // with sudden horizontal offsets. Worse when tape is very slow.
                    float tear_amount = 0.0f;
                    if (safeTapeSpd < 0.9f) {
                        float tear_strength = (0.9f - safeTapeSpd) * 25.0f;
                        // Create 3-5 tear bands across the frame
                        int num_tear_bands = 3 + int(tear_strength);
                        float band_height = float(H) / float(num_tear_bands);
                        int current_band = int(float(y) / band_height);
                        // Each band gets a random offset
                        uint32_t tear_seed = uint32_t(current_band) * 2654435761u + uint32_t(frameNum) * 65537u;
                        tear_seed ^= tear_seed << 13; tear_seed ^= tear_seed >> 17; tear_seed ^= tear_seed << 5;
                        tear_amount = (float(int32_t(tear_seed) % 40) - 20.0f) * tear_strength * 0.3f;
                        // Tearing is more visible in the middle of the frame
                        float y_norm = float(y) / float(H);
                        tear_amount *= std::sin(y_norm * 3.14159f);
                    }

                    // Skewing: at low tape speeds, the image skews (parallelogram distortion)
                    // The amount of skew increases from top to bottom
                    float skew_amount = 0.0f;
                    if (safeTapeSpd < 0.95f) {
                        float skew_strength = (0.95f - safeTapeSpd) * 60.0f;
                        float y_norm = float(y) / float(H);
                        skew_amount = skew_strength * y_norm * y_norm;
                        // Add slight wobble to the skew
                        skew_amount += std::sin(tapeTime_ * 2.0f + y_norm * 6.0f) * skew_strength * 0.15f;
                    }

                    float xSrcCentered = (float(x) - float(W_TOTAL) * 0.5f) / h_scale + float(W_TOTAL) * 0.5f;
                    float xSrc = xSrcCentered - current_tbe - tear_amount - skew_amount;
                    bool inLine = xSrc >= 0.0f && xSrc < float(W_TOTAL);
                    bool activeSrc = inLine && xSrc >= H_BLANK && xSrc < float(H_BLANK + W);

                    // ── Encode at nominal subcarrier frequency ────────────────
                    // The VCR handles subcarrier frequency conversion internally.
                    // We encode at the nominal 227.5 cycles/line with tiny
                    // per-line phase offsets simulating mechanical residuals.
                    float theta = linePhase + float(x) * kSC_PX + line_wow_offset + field_phase_offset;

                    float Y = 0.0f, I = 0.0f, Q = 0.0f;

                    if (inLine && xSrc < 60) {
                        Y = -0.3f; // Sync
                    } else if (inLine && xSrc >= 70 && xSrc < 110) {
                        Y = 0.0f;  // Burst
                        I = 0.0f; Q = 0.35f;
                    } else if (activeSrc) {
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

                        I *= chroma_attenuation;
                        Q *= chroma_attenuation;
                        Y += std::normal_distribution<float>(0.0f, 1.0f)(rng) * (luma_noise_level * 0.02f);
                        I += std::normal_distribution<float>(0.0f, 1.0f)(rng) * (chroma_noise_level * 0.01f);
                        Q += std::normal_distribution<float>(0.0f, 1.0f)(rng) * (chroma_noise_level * 0.01f);

                        if (motor_defect > 0.05f) {
                            Y += motor_rf_noise * 0.03f;
                        }

                        if (fm_carrier_noise_deg > 0.005f) {
                            float fm_dev = fm_carrier_noise_deg * 0.012f;
                            fm_noise_lpf += fm_dev * (std::uniform_real_distribution<float>(-1, 1)(rng) - fm_noise_lpf);
                            Y += fm_noise_lpf * 0.3f;
                            float fm_chroma_mod = fm_noise_lpf * 0.15f;
                            I += fm_chroma_mod * std::cos(theta * 0.5f);
                            Q += fm_chroma_mod * std::sin(theta * 0.5f);
                        }
                        
                        if (d_tail > 0.01f) {
                            Y = Y * (1.0f - d_tail) + (d_tail > 0.8f ? 0.8f : -0.2f);
                            I *= (1.0f - d_tail); Q *= (1.0f - d_tail);
                            d_tail *= 0.92f;
                        }
                    }
                    if (tracking_rf < 0.95f && activeSrc) {
                         Y *= tracking_rf; I *= tracking_rf; Q *= tracking_rf;
                         float noise_amp = 1.0f - tracking_rf;
                         Y += std::uniform_real_distribution<float>(-0.8f, 0.8f)(rng) * noise_amp;
                         if (noise_amp > 0.35f) { I *= 0.2f; Q *= 0.2f; }
                    }

                    if (dropout_active && activeSrc) {
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

                    comp_line[x] = Y + I * std::cos(theta) - Q * std::sin(theta);
                }

                // ── FM Luma Encode → Tape → Decode (Phase 2) ────────────────
                // Disabled: FM processing artifacts will be added in decode section.
                // FM encode/decode of composite signal causes chroma corruption.

                // ── Color-Under Processing (Phase 3) ──────────────────────────
                // Disabled: chroma artifacts will be added in decode section.
                // Quadrature demodulation of composite causes line-by-line artifacts.

                // --- DECODE (Closed-Loop PLL) ---
                float phase_accum = 0.0f;

                float sumI = 0.f, sumQ = 0.f;
                for (int x = 70; x < 110; ++x) {
                    float refTheta = linePhase + float(x) * kSC_PX;
                    sumI += comp_line[x] * std::cos(refTheta);
                    sumQ -= comp_line[x] * std::sin(refTheta);
                }
                phase_accum = std::atan2(sumQ, sumI) - (kPI / 2.0f);

                phase_accum += field_chroma_error;

                // Real TV PLL loses lock during speed changes. The burst signal
                // is shifted in frequency by tape speed mismatch, causing the PLL
                // to accumulate a uniform per-line phase error. This reduces chroma
                // amplitude (I/Q demodulation with phase offset loses energy).
                // Uniform across the scanline — no spatial hue variation.
                float pll_speed_error = speed_dev * 1.2f
                    + std::abs(instantSpd - 1.0f) * 0.4f;

                float lpY = 0.f, si = 0.f, sq = 0.f;
                const float lpfA = 0.35f;
                const float chrA = 0.12f;

                float prev_r = 0.0f, prev_g = 0.0f, prev_b = 0.0f;
                bool has_prev = false;
                for (int out_x = 0; out_x < W; ++out_x) {
                    int x = out_x + H_BLANK;
                    float sig = comp_line[x];

                    // Color burst (x=70-110) sets the subcarrier phase reference.
                    // The PLL extracts line_wow_offset from the burst and stores
                    // it in phase_accum. During speed changes the PLL accumulates
                    // a uniform per-line phase error (pll_speed_error), reducing
                    // chroma amplitude without spatial hue variation.
                    float decTheta = linePhase + float(x) * kSC_PX + phase_accum
                                   + field_phase_offset + pll_speed_error;

                lpY += lpfA * (sig - lpY);

                // 3. Demodulate Chroma
                float chromaSig = sig - lpY;
                float rawI = 2.0f * chromaSig * std::cos(decTheta);
                float rawQ = -2.0f * chromaSig * std::sin(decTheta);

                si += chrA * (rawI - si);
                sq += chrA * (rawQ - sq);

                // Azimuth crosstalk: adjacent track chroma leaks through with
                // phase inversion. 6-head VCRs reduce this by 25% via Hi-Fi heads.
                // The crosstalk appears as a ghost of the previous line's chroma
                // with a 180-degree phase shift (azimuth rejection failure).
                if (chroma_crosstalk_deg > 0.005f) {
                    float crosstalk_phase = kPI;  // 180-degree phase shift
                    float prevLineI = prev_line_luma_[0];  // store I from prev line
                    float prevLineQ = prev_line_luma_[1];  // store Q from prev line
                    float xtI = prevLineI * std::cos(crosstalk_phase) - prevLineQ * std::sin(crosstalk_phase);
                    float xtQ = prevLineI * std::sin(crosstalk_phase) + prevLineQ * std::cos(crosstalk_phase);
                    float xt_amp = chroma_crosstalk_deg * 0.15f;
                    si += xtI * xt_amp;
                    sq += xtQ * xt_amp;
                }

                // 4. Transform back to RGB
                float r = lpY + 0.956f * si + 0.621f * sq;
                float g = lpY - 0.272f * si - 0.647f * sq;
                float b = lpY - 1.106f * si + 1.703f * sq;

                float snow = noise_buf_[(H_ - 1 - y) * W_TOTAL_ + (W_TOTAL_ - 1 - out_x)] * (wear_noise * 0.12f);
                r += snow;
                g += snow;
                b += snow;

                if (wear_noise > 0.35f && std::uniform_real_distribution<float>(0.0f, 1.0f)(rng) < wear_noise * 0.03f) {
                    float spark = std::uniform_real_distribution<float>(-0.35f, 0.55f)(rng);
                    r += spark;
                    g += spark;
                    b += spark;
                }

                float smear = std::clamp((metal_deg * 0.6f + binder_deg * 0.4f) / bw_sharpness_factor, 0.0f, 0.85f);
                if (has_prev && smear > 0.001f) {
                    r = r * (1.0f - smear) + prev_r * smear;
                    g = g * (1.0f - smear) + prev_g * smear;
                    b = b * (1.0f - smear) + prev_b * smear;
                }
                prev_r = r;
                prev_g = g;
                prev_b = b;
                has_prev = true;

                dr[out_x * 3 + 0] = (uchar)std::clamp(b * 255.f, 0.f, 255.f);
                dr[out_x * 3 + 1] = (uchar)std::clamp(g * 255.f, 0.f, 255.f);
                dr[out_x * 3 + 2] = (uchar)std::clamp(r * 255.f, 0.f, 255.f);
            }
            // Save final I/Q for next line's azimuth crosstalk
            prev_line_luma_[0] = si;
            prev_line_luma_[1] = sq;
        }
        }
#endif
}
