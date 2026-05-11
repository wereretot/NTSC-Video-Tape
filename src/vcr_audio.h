#pragma once

#include <vector>
#include <cstdint>
#include <cmath>
#include <algorithm>

// ── VCR Audio Processing ──────────────────────────────────────────
// Models the three audio systems in VHS/S-VHS:
//   1. Hi-Fi Audio: FM multiplex on video heads (deep recording)
//      - Left channel: 1.44 MHz carrier, ±150 kHz deviation
//      - Right channel: 1.80 MHz carrier, ±150 kHz deviation
//      - 80+ dB dynamic range, 20Hz-20kHz response
//   2. Linear Audio: Longitudinal edge track
//      - 50 Hz - 10 kHz bandwidth
//      - 40-50 dB dynamic range, tape hiss prominent
//   3. Control Track: CTL pulses for servo sync
//      - 30 Hz (NTSC) pulses at tape edge
//      - Used for tracking and speed reference

class VCRAudioProcessor {
public:
    VCRAudioProcessor();

    // Process Hi-Fi audio: FM encode → tape degradation → FM decode
    // Returns processed stereo samples interleaved (L, R, L, R, ...)
    void processHiFi(const float* input_l, const float* input_r,
                     float* output_l, float* output_r,
                     int num_samples, float tape_speed, float snr_db);

    // Process linear audio: bandpass → tape noise → wow/flutter
    void processLinear(const float* input, float* output,
                       int num_samples, float tape_speed, float snr_db);

    // Generate control track pulses (30 Hz for NTSC)
    // Returns 1.0f when pulse is active, 0.0f otherwise
    float generateControlPulse(float tape_time, float tape_speed);

    // Get Hi-Fi carrier frequencies
    float getHiFiLeftCarrier() const { return hifi_left_carrier_; }
    float getHiFiRightCarrier() const { return hifi_right_carrier_; }

    // Configure for S-VHS (slightly different Hi-Fi frequencies)
    void setSVHS(bool is_svhs) {
        is_svhs_ = is_svhs;
        hifi_left_carrier_ = is_svhs ? 1.48e6f : 1.44e6f;
        hifi_right_carrier_ = is_svhs ? 1.84e6f : 1.80e6f;
    }

    // Configure audio mode
    enum class AudioMode {
        HiFiOnly,       // Hi-Fi audio only (best quality)
        LinearOnly,     // Linear audio only (tape hiss)
        HiFiAndLinear,  // Both tracks (Hi-Fi primary, linear fallback)
        Mono            // Mono linear track
    };

    void setAudioMode(AudioMode mode) { audio_mode_ = mode; }
    AudioMode getAudioMode() const { return audio_mode_; }

private:
    static constexpr float kSAMPLE_RATE = 48000.0f;  // Audio sample rate
    static constexpr float kNTSC_FRAME_RATE = 29.97f;
    static constexpr float kCTL_FREQ = 30.0f;         // Control track pulse frequency

    // Hi-Fi FM parameters
    float hifi_left_carrier_ = 1.44e6f;
    float hifi_right_carrier_ = 1.80e6f;
    float hifi_deviation_ = 150e3f;  // ±150 kHz
    bool is_svhs_ = false;

    // FM processing state
    float hifi_fm_phase_l_ = 0.0f;
    float hifi_fm_phase_r_ = 0.0f;
    float hifi_pll_phase_l_ = 0.0f;
    float hifi_pll_phase_r_ = 0.0f;
    float hifi_pll_freq_l_ = 1.44e6f;
    float hifi_pll_freq_r_ = 1.80e6f;

    // Linear audio state
    float linear_wow_phase_ = 0.0f;
    float linear_flutter_phase_ = 0.0f;

    // Control track state
    float ctl_phase_ = 0.0f;

    AudioMode audio_mode_ = AudioMode::HiFiAndLinear;

    // Hi-Fi FM encode/decode
    void hifiFmEncode(const float* input_l, const float* input_r,
                      float* fm_signal, int num_samples);
    void hifiFmDecode(const float* fm_signal,
                      float* output_l, float* output_r,
                      int num_samples, float snr_db);

    // Tape degradation for audio
    void applyTapeNoise(float* signal, int num_samples, float snr_db);
    void applyWowFlutter(float* signal, int num_samples, float tape_speed);
};
