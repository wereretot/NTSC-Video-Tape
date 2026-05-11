#pragma once

#include <cmath>
#include <vector>
#include <cstdint>
#include <algorithm>

// ── FM Luma Processing ──────────────────────────────────────────────
// Models the VHS/S-VHS FM luma recording and playback chain:
//   Record: Y(t) → Pre-emphasis → FM Modulation → Tape
//   Playback: Tape → FM Demodulation → De-emphasis → Y(t)

class FMLumaProcessor {
public:
    FMLumaProcessor();

    // Configure FM parameters
    void configure(float sync_tip_freq_mhz, float peak_white_freq_mhz,
                   float luma_bw_mhz, bool is_svhhs);

    // Encode: luma → FM signal
    void encode(const float* luma_in, float* fm_out, int num_samples);

    // Decode: FM signal → luma (quadrature PLL demodulator)
    void decode(const float* fm_in, float* luma_out, int num_samples);

    // FM threshold effect: sparkle noise below threshold SNR
    void applyThresholdEffect(float* luma_out, const float* fm_in,
                              float snr_db, int num_samples);

private:
    // FM parameters
    float fm_sync_tip_freq_ = 3.4e6f;      // Hz
    float fm_peak_white_freq_ = 4.4e6f;    // Hz
    float fm_deviation_ = 1.0e6f;          // Hz
    float luma_bw_mhz_ = 3.0f;

    // Pre-emphasis / De-emphasis
    float preemph_a1_ = 0.0f, preemph_b0_ = 1.0f, preemph_b1_ = 0.0f;
    float deemph_a1_ = 0.0f, deemph_b0_ = 1.0f, deemph_b1_ = 0.0f;

    // PLL state
    float pll_phase_ = 0.0f;
    float pll_freq_ = 3.9e6f;     // Center frequency
    float pll_filter_state_ = 0.0f;
    float pll_prev_output_ = 0.0f;

    // De-emphasis state
    float deemph_state_ = 0.0f;

    // Butterworth LPF state (6-pole = 3 stages of 2-pole)
    struct BiquadState {
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    };
    BiquadState biquad_[3];
    float fs_ = 14.31818e6f;  // NTSC color clock * 4 ≈ 57.27 MHz sample rate

    // Lookup tables
    std::vector<float> fm_cos_table_;
    std::vector<float> fm_sin_table_;
    static constexpr int kTableSize = 4096;

    void buildFMTables();
    void designButterworthLPF(float cutoff_hz, float fs_hz);
    float biquadProcess(int stage, float x);
};
