#pragma once

#include <cmath>
#include <vector>
#include <cstdint>
#include <algorithm>

// ── Color-Under Processing ──────────────────────────────────────────
// Models the VHS/S-VHS color-under recording process:
//   Record: 3.58 MHz chroma → 629 kHz downconversion (with 90° phase switching)
//   Playback: 629 kHz → 3.58 MHz upconversion
//
// VHS uses 629 kHz color-under carrier (f_sc / 5.68 ≈ 631.3 kHz)
// The 3:2 phase switching advances chroma phase 90° per line in a 4-line cycle
// to reduce adjacent track crosstalk via azimuth rejection.

class ColorUnderProcessor {
public:
    ColorUnderProcessor();

    // Downconvert 3.58 MHz chroma to 629 kHz for recording
    // chroma_i/q are the I/Q components at 3.58 MHz
    // Returns downconverted I/Q at 629 kHz
    void downconvert(const float* chroma_i_in, const float* chroma_q_in,
                     float* chroma_i_out, float* chroma_q_out,
                     int line_num, int num_samples);

    // Upconvert 629 kHz back to 3.58 MHz for playback
    void upconvert(const float* chroma_i_in, const float* chroma_q_in,
                   float* chroma_i_out, float* chroma_q_out,
                   int line_num, int num_samples);

    // Model adjacent-track chroma crosstalk with phase inversion
    // Simulates the chroma signal from the adjacent helical track
    // leaking through (reduced by azimuth rejection)
    void applyCrosstalk(float* target_i, float* target_q,
                        const float* adjacent_i, const float* adjacent_q,
                        float crosstalk_level, int line_num, int num_samples);

    // Get the current downconverted carrier frequency
    float getDownconvertFreq() const { return down_freq_; }

    // Configure for S-VHS (wider chroma bandwidth)
    void setSVHS(bool is_svhs) {
        is_svhs_ = is_svhs;
        chroma_bw_hz_ = is_svhs ? 1.0e6f : 0.5e6f;
    }

private:
    static constexpr float kNTSC_SC_FREQ = 3579545.0f;  // 3.579545 MHz
    static constexpr float kVHS_CU_FREQ = 631300.0f;     // ~629 kHz VHS color-under

    float down_freq_ = kVHS_CU_FREQ;
    float up_freq_ = kNTSC_SC_FREQ;
    float chroma_bw_hz_ = 0.5e6f;
    bool is_svhs_ = false;

    // Phase switching state
    int phase_switch_pattern_[4] = {0, 1, 2, 3};  // 90° increments per line

    // Phase accumulators
    float down_phase_ = 0.0f;
    float up_phase_ = 0.0f;

    // Lookup tables for down/up conversion
    std::vector<float> cu_cos_table_;
    std::vector<float> cu_sin_table_;
    static constexpr int kTableSize = 2048;

    void buildTables();
    int getLinePhaseShift(int line_num);  // Returns 0, 1, 2, or 3 (x 90°)
};
