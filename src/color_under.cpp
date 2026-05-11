#include "color_under.h"

// Sample rate: same as FM luma processing
static constexpr float kSAMPLE_RATE = 14.31818e6f;

ColorUnderProcessor::ColorUnderProcessor() {
    buildTables();
}

void ColorUnderProcessor::buildTables() {
    cu_cos_table_.resize(kTableSize);
    cu_sin_table_.resize(kTableSize);
    for (int i = 0; i < kTableSize; ++i) {
        float phase = float(i) / float(kTableSize) * 2.0f * float(M_PI);
        cu_cos_table_[i] = std::cos(phase);
        cu_sin_table_[i] = std::sin(phase);
    }
}

int ColorUnderProcessor::getLinePhaseShift(int line_num) {
    // VHS color-under uses 90° phase switching per line
    // Pattern repeats every 4 lines: 0°, 90°, 180°, 270°
    return (line_num / 2) % 4;  // Switch every 2 lines (VHS pattern)
}

void ColorUnderProcessor::downconvert(const float* chroma_i_in, const float* chroma_q_in,
                                       float* chroma_i_out, float* chroma_q_out,
                                       int line_num, int num_samples) {
    // Phase shift for this line (90° increments)
    int phase_idx = getLinePhaseShift(line_num);
    float phase_shift = float(phase_idx) * float(M_PI) * 0.5f;  // 90° = pi/2

    // Downconversion: mix 3.58 MHz signal with LO at (3.58 - 0.631) MHz
    // to produce 629 kHz difference frequency
    float lo_freq = kNTSC_SC_FREQ - down_freq_;
    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;
    float phase = down_phase_;

    for (int i = 0; i < num_samples; ++i) {
        float i_in = chroma_i_in[i];
        float q_in = chroma_q_in[i];

        // LO signal at difference frequency
        int idx = int((phase / (2.0f * float(M_PI))) * float(kTableSize)) & (kTableSize - 1);
        float lo_cos = cu_cos_table_[idx];
        float lo_sin = cu_sin_table_[idx];

        // Mix: I/Q rotation by phase shift + downconversion
        float cos_shift = std::cos(phase_shift);
        float sin_shift = std::sin(phase_shift);

        // Rotate input by line phase shift
        float i_rot = i_in * cos_shift - q_in * sin_shift;
        float q_rot = i_in * sin_shift + q_in * cos_shift;

        // Downconvert: multiply by LO (takes difference frequency component)
        chroma_i_out[i] = i_rot * lo_cos + q_rot * lo_sin;
        chroma_q_out[i] = -i_rot * lo_sin + q_rot * lo_cos;

        phase += lo_freq * two_pi_dt;
        if (phase > 2.0f * float(M_PI)) phase -= 2.0f * float(M_PI);
    }

    down_phase_ = phase;
}

void ColorUnderProcessor::upconvert(const float* chroma_i_in, const float* chroma_q_in,
                                     float* chroma_i_out, float* chroma_q_out,
                                     int line_num, int num_samples) {
    // Reverse the line phase shift
    int phase_idx = getLinePhaseShift(line_num);
    float phase_shift = -float(phase_idx) * float(M_PI) * 0.5f;  // Reverse direction

    // Upconversion: mix 629 kHz with LO to restore 3.58 MHz
    float lo_freq = kNTSC_SC_FREQ - down_freq_;
    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;
    float phase = up_phase_;

    for (int i = 0; i < num_samples; ++i) {
        float i_in = chroma_i_in[i];
        float q_in = chroma_q_in[i];

        // LO signal
        int idx = int((phase / (2.0f * float(M_PI))) * float(kTableSize)) & (kTableSize - 1);
        float lo_cos = cu_cos_table_[idx];
        float lo_sin = cu_sin_table_[idx];

        // Upconvert
        float i_mixed = i_in * lo_cos - q_in * lo_sin;
        float q_mixed = i_in * lo_sin + q_in * lo_cos;

        // Reverse line phase rotation
        float cos_shift = std::cos(phase_shift);
        float sin_shift = std::sin(phase_shift);
        chroma_i_out[i] = i_mixed * cos_shift - q_mixed * sin_shift;
        chroma_q_out[i] = i_mixed * sin_shift + q_mixed * cos_shift;

        phase += lo_freq * two_pi_dt;
        if (phase > 2.0f * float(M_PI)) phase -= 2.0f * float(M_PI);
    }

    up_phase_ = phase;
}

void ColorUnderProcessor::applyCrosstalk(float* target_i, float* target_q,
                                          const float* adjacent_i, const float* adjacent_q,
                                          float crosstalk_level, int line_num, int num_samples) {
    if (crosstalk_level < 0.001f) return;

    // Adjacent track crosstalk has phase inversion due to azimuth difference
    // The azimuth angle difference (±6°) causes ~25-30 dB rejection at luma frequencies
    // but only ~10-15 dB at 629 kHz color-under frequency

    // Phase inversion for crosstalk (180° shift due to opposite azimuth)
    int target_phase = getLinePhaseShift(line_num);
    float crosstalk_amp = crosstalk_level * 0.3f;  // Scale to visible level

    for (int i = 0; i < num_samples; ++i) {
        // Adjacent track signal is phase-inverted (azimuth rejection failure)
        float xt_i = -adjacent_i[i] * crosstalk_amp;
        float xt_q = -adjacent_q[i] * crosstalk_amp;

        // Add to target with slight delay (tape geometry effect)
        target_i[i] += xt_i;
        target_q[i] += xt_q;
    }
}
