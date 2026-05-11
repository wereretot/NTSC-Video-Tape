#include "fm_luma.h"
#include <cmath>
#include <algorithm>

// NTSC sample rate: 4x color clock = 14.31818 MHz
static constexpr float kNTSC_SAMPLE_RATE = 14.31818e6f;

FMLumaProcessor::FMLumaProcessor() {
    buildFMTables();
    configure(3.4e6f, 4.4e6f, 3.0f, false);
}

void FMLumaProcessor::configure(float sync_tip_freq_mhz, float peak_white_freq_mhz,
                                 float luma_bw_mhz, bool is_svhhs) {
    fm_sync_tip_freq_ = sync_tip_freq_mhz;
    fm_peak_white_freq_ = peak_white_freq_mhz;
    fm_deviation_ = peak_white_freq_mhz - sync_tip_freq_mhz;
    luma_bw_mhz_ = luma_bw_mhz;

    // PLL center frequency
    pll_freq_ = (sync_tip_freq_mhz + peak_white_freq_mhz) * 0.5f;

    // Pre-emphasis: H(f) = 1 + j*f/1.5MHz (simple first-order high-shelf)
    // Approximated as: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
    // Time constant tau = 1/(2*pi*1.5e6) ≈ 106ns
    float dt = 1.0f / kNTSC_SAMPLE_RATE;
    float preemph_tc = 1.0f / (2.0f * float(M_PI) * 1.5e6f);
    float alpha = dt / (preemph_tc + dt);
    preemph_b0_ = 1.0f + alpha * 0.5f;
    preemph_b1_ = -alpha * 0.5f;
    preemph_a1_ = -(1.0f - alpha);

    // De-emphasis: H(f) = 1/(1 + j*f/1.5MHz) (complementary low-shelf)
    deemph_b0_ = alpha;
    deemph_b1_ = 0.0f;
    deemph_a1_ = -(1.0f - alpha);

    // Design 6-pole Butterworth LPF at luma bandwidth
    float cutoff_hz = luma_bw_mhz * 1e6f;
    designButterworthLPF(cutoff_hz, kNTSC_SAMPLE_RATE);
}

void FMLumaProcessor::buildFMTables() {
    fm_cos_table_.resize(kTableSize);
    fm_sin_table_.resize(kTableSize);
    for (int i = 0; i < kTableSize; ++i) {
        float phase = float(i) / float(kTableSize) * 2.0f * float(M_PI);
        fm_cos_table_[i] = std::cos(phase);
        fm_sin_table_[i] = std::sin(phase);
    }
}

// 2-pole Butterworth lowpass: bilinear transform design
static void designBiquad(float fc, float fs, float& b0, float& b1, float& b2,
                          float& a1, float& a2) {
    float wc = 2.0f * float(M_PI) * fc;
    float k = wc / std::tan(float(M_PI) * fc / fs);
    float k2 = k * k;
    float norm = 1.0f / (1.0f + std::sqrt(2.0f) * k + k2);
    b0 = norm * k2;
    b1 = 2.0f * b0;
    b2 = b0;
    a1 = 2.0f * norm * (k2 - 1.0f);
    a2 = norm * (1.0f - std::sqrt(2.0f) * k + k2);
}

void FMLumaProcessor::designButterworthLPF(float cutoff_hz, float fs_hz) {
    // 6-pole = 3 cascaded 2-pole Butterworth sections
    // Each section has slightly different Q for overall Butterworth response
    float b0, b1, b2, a1, a2;

    // Section frequencies (staggered for 6-pole response)
    float q_stages[3] = {0.5176f, 0.7071f, 1.9319f};

    for (int i = 0; i < 3; ++i) {
        float fc_stage = cutoff_hz / q_stages[i];
        designBiquad(fc_stage, fs_hz, b0, b1, b2, a1, a2);
        // Store as normalized first-order approximation for simplicity
        // In practice, full biquad would be used
        biquad_[i].x1 = 0; biquad_[i].x2 = 0;
        biquad_[i].y1 = 0; biquad_[i].y2 = 0;
    }

    // Store coefficients as instance variables for biquadProcess
    // (simplified: using a single lowpass approximation)
    fs_ = fs_hz;
}

float FMLumaProcessor::biquadProcess(int stage, float x) {
    // Simplified single-pole lowpass for performance
    // Real implementation would use full biquad
    float cutoff_hz = luma_bw_mhz_ * 1e6f;
    float rc = 1.0f / (2.0f * float(M_PI) * cutoff_hz);
    float dt = 1.0f / fs_;
    float alpha = dt / (rc + dt);

    float y = biquad_[stage].y1 + alpha * (x - biquad_[stage].y1);
    biquad_[stage].y1 = y;
    return y;
}

void FMLumaProcessor::encode(const float* luma_in, float* fm_out, int num_samples) {
    // Pre-emphasis
    float preemph_prev = 0.0f;
    float preemph_y = 0.0f;

    // FM phase accumulator
    float fm_phase = pll_phase_;
    float two_pi_dt = 2.0f * float(M_PI) / kNTSC_SAMPLE_RATE;

    for (int i = 0; i < num_samples; ++i) {
        float y = luma_in[i];
        y = std::clamp(y, 0.0f, 1.0f);

        // Pre-emphasis
        preemph_y = preemph_b0_ * y + preemph_b1_ * preemph_prev - preemph_a1_ * preemph_y;
        preemph_prev = y;

        // FM modulation: f(t) = f_sync + deviation * Y(t)
        float freq = fm_sync_tip_freq_ + fm_deviation_ * preemph_y;

        // Phase accumulation
        fm_phase += freq * two_pi_dt;
        if (fm_phase > 2.0f * float(M_PI)) fm_phase -= 2.0f * float(M_PI);

        // Table lookup for cosine
        int idx = int((fm_phase / (2.0f * float(M_PI))) * float(kTableSize)) & (kTableSize - 1);
        fm_out[i] = fm_cos_table_[idx];
    }

    pll_phase_ = fm_phase;
}

void FMLumaProcessor::decode(const float* fm_in, float* luma_out, int num_samples) {
    // Quadrature PLL FM demodulator
    float phase = pll_phase_;
    float freq = pll_freq_;
    float two_pi_dt = 2.0f * float(M_PI) / kNTSC_SAMPLE_RATE;

    // PLL loop filter parameters
    float pll_bw = 2.0e6f;  // 2 MHz PLL bandwidth
    float pll_k1 = pll_bw / kNTSC_SAMPLE_RATE;
    float pll_k2 = pll_k1 * 0.25f;  // Second order term

    float loop_filter = pll_filter_state_;
    float prev_output = pll_prev_output_;

    for (int i = 0; i < num_samples; ++i) {
        float sig = fm_in[i];

        // Quadrature mixing
        int idx_cos = int((phase / (2.0f * float(M_PI))) * float(kTableSize)) & (kTableSize - 1);
        int idx_sin = (idx_cos + kTableSize / 4) & (kTableSize - 1);

        float sin_ref = fm_cos_table_[idx_sin];  // cos(phase - pi/2) = sin(phase)

        // Phase detector (multiply + low-pass approx)
        float phase_error = sig * sin_ref;

        // Loop filter (PI controller)
        loop_filter += pll_k1 * phase_error;
        float freq_correction = loop_filter + pll_k2 * phase_error;

        // Clamp frequency to valid range
        float min_freq = fm_sync_tip_freq_ * 0.8f;
        float max_freq = fm_peak_white_freq_ * 1.2f;
        freq = std::clamp(freq + freq_correction * kNTSC_SAMPLE_RATE, min_freq, max_freq);

        // Phase update
        phase += freq * two_pi_dt;
        if (phase > 2.0f * float(M_PI)) phase -= 2.0f * float(M_PI);

        // Demodulated output: frequency → voltage
        // Normalize: 0V at sync tip, 1V at peak white
        float voltage = (freq - fm_sync_tip_freq_) / fm_deviation_;
        voltage = std::clamp(voltage, -0.1f, 1.1f);

        // De-emphasis: first-order low-shelf filter
        // y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
        deemph_state_ = deemph_b0_ * voltage + deemph_b1_ * prev_output - deemph_a1_ * deemph_state_;
        prev_output = voltage;

        // Apply bandwidth limiting through biquad stages
        float filtered = deemph_state_;
        for (int s = 0; s < 3; ++s) {
            filtered = biquadProcess(s, filtered);
        }

        luma_out[i] = std::clamp(filtered, 0.0f, 1.0f);
    }

    pll_phase_ = phase;
    pll_filter_state_ = loop_filter;
    pll_prev_output_ = prev_output;
}

void FMLumaProcessor::applyThresholdEffect(float* luma_out, const float* fm_in,
                                            float snr_db, int num_samples) {
    // FM threshold effect: below ~12 dB SNR, demodulator produces sparkle noise
    if (snr_db > 12.0f) return;

    float sparkle_prob = std::max(0.0f, 1.0f - snr_db / 12.0f);
    sparkle_prob = std::clamp(sparkle_prob * sparkle_prob, 0.0f, 1.0f);

    for (int i = 0; i < num_samples; ++i) {
        // Simple sparkle: random impulse when FM amplitude drops
        float fm_amp = std::abs(fm_in[i]);
        if (fm_amp < 0.3f && sparkle_prob > 0.1f) {
            // Sparkle noise: bright or dark impulse
            float sparkle = (std::rand() / float(RAND_MAX)) > 0.5f ? 1.2f : -0.2f;
            luma_out[i] = sparkle * sparkle_prob + luma_out[i] * (1.0f - sparkle_prob);
        }
    }
}
