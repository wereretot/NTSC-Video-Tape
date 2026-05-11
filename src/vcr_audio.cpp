#include "vcr_audio.h"

VCRAudioProcessor::VCRAudioProcessor() {
    // Initialize phases to random values to avoid startup transients
    hifi_fm_phase_l_ = 0.0f;
    hifi_fm_phase_r_ = 0.0f;
    hifi_pll_phase_l_ = 0.0f;
    hifi_pll_phase_r_ = 0.0f;
}

void VCRAudioProcessor::hifiFmEncode(const float* input_l, const float* input_r,
                                      float* fm_signal, int num_samples) {
    // FM encode: f(t) = f_carrier + k_deviation * audio(t)
    // Audio is normalized to [-1, 1], so deviation is ±150 kHz
    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;
    float dev_scale_l = hifi_deviation_ * two_pi_dt;
    float dev_scale_r = hifi_deviation_ * two_pi_dt;

    float phase_l = hifi_fm_phase_l_;
    float phase_r = hifi_fm_phase_r_;
    float carrier_freq_l = hifi_left_carrier_ * two_pi_dt;
    float carrier_freq_r = hifi_right_carrier_ * two_pi_dt;

    for (int i = 0; i < num_samples; ++i) {
        float audio_l = std::clamp(input_l[i], -1.0f, 1.0f);
        float audio_r = std::clamp(input_r[i], -1.0f, 1.0f);

        // FM modulation
        float freq_l = carrier_freq_l + dev_scale_l * audio_l;
        float freq_r = carrier_freq_r + dev_scale_r * audio_r;

        // Generate FM carrier signals (multiplexed)
        float carrier_l = std::sin(phase_l);
        float carrier_r = std::sin(phase_r);

        // Combine both channels (frequency division multiplex)
        fm_signal[i * 2] = carrier_l * 0.5f;      // Left channel samples
        fm_signal[i * 2 + 1] = carrier_r * 0.5f;   // Right channel samples

        phase_l += freq_l;
        phase_r += freq_r;

        // Keep phases bounded
        if (phase_l > 2.0f * float(M_PI)) phase_l -= 2.0f * float(M_PI);
        if (phase_r > 2.0f * float(M_PI)) phase_r -= 2.0f * float(M_PI);
    }

    hifi_fm_phase_l_ = phase_l;
    hifi_fm_phase_r_ = phase_r;
}

void VCRAudioProcessor::hifiFmDecode(const float* fm_signal,
                                      float* output_l, float* output_r,
                                      int num_samples, float snr_db) {
    // Quadrature PLL FM demodulator
    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;
    float pll_k1 = 0.01f;  // PLL loop filter gain
    float pll_k2 = 0.001f;

    float phase_l = hifi_pll_phase_l_;
    float phase_r = hifi_pll_phase_r_;
    float freq_l = hifi_pll_freq_l_ * two_pi_dt;
    float freq_r = hifi_pll_freq_r_ * two_pi_dt;
    float filter_state_l = 0.0f;
    float filter_state_r = 0.0f;

    float min_freq_l = hifi_left_carrier_ * 0.7f * two_pi_dt;
    float max_freq_l = hifi_left_carrier_ * 1.3f * two_pi_dt;
    float min_freq_r = hifi_right_carrier_ * 0.7f * two_pi_dt;
    float max_freq_r = hifi_right_carrier_ * 1.3f * two_pi_dt;

    float deviation_norm = 1.0f / hifi_deviation_;

    // Noise amplitude based on SNR
    float noise_amp = std::pow(10.0f, -snr_db / 20.0f);

    for (int i = 0; i < num_samples; ++i) {
        // Left channel demodulation
        {
            // Simple oscillator
            float sin_ref = std::sin(phase_l);

            float sig = fm_signal[i * 2];

            // Phase detector
            float phase_error = sig * sin_ref;

            // Loop filter (PI controller)
            filter_state_l += pll_k1 * phase_error;
            float freq_correction = filter_state_l + pll_k2 * phase_error;

            // Update frequency
            freq_l = std::clamp(freq_l + freq_correction, min_freq_l, max_freq_l);

            // Demodulated output: frequency → audio
            float audio = (freq_l - hifi_left_carrier_ * two_pi_dt) * deviation_norm;
            output_l[i] = std::clamp(audio, -1.0f, 1.0f);

            // Add tape noise
            output_l[i] += (float(rand()) / float(RAND_MAX) - 0.5f) * noise_amp * 2.0f;

            // Phase update
            phase_l += freq_l;
            if (phase_l > 2.0f * float(M_PI)) phase_l -= 2.0f * float(M_PI);
        }

        // Right channel demodulation
        {
            float sin_ref = std::sin(phase_r);

            float sig = fm_signal[i * 2 + 1];

            // Phase detector
            float phase_error = sig * sin_ref;

            // Loop filter
            filter_state_r += pll_k1 * phase_error;
            float freq_correction = filter_state_r + pll_k2 * phase_error;

            // Update frequency
            freq_r = std::clamp(freq_r + freq_correction, min_freq_r, max_freq_r);

            // Demodulated output
            float audio = (freq_r - hifi_right_carrier_ * two_pi_dt) * deviation_norm;
            output_r[i] = std::clamp(audio, -1.0f, 1.0f);

            // Add tape noise
            output_r[i] += (float(rand()) / float(RAND_MAX) - 0.5f) * noise_amp * 2.0f;

            // Phase update
            phase_r += freq_r;
            if (phase_r > 2.0f * float(M_PI)) phase_r -= 2.0f * float(M_PI);
        }
    }

    hifi_pll_phase_l_ = phase_l;
    hifi_pll_phase_r_ = phase_r;
    hifi_pll_freq_l_ = freq_l / two_pi_dt;
    hifi_pll_freq_r_ = freq_r / two_pi_dt;
}

void VCRAudioProcessor::applyTapeNoise(float* signal, int num_samples, float snr_db) {
    float noise_amp = std::pow(10.0f, -snr_db / 20.0f);
    for (int i = 0; i < num_samples; ++i) {
        signal[i] += (float(rand()) / float(RAND_MAX) - 0.5f) * noise_amp * 2.0f;
    }
}

void VCRAudioProcessor::applyWowFlutter(float* signal, int num_samples, float tape_speed) {
    // Wow: slow speed variations (0.5-6 Hz) from motor/belt issues
    // Flutter: fast speed variations (30-200 Hz) from head/tape interaction
    float wow_freq = 2.0f;  // Hz
    float flutter_freq = 80.0f;  // Hz
    float wow_depth = 0.003f * (1.0f + (1.0f - tape_speed) * 2.0f);
    float flutter_depth = 0.001f * (1.0f + (1.0f - tape_speed) * 3.0f);

    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;

    for (int i = 0; i < num_samples; ++i) {
        float wow = std::sin(linear_wow_phase_) * wow_depth;
        float flutter = std::sin(linear_flutter_phase_) * flutter_depth;

        // Speed variation causes pitch shift (resampling effect)
        float speed_var = 1.0f + wow + flutter;
        signal[i] *= speed_var;  // Amplitude modulation from speed variation

        linear_wow_phase_ += wow_freq * two_pi_dt;
        linear_flutter_phase_ += flutter_freq * two_pi_dt;

        if (linear_wow_phase_ > 2.0f * float(M_PI)) linear_wow_phase_ -= 2.0f * float(M_PI);
        if (linear_flutter_phase_ > 2.0f * float(M_PI)) linear_flutter_phase_ -= 2.0f * float(M_PI);
    }
}

void VCRAudioProcessor::processHiFi(const float* input_l, const float* input_r,
                                     float* output_l, float* output_r,
                                     int num_samples, float tape_speed, float snr_db) {
    // FM encode
    std::vector<float> fm_signal(num_samples * 2);
    hifiFmEncode(input_l, input_r, fm_signal.data(), num_samples);

    // Apply tape degradation
    applyTapeNoise(fm_signal.data(), num_samples * 2, snr_db);

    // FM decode
    hifiFmDecode(fm_signal.data(), output_l, output_r, num_samples, snr_db);

    // Apply wow/flutter (less pronounced for Hi-Fi due to deep recording)
    applyWowFlutter(output_l, num_samples, tape_speed * 0.5f);
    applyWowFlutter(output_r, num_samples, tape_speed * 0.5f);
}

void VCRAudioProcessor::processLinear(const float* input, float* output,
                                       int num_samples, float tape_speed, float snr_db) {
    // Linear audio: simple bandpass + noise + wow/flutter
    float two_pi_dt = 2.0f * float(M_PI) / kSAMPLE_RATE;

    // Simple first-order bandpass (50 Hz - 10 kHz)
    float hp_cutoff = 50.0f * two_pi_dt;
    float lp_cutoff = 10000.0f * two_pi_dt;
    float hp_state = 0.0f;
    float lp_state = 0.0f;

    for (int i = 0; i < num_samples; ++i) {
        // High-pass filter (remove DC and very low frequencies)
        hp_state += hp_cutoff * (input[i] - hp_state);
        float filtered = input[i] - hp_state;

        // Low-pass filter (limit bandwidth)
        lp_state += lp_cutoff * (filtered - lp_state);
        output[i] = lp_state;

        // Apply tape noise (linear track has more hiss)
        float noise_amp = std::pow(10.0f, -snr_db / 20.0f) * 1.5f;  // More noise than Hi-Fi
        output[i] += (float(rand()) / float(RAND_MAX) - 0.5f) * noise_amp * 2.0f;
    }

    // Apply wow/flutter (more pronounced for linear track)
    applyWowFlutter(output, num_samples, tape_speed);
}

float VCRAudioProcessor::generateControlPulse(float tape_time, float tape_speed) {
    // CTL pulses at 30 Hz for NTSC (one pulse per field)
    // Pulse width is approximately 150 μs
    float pulse_freq = kCTL_FREQ * (tape_speed > 0.0f ? tape_speed : 1.0f);
    float two_pi_dt = 2.0f * float(M_PI) * pulse_freq;

    ctl_phase_ += two_pi_dt / kSAMPLE_RATE;
    if (ctl_phase_ > 2.0f * float(M_PI)) ctl_phase_ -= 2.0f * float(M_PI);

    // Generate pulse: active for first 150 μs of each cycle
    float cycle_time = ctl_phase_ / (2.0f * float(M_PI)) / pulse_freq;
    float pulse_width = 150e-6f;  // 150 μs

    return (cycle_time < pulse_width) ? 1.0f : 0.0f;
}
