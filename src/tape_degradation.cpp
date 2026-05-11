#include "tape_degradation.h"

TapeDegradationProcessor::TapeDegradationProcessor() {
    // Initialize with empty state
}

void TapeDegradationProcessor::updateTapeState(TapeState& state, float playback_hours) {
    state.total_playback_hours += playback_hours;
    state.play_count++;

    // Oxide shedding increases with play count
    // Typical VHS tape: noticeable dropouts after 50-100 plays
    state.oxide_shedding = std::min(1.0f, state.play_count / 100.0f * 0.3f
                                    + state.total_playback_hours / 10.0f * 0.2f);

    // Binder degradation accelerates with humidity and temperature
    state.binder_degradation = std::min(1.0f,
        state.total_playback_hours / 50.0f * 0.1f
        + state.humidity_damage * 0.3f
        + state.temperature_damage * 0.2f);

    // Magnetic decay: slow loss of signal strength over years
    state.magnetic_decay = std::min(1.0f, state.total_playback_hours / 100.0f * 0.15f);

    // Edge wear: increases with play count and poor tracking
    state.edge_wear = std::min(1.0f, state.play_count / 200.0f * 0.2f
                               + state.total_playback_hours / 20.0f * 0.1f);

    // Crease damage: usually from mishandling, not playback
    // (Set externally based on tape condition)
}

void TapeDegradationProcessor::applyDropouts(float* signal, int num_samples, int line_num,
                                              const TapeState& state, uint32_t& rng_state) {
    if (state.oxide_shedding < 0.01f) return;

    // Dropout probability based on oxide shedding
    float dropout_prob = state.oxide_shedding * 0.02f;  // Up to 2% chance per line

    // Simple LCG for deterministic dropouts per line
    uint32_t line_seed = rng_state + uint32_t(line_num) * 65537u;
    line_seed ^= line_seed << 13; line_seed ^= line_seed >> 17; line_seed ^= line_seed << 5;

    if (float(line_seed & 0xFFFF) / float(0xFFFF) < dropout_prob) {
        // Generate dropout
        int dropout_x = line_seed % num_samples;
        int dropout_duration = 1 + (line_seed >> 16) % 20;  // 1-20 samples
        float dropout_severity = 0.3f + float((line_seed >> 8) & 0xFF) / 255.0f * 0.7f;

        for (int i = 0; i < dropout_duration && (dropout_x + i) < num_samples; ++i) {
            // Dropout: signal drops to noise or zero
            float noise = float((line_seed >> 4) & 0xFF) / 255.0f - 0.5f;
            signal[dropout_x + i] = signal[dropout_x + i] * (1.0f - dropout_severity)
                                   + noise * dropout_severity * 0.5f;
        }
    }
}

void TapeDegradationProcessor::generateCreasePositions(const TapeState& state, uint32_t& rng_state) {
    if (state.crease_damage < 0.01f) {
        crease_lines_.clear();
        return;
    }

    // Generate 1-5 crease lines based on damage level
    int num_creases = 1 + int(state.crease_damage * 4.0f);
    crease_lines_.resize(num_creases);
    crease_intensity_ = state.crease_damage;

    for (int i = 0; i < num_creases; ++i) {
        // Random line position, biased toward middle (where tape is most stressed)
        rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
        float t = float(rng_state & 0xFFFF) / float(0xFFFF);
        // Bias toward center: use quadratic distribution
        t = t * t * (3.0f - 2.0f * t);  // Smoothstep
        crease_lines_[i] = int(t * 480);  // NTSC height
    }
}

void TapeDegradationProcessor::applyCreaseDamage(float* signal, int width, int height,
                                                  const TapeState& state, uint32_t& rng_state) {
    if (state.crease_damage < 0.01f) return;

    generateCreasePositions(state, rng_state);

    // Apply crease damage to affected lines
    for (int crease_y : crease_lines_) {
        if (crease_y < 0 || crease_y >= height) continue;

        int line_start = crease_y * width;
        float crease_width = 2.0f + crease_intensity_ * 8.0f;  // 2-10 pixels wide

        for (int x = 0; x < width; ++x) {
            float dist = float(x) / float(width);
            // Crease effect: bright/dark band with edge enhancement
            float crease_effect = std::exp(-0.5f * (dist - 0.5f) * (dist - 0.5f) / (0.1f * 0.1f));
            crease_effect *= crease_intensity_;

            // Add noise and distortion at crease location
            float noise = float(rng_state & 0xFF) / 255.0f - 0.5f;
            signal[line_start + x] += noise * crease_effect * 0.5f;
            signal[line_start + x] *= (1.0f + crease_effect * 0.3f);  // Gain variation

            rng_state ^= rng_state >> 8;
        }
    }
}

void TapeDegradationProcessor::applyOxygenation(float* luma, float* chroma_i, float* chroma_q,
                                                 int num_samples, const TapeState& state) {
    if (state.magnetic_decay < 0.01f) return;

    // HF loss: older tapes lose high frequencies first
    // Apply low-pass filter with cutoff decreasing with decay
    float hf_cutoff = 5.0e6f * (1.0f - state.magnetic_decay * 0.5f);  // 5 MHz → 2.5 MHz
    float alpha = hf_cutoff / (hf_cutoff + kSAMPLE_RATE / (2.0f * float(M_PI)));

    // Apply to luma
    for (int i = 0; i < num_samples; ++i) {
        oxygenation_lpf_state_[0] += alpha * (luma[i] - oxygenation_lpf_state_[0]);
        luma[i] = oxygenation_lpf_state_[0];
    }

    // Color fading: chroma amplitude decreases with age
    float chroma_fade = 1.0f - state.magnetic_decay * 0.3f;
    for (int i = 0; i < num_samples; ++i) {
        chroma_i[i] *= chroma_fade;
        chroma_q[i] *= chroma_fade;
    }

    // Increased noise floor from oxide degradation
    float noise_floor = state.magnetic_decay * 0.05f;
    for (int i = 0; i < num_samples; ++i) {
        luma[i] += noise_floor * (float(rand()) / float(RAND_MAX) - 0.5f);
    }
}

void TapeDegradationProcessor::applyEdgeDamage(float* signal, int width, int height,
                                                const TapeState& state, uint32_t& rng_state) {
    if (state.edge_wear < 0.01f) return;

    // Edge damage affects outer 10-30% of each scanline
    float edge_width = 0.1f + state.edge_wear * 0.2f;  // 10-30% from each edge

    for (int y = 0; y < height; ++y) {
        int line_start = y * width;
        int edge_pixels = int(width * edge_width);

        for (int x = 0; x < edge_pixels; ++x) {
            // Left edge damage
            float left_damage = (1.0f - float(x) / float(edge_pixels)) * state.edge_wear;
            float noise_l = float(rng_state & 0xFF) / 255.0f - 0.5f;
            signal[line_start + x] = signal[line_start + x] * (1.0f - left_damage)
                                    + noise_l * left_damage * 0.3f;

            // Right edge damage
            int rx = width - 1 - x;
            float right_damage = (1.0f - float(x) / float(edge_pixels)) * state.edge_wear;
            float noise_r = float((rng_state >> 8) & 0xFF) / 255.0f - 0.5f;
            signal[line_start + rx] = signal[line_start + rx] * (1.0f - right_damage)
                                     + noise_r * right_damage * 0.3f;

            rng_state ^= rng_state << 5;
        }
    }
}

void TapeDegradationProcessor::applyStickyShed(float* signal, int width, int height,
                                                const TapeState& state, uint32_t& rng_state,
                                                float& tracking_error_out) {
    if (state.binder_degradation < 0.01f) return;

    // Sticky shed syndrome causes:
    // 1. Head clogging → dropouts and tracking errors
    // 2. Increased friction → speed variations
    // 3. Oxide transfer → signal contamination

    // Tracking error increases with binder degradation
    tracking_error_out += state.binder_degradation * 0.3f;
    tracking_error_out = std::min(tracking_error_out, 1.0f);

    // Apply horizontal streaks from head clogging
    float streak_prob = state.binder_degradation * 0.1f;
    for (int y = 0; y < height; ++y) {
        rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;
        if (float(rng_state & 0xFFFF) / float(0xFFFF) < streak_prob) {
            int line_start = y * width;
            float streak_len = 0.1f + float((rng_state >> 16) & 0xFF) / 255.0f * 0.4f;
            int streak_start = rng_state % width;
            int streak_end = std::min(width, streak_start + int(width * streak_len));

            for (int x = streak_start; x < streak_end; ++x) {
                float streak_effect = state.binder_degradation * 0.5f;
                float noise = float(rng_state & 0xFF) / 255.0f - 0.5f;
                signal[line_start + x] = signal[line_start + x] * (1.0f - streak_effect)
                                        + noise * streak_effect;
            }
        }
    }
}

std::vector<TapeDegradationProcessor::DropoutEvent>
TapeDegradationProcessor::generateDropouts(int width, int height,
                                            const TapeState& state, uint32_t rng_state) {
    std::vector<DropoutEvent> dropouts;

    if (state.oxide_shedding < 0.01f) return dropouts;

    // Number of dropouts proportional to oxide shedding
    int num_dropouts = int(state.oxide_shedding * 20.0f);

    for (int i = 0; i < num_dropouts; ++i) {
        rng_state ^= rng_state << 13; rng_state ^= rng_state >> 17; rng_state ^= rng_state << 5;

        DropoutEvent event;
        event.y = rng_state % height;
        event.x = (rng_state >> 8) % width;
        event.duration = 1 + ((rng_state >> 16) % 30);  // 1-30 samples
        event.severity = 0.2f + float((rng_state >> 24) & 0xFF) / 255.0f * 0.8f;

        dropouts.push_back(event);
    }

    return dropouts;
}
