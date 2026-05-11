#pragma once

#include <vector>
#include <cstdint>
#include <random>
#include <cmath>
#include <algorithm>

// ── Tape Degradation Effects ──────────────────────────────────────
// Models physical tape deterioration over time and use:
//   1. Dropouts: oxide shedding, dust particles (microsecond signal loss)
//   2. Tape Crease: fold damage, permanent deformation (line-wide artifacts)
//   3. Oxygenation: chemical degradation of magnetic coating (HF loss)
//   4. Edge Damage: tape edge wear from repeated use
//   5. Sticky Shed Syndrome: binder hydrolysis (sticky tape, head clogging)

struct TapeState {
    // Age-related degradation (0 = new, 1 = severely degraded)
    float oxide_shedding = 0.0f;       // Dropout frequency increases
    float binder_degradation = 0.0f;   // Sticky shed syndrome
    float magnetic_decay = 0.0f;       // Signal loss from oxide aging
    float edge_wear = 0.0f;            // Tape edge damage
    float crease_damage = 0.0f;        // Fold/crease marks
    float stretch_damage = 0.0f;       // Permanent tape stretching

    // Usage-related wear
    int play_count = 0;                // Number of playbacks
    float total_playback_hours = 0.0f; // Cumulative playback time

    // Environmental factors
    float humidity_damage = 0.0f;      // High humidity accelerates degradation
    float temperature_damage = 0.0f;   // Heat damage to binder
};

class TapeDegradationProcessor {
public:
    TapeDegradationProcessor();

    // Apply dropout artifacts to video signal
    // Dropouts are microsecond losses of signal from oxide shedding or dust
    void applyDropouts(float* signal, int num_samples, int line_num,
                       const TapeState& state, uint32_t& rng_state);

    // Apply tape crease damage (fold marks)
    // Creates horizontal bands of distortion across the frame
    void applyCreaseDamage(float* signal, int width, int height,
                           const TapeState& state, uint32_t& rng_state);

    // Apply oxygenation effects (chemical degradation)
    // High-frequency loss, increased noise floor, color fading
    void applyOxygenation(float* luma, float* chroma_i, float* chroma_q,
                          int num_samples, const TapeState& state);

    // Apply edge damage (tape edge wear)
    // Affects the outer portions of each scanline
    void applyEdgeDamage(float* signal, int width, int height,
                         const TapeState& state, uint32_t& rng_state);

    // Apply sticky shed syndrome effects
    // Head clogging, increased friction, tracking errors
    void applyStickyShed(float* signal, int width, int height,
                         const TapeState& state, uint32_t& rng_state,
                         float& tracking_error_out);

    // Update tape state based on usage
    void updateTapeState(TapeState& state, float playback_hours);

    // Generate random dropout positions for a frame
    struct DropoutEvent {
        int x;           // Position in scanline
        int y;           // Scanline number
        float duration;  // Duration in samples
        float severity;  // 0 = mild, 1 = complete signal loss
    };

    std::vector<DropoutEvent> generateDropouts(int width, int height,
                                                const TapeState& state,
                                                uint32_t rng_state);

private:
    static constexpr float kSAMPLE_RATE = 14.31818e6f;  // NTSC sample rate

    // Precomputed crease positions (persistent across frames)
    std::vector<int> crease_lines_;
    float crease_intensity_ = 0.0f;

    // Precomputed dropout positions (regenerate per frame)
    std::vector<DropoutEvent> current_dropouts_;

    // HF loss filter state for oxygenation
    float oxygenation_lpf_state_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    void generateCreasePositions(const TapeState& state, uint32_t& rng_state);
};
