#pragma once

#include <cstdint>

// ── VCR Brand Profiles ──────────────────────────────────────────────
// Each brand has distinct mechanical and electronic characteristics
// that affect the simulation output.

enum class VCRBrand {
    JVC,            // HR-S3600 — VHS inventor, reference standard
    Panasonic,      // AG-1980 — Quiet noise floor, strong CTL lock
    Sony,           // SLV-1000 — Unique drum bearing resonance at 30.5 Hz
    Mitsubishi,     // HS-HD2000U — Longer tape path, more scrape flutter
    Sharp,          // VC-A588U — Weaker CTL lock, more tracking sensitivity
    Toshiba,        // M-462 — Motor cogging at 6 poles
    COUNT
};

enum class HeadCount : int {
    TwoHead  = 2,   // Standard VHS playback
    FourHead = 4,   // HQ / slow-mo / still-frame heads
    SixHead  = 6,   // S-VHS / professional with Hi-Fi + eraser heads
};

enum class TapeFormat {
    VHS,            // Standard VHS (3.4-4.4 MHz FM, 3.0 MHz luma BW)
    SVHS,           // Super VHS (5.4-7.0 MHz FM, 5.0 MHz luma BW)
};

struct VCRBrandProfile {
    const char* name;

    // Drum mechanics
    float drum_bearing_resonance_hz;   // Primary resonant frequency (Hz)
    float drum_bearing_q;              // Q factor of bearing resonance
    float drum_eccentricity_base;      // Baseline drum wobble (μm)
    int   motor_pole_count;            // DC motor poles

    // CTL system
    float ctl_head_sensitivity;        // Control head readout efficiency (0-1)
    float ctl_servo_bandwidth_hz;      // Servo loop bandwidth (Hz)
    float ctl_pulse_width_us;          // CTL pulse width variation (μs)

    // Head amplifier
    float head_amp_noise_floor_db;     // Noise floor relative to signal (dB)
    float head_amp_bandwidth_mhz;      // Head amp -3 dB point (MHz)

    // Tape path
    float tape_path_length_mm;         // Total tape path length (mm)
    float scrape_flutter_coeff;        // Scrape flutter multiplier
    float tape_tension_variation;      // Tape tension stability (0-1)

    // Head switch
    float head_switch_jitter_lines;    // Head switch position jitter (lines)
    float head_switch_transient_amp;   // Switch transient magnitude
};

struct HeadConfig {
    HeadCount count;
    float azimuth_a_deg;               // +5.967° for standard A head
    float azimuth_b_deg;               // -5.967° for standard B head
    float head_gap_width_um;           // Video head gap width (~0.3 μm)
    float head_gap_depth_um;           // Video head gap depth (~0.5 μm)
    float head_switch_lines;           // Lines into VBI where switch occurs
    bool  has_hifi_heads;              // Dedicated Hi-Fi heads (6-head only)
    bool  has_eraser_head;             // Full-track eraser head (6-head only)
    bool  has_slowmo_heads;            // Dedicated still/slow-mo heads (4/6-head)
    float crosstalk_level;             // Adjacent-track crosstalk amplitude (0-1)
    float azimuth_reduction;           // Azimuth rejection effectiveness (0-1)
};

// ── Brand Profile Database ──────────────────────────────────────────

inline const VCRBrandProfile& getBrandProfile(VCRBrand brand) {
    static const VCRBrandProfile profiles[] = {
        // JVC HR-S3600 — VHS inventor, reference standard
        {
            .name = "JVC HR-S3600",
            .drum_bearing_resonance_hz = 30.0f,
            .drum_bearing_q = 3.5f,
            .drum_eccentricity_base = 2.0f,
            .motor_pole_count = 8,
            .ctl_head_sensitivity = 0.85f,
            .ctl_servo_bandwidth_hz = 7.0f,
            .ctl_pulse_width_us = 150.0f,
            .head_amp_noise_floor_db = -48.0f,
            .head_amp_bandwidth_mhz = 5.5f,
            .tape_path_length_mm = 420.0f,
            .scrape_flutter_coeff = 1.0f,
            .tape_tension_variation = 0.15f,
            .head_switch_jitter_lines = 0.3f,
            .head_switch_transient_amp = 0.12f,
        },
        // Panasonic AG-1980 — Quiet noise floor, strong CTL lock
        {
            .name = "Panasonic AG-1980",
            .drum_bearing_resonance_hz = 30.0f,
            .drum_bearing_q = 4.0f,
            .drum_eccentricity_base = 1.5f,
            .motor_pole_count = 8,
            .ctl_head_sensitivity = 0.95f,
            .ctl_servo_bandwidth_hz = 8.0f,
            .ctl_pulse_width_us = 145.0f,
            .head_amp_noise_floor_db = -52.0f,
            .head_amp_bandwidth_mhz = 6.0f,
            .tape_path_length_mm = 400.0f,
            .scrape_flutter_coeff = 0.9f,
            .tape_tension_variation = 0.10f,
            .head_switch_jitter_lines = 0.2f,
            .head_switch_transient_amp = 0.08f,
        },
        // Sony SLV-1000 — Unique drum bearing resonance at 30.5 Hz
        {
            .name = "Sony SLV-1000",
            .drum_bearing_resonance_hz = 30.5f,
            .drum_bearing_q = 5.0f,
            .drum_eccentricity_base = 1.8f,
            .motor_pole_count = 8,
            .ctl_head_sensitivity = 0.80f,
            .ctl_servo_bandwidth_hz = 6.5f,
            .ctl_pulse_width_us = 155.0f,
            .head_amp_noise_floor_db = -50.0f,
            .head_amp_bandwidth_mhz = 5.8f,
            .tape_path_length_mm = 410.0f,
            .scrape_flutter_coeff = 1.0f,
            .tape_tension_variation = 0.12f,
            .head_switch_jitter_lines = 0.4f,
            .head_switch_transient_amp = 0.15f,
        },
        // Mitsubishi HS-HD2000U — Longer tape path, more scrape flutter
        {
            .name = "Mitsubishi HS-HD2000U",
            .drum_bearing_resonance_hz = 30.0f,
            .drum_bearing_q = 3.0f,
            .drum_eccentricity_base = 2.5f,
            .motor_pole_count = 8,
            .ctl_head_sensitivity = 0.88f,
            .ctl_servo_bandwidth_hz = 6.0f,
            .ctl_pulse_width_us = 152.0f,
            .head_amp_noise_floor_db = -47.0f,
            .head_amp_bandwidth_mhz = 5.2f,
            .tape_path_length_mm = 480.0f,
            .scrape_flutter_coeff = 1.2f,
            .tape_tension_variation = 0.20f,
            .head_switch_jitter_lines = 0.35f,
            .head_switch_transient_amp = 0.14f,
        },
        // Sharp VC-A588U — Weaker CTL lock, more tracking sensitivity
        {
            .name = "Sharp VC-A588U",
            .drum_bearing_resonance_hz = 30.0f,
            .drum_bearing_q = 2.5f,
            .drum_eccentricity_base = 2.8f,
            .motor_pole_count = 8,
            .ctl_head_sensitivity = 0.70f,
            .ctl_servo_bandwidth_hz = 5.0f,
            .ctl_pulse_width_us = 160.0f,
            .head_amp_noise_floor_db = -46.0f,
            .head_amp_bandwidth_mhz = 5.0f,
            .tape_path_length_mm = 440.0f,
            .scrape_flutter_coeff = 1.1f,
            .tape_tension_variation = 0.25f,
            .head_switch_jitter_lines = 0.5f,
            .head_switch_transient_amp = 0.18f,
        },
        // Toshiba M-462 — Motor cogging at 6 poles
        {
            .name = "Toshiba M-462",
            .drum_bearing_resonance_hz = 30.0f,
            .drum_bearing_q = 3.2f,
            .drum_eccentricity_base = 2.2f,
            .motor_pole_count = 6,
            .ctl_head_sensitivity = 0.82f,
            .ctl_servo_bandwidth_hz = 6.5f,
            .ctl_pulse_width_us = 148.0f,
            .head_amp_noise_floor_db = -48.0f,
            .head_amp_bandwidth_mhz = 5.3f,
            .tape_path_length_mm = 430.0f,
            .scrape_flutter_coeff = 1.0f,
            .tape_tension_variation = 0.18f,
            .head_switch_jitter_lines = 0.4f,
            .head_switch_transient_amp = 0.13f,
        },
    };

    static_assert(sizeof(profiles) / sizeof(profiles[0]) == (int)VCRBrand::COUNT,
                  "Profile count must match VCRBrand::COUNT");

    int idx = static_cast<int>(brand);
    if (idx < 0 || idx >= (int)VCRBrand::COUNT) {
        idx = 0; // Default to JVC
    }
    return profiles[idx];
}

inline const char* getBrandName(VCRBrand brand) {
    return getBrandProfile(brand).name;
}

inline HeadConfig getHeadConfig(HeadCount count) {
    switch (count) {
        case HeadCount::TwoHead:
            return {
                .count = HeadCount::TwoHead,
                .azimuth_a_deg = 5.967f,
                .azimuth_b_deg = -5.967f,
                .head_gap_width_um = 0.3f,
                .head_gap_depth_um = 0.5f,
                .head_switch_lines = 7.0f,
                .has_hifi_heads = false,
                .has_eraser_head = false,
                .has_slowmo_heads = false,
                .crosstalk_level = 0.15f,
                .azimuth_reduction = 0.7f,
            };
        case HeadCount::FourHead:
            return {
                .count = HeadCount::FourHead,
                .azimuth_a_deg = 5.967f,
                .azimuth_b_deg = -5.967f,
                .head_gap_width_um = 0.3f,
                .head_gap_depth_um = 0.5f,
                .head_switch_lines = 7.0f,
                .has_hifi_heads = false,
                .has_eraser_head = false,
                .has_slowmo_heads = true,
                .crosstalk_level = 0.10f,
                .azimuth_reduction = 0.8f,
            };
        case HeadCount::SixHead:
            return {
                .count = HeadCount::SixHead,
                .azimuth_a_deg = 5.967f,
                .azimuth_b_deg = -5.967f,
                .head_gap_width_um = 0.25f,  // Higher quality heads
                .head_gap_depth_um = 0.5f,
                .head_switch_lines = 7.0f,
                .has_hifi_heads = true,
                .has_eraser_head = true,
                .has_slowmo_heads = true,
                .crosstalk_level = 0.05f,
                .azimuth_reduction = 0.9f,
            };
    }
    // Should not reach here
    return getHeadConfig(HeadCount::TwoHead);
}

inline const char* getHeadCountName(HeadCount count) {
    switch (count) {
        case HeadCount::TwoHead:  return "2-Head (Standard)";
        case HeadCount::FourHead: return "4-Head (HQ/Slow-Mo)";
        case HeadCount::SixHead:  return "6-Head (S-VHS/Pro)";
    }
    return "Unknown";
}

inline const char* getTapeFormatName(TapeFormat fmt) {
    switch (fmt) {
        case TapeFormat::VHS:  return "VHS";
        case TapeFormat::SVHS: return "S-VHS";
    }
    return "Unknown";
}

// ── FM Carrier Frequencies ──────────────────────────────────────────

inline float getFMSyncTipFreq(TapeFormat fmt) {
    return (fmt == TapeFormat::SVHS) ? 5.4f : 3.4f;  // MHz
}

inline float getFMPeakWhiteFreq(TapeFormat fmt) {
    return (fmt == TapeFormat::SVHS) ? 7.0f : 4.4f;  // MHz
}

inline float getFMDeviation(TapeFormat fmt) {
    return (fmt == TapeFormat::SVHS) ? 1.6f : 1.0f;  // MHz
}

inline float getLumaBandwidth(TapeFormat fmt) {
    return (fmt == TapeFormat::SVHS) ? 5.0f : 3.0f;  // MHz (-3 dB)
}

inline float getChromaBandwidth(TapeFormat fmt) {
    return (fmt == TapeFormat::SVHS) ? 1.0f : 0.5f;  // MHz (-3 dB)
}
