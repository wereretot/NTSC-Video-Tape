# Requirements Document

## Introduction

The current VHS helical-scan simulation accurately models NTSC timing constants (drum RPM, FM carrier frequencies, azimuth angle, track pitch) but lacks several critical physical phenomena. This feature enhances the simulation to include:

1. **VCR brand differentiation** — distinct characteristics for Panasonic, JVC, Sony, Mitsubishi, Sharp, Toshiba
2. **Head count configuration** — 2-head (standard), 4-head (HQ/slow-mo), 6-head (S-VHS/professional)
3. **Azimuth crosstalk rejection** — actual implementation of the ±6° azimuth offset between A/B heads
4. **True FM luma encode/decode** — 3.4-4.4 MHz FM modulation with threshold effect and triangular noise
5. **Color-under process** — 3.58 MHz → 629 kHz → 3.58 MHz downconvert/upconvert with phase relationships
6. **Helical-scan geometry** — diagonal track writing, 180° drum wrap, tape helix angle
7. **Bandwidth limiting** — luma ~3 MHz, chroma ~500 kHz VHS bandwidth constraints
8. **Pre-emphasis / de-emphasis** — luma pre-emphasis during record, de-emphasis during playback
9. **1H delay line dropout compensation** — previous-line substitution for oxide dropouts
10. **S-VHS support** — 5.4-7.0 MHz FM carrier, wider bandwidth, metal tape
11. **CTL control track simulation** — servo reference pulse, drum phase lock

## Glossary

- **Helical Scan**: Recording method where tape wraps diagonally around a rotating drum, each head writing a diagonal track
- **Azimuth Recording**: ±6° head gap tilt between A/B heads, preventing adjacent-track crosstalk without guard bands
- **FM Luma Carrier**: Frequency-modulated carrier (3.4-4.4 MHz VHS, 5.4-7.0 MHz S-VHS) encoding luminance
- **Color-Under**: Chroma signal downconverted from 3.58 MHz to 629 kHz, recorded below the FM luma carrier
- **TBE (Time Base Error)**: Timing instability from tape transport mechanical imperfections
- **CTL (Control Track)**: Longitudinal servo track providing synchronization reference
- **1H Delay Line**: One horizontal line (63.5 μs) storage for dropout compensation
- **SP/LP/EP**: Standard Play (3.335 cm/s), Long Play (2.223 cm/s), Extended Play (1.112 cm/s) tape speeds

## Requirements

### REQ-1: VCR Brand Differentiation

**User Story:** AS a VHS simulation user, I WANT to select a specific VCR brand and model, SO THAT the simulation produces brand-characteristic artifacts.

#### Acceptance Criteria

1. WHEN the user selects a VCR brand from the UI, the simulator SHALL apply brand-specific constants for drum bearing design, head amplifier noise floor, CTL head sensitivity, and tape path geometry.
2. THE simulator SHALL support these brands with distinct parameter profiles: JVC (inventor, standard reference), Panasonic (quiet noise floor, strong CTL lock), Sony (unique drum bearing resonance at 30.5 Hz), Mitsubishi (longer tape path, increased scrape flutter), Sharp (weaker CTL lock, more tracking sensitivity), Toshiba (motor cogging at 6 poles).
3. WHEN the user changes the VCR brand during playback, the simulator SHALL transition the brand-specific parameters over 0.5 seconds to avoid discontinuous artifacts.

### REQ-2: Head Count Configuration

**User Story:** AS a VHS simulation user, I WANT to configure the number of drum heads (2/4/6), SO THAT the simulation reflects the capabilities of different VCR classes.

#### Acceptance Criteria

1. THE simulator SHALL support 2-head (standard playback), 4-head (HQ/slow-mo/still-frame), and 6-head (S-VHS/professional with dedicated erase head and Hi-Fi heads) configurations.
2. WHEN a 4-head VCR is selected, the simulator SHALL reduce head-switching transients by 40% and eliminate slow-motion playback artifacts (the extra heads are dedicated to special functions).
3. WHEN a 6-head VCR is selected, the simulator SHALL enable Hi-Fi FM audio simulation (depth-multiplexed 1.3 MHz / 1.7 MHz carriers) and reduce azimuth crosstalk by 25% (higher-quality heads).
4. THE head count configuration SHALL be independent of the brand selection — any brand can have any head count.

### REQ-3: Azimuth Crosstalk Rejection

**User Story:** AS a VHS simulation user, I WANT adjacent-track crosstalk to be reduced by the azimuth offset, SO THAT the simulation matches real VHS behavior where LP/EP modes rely on azimuth (not guard bands) for crosstalk rejection.

#### Acceptance Criteria

1. THE simulator SHALL apply a frequency-dependent azimuth rejection filter that attenuates crosstalk from adjacent tracks by 30 dB at the luma FM carrier frequency and 20 dB at the chroma-under frequency.
2. WHEN simulating LP or EP mode (no guard bands), the simulator SHALL model crosstalk from the adjacent track with azimuth-rejected amplitude, creating the characteristic "venetian blind" pattern.
3. THE azimuth rejection SHALL use the correct ±6° gap angle: rejection(dB) = 20 * log10(sinc(Δf * d * sin(6°) / v)) where Δf is frequency offset, d is head gap width (0.3 μm), v is tape-head relative velocity (4.86 m/s).

### REQ-4: True FM Luma Encode/Decode

**User Story:** AS a VHS simulation user, I WANT the luma signal to be genuinely FM-modulated at 3.4-4.4 MHz and demodulated, SO THAT the simulation captures FM-specific artifacts like threshold effect and triangular noise spectrum.

#### Acceptance Criteria

1. THE simulator SHALL encode luma as f(t) = f_carrier + k_deviation * Y(t) where f_carrier is 3.4 MHz (SP) or 4.4 MHz (SP peak), k_deviation = 1.0 MHz, Y(t) is normalized luma [0, 1].
2. THE simulator SHALL demodulate the FM carrier using a quadrature detector or PLL, producing the FM-specific triangular noise spectrum (noise power ∝ 1/f²).
3. THE simulator SHALL model the FM threshold effect: when signal strength drops below 12 dB SNR, the demodulated output produces impulsive "sparkle" noise characteristic of FM demodulation failure.
4. THE simulator SHALL limit luma bandwidth to 3.0 MHz (-3 dB) post-demodulation, matching VHS luma bandwidth specification.

### REQ-5: Color-Under Process

**User Story:** AS a VHS simulation user, I WANT the chroma signal to be downconverted to 629 kHz during record and upconverted during playback, SO THAT the simulation captures the correct 3:2 phase switching relationship between A and B fields.

#### Acceptance Criteria

1. THE simulator SHALL downconvert chroma during encode: I_under = I * cos(2π * 629.375 kHz * t), Q_under = Q * sin(2π * 629.375 kHz * t).
2. THE simulator SHALL apply the VHS 3:2 phase switching: the 629 kHz carrier phase advances by 90° per line, creating a 4-line (2-field) repeating pattern.
3. THE simulator SHALL upconvert chroma during decode: I = I_under * cos(2π * f_sc * t), Q = Q_under * sin(2π * f_sc * t), where f_sc = 3.579545 MHz.
4. THE simulator SHALL limit chroma bandwidth to 500 kHz (-3 dB) in the under domain, matching VHS chroma bandwidth specification.
5. THE simulator SHALL model the color-under crosstalk: adjacent tracks contain phase-shifted chroma-under signals that produce the characteristic "cross-color" moiré pattern.

### REQ-6: Helical-Scan Geometry

**User Story:** AS a VHS simulation user, I WANT the simulation to model the physical geometry of the helical-scan drum, SO THAT the TBE patterns and head-switching artifacts match real VHS behavior.

#### Acceptance Criteria

1. THE simulator SHALL model a 62 mm diameter drum with 180° tape wrap angle and 5.016° helix angle, computing the head-to-tape relative velocity as 4.86 m/s.
2. THE simulator SHALL compute the diagonal track pitch as the physical distance between adjacent track centers on the tape, using the formula: track_pitch = (tape_speed / drum_RPS) / sin(helix_angle).
3. THE simulator SHALL position the head-switching point at 7 lines into the VBI, with the transition occurring over 2-3 scanlines.
4. THE simulator SHALL model the drum eccentricity as a sinusoidal TBE component at 30 Hz (drum rotation frequency) with amplitude proportional to drum_eccentricity parameter.

### REQ-7: Bandwidth Limiting

**User Story:** AS a VHS simulation user, I WANT the luma and chroma signals to be bandwidth-limited to VHS specifications, SO THAT the simulation produces the characteristic soft-focus look of VHS video.

#### Acceptance Criteria

1. THE simulator SHALL apply a low-pass filter to the luma signal with -3 dB cutoff at 3.0 MHz and -20 dB at 4.2 MHz, using a 6-pole Butterworth response.
2. THE simulator SHALL apply a low-pass filter to the chroma signal with -3 dB cutoff at 500 kHz and -20 dB at 1.0 MHz, using a 4-pole Butterworth response.
3. THE simulator SHALL apply these bandwidth limits during the encode phase (recording), not during decode, matching the physical VHS recording process.
4. THE simulator SHALL provide a "S-VHS mode" toggle that increases luma bandwidth to 5.0 MHz and chroma bandwidth to 1.0 MHz.

### REQ-8: Pre-Emphasis / De-Emphasis

**User Story:** AS a VHS simulation user, I WANT the luma signal to receive pre-emphasis during recording and de-emphasis during playback, SO THAT the simulation captures the SNR improvement and high-frequency overshoot characteristics.

#### Acceptance Criteria

1. THE simulator SHALL apply a first-order pre-emphasis filter to the luma signal before FM encoding: H_pre(f) = 1 + j*f/f_corner where f_corner = 1.5 MHz.
2. THE simulator SHALL apply the corresponding de-emphasis filter after FM demodulation: H_de(f) = 1 / (1 + j*f/f_corner).
3. THE pre-emphasis / de-emphasis pair SHALL produce a net 6 dB SNR improvement for high-frequency luma content (above 1.5 MHz).
4. THE simulator SHALL produce characteristic high-frequency "ringing" on sharp luma transitions (white/black edges) when the pre-emphasis is applied without full de-emphasis (simulating worn electronics).

### REQ-9: 1H Delay Line Dropout Compensation

**User Story:** AS a VHS simulation user, I WANT the simulator to compensate for dropouts by substituting the previous line's data, SO THAT the simulation matches the behavior of real VCRs with dropout compensators.

#### Acceptance Criteria

1. THE simulator SHALL maintain a 1H (one horizontal line) delay buffer containing the previously decoded scanline.
2. WHEN a dropout event is detected (signal amplitude < 10% of normal for > 5 pixels), the simulator SHALL substitute the corresponding pixels from the 1H delay buffer.
3. THE simulator SHALL apply a configurable dropout compensation strength (0 = raw dropouts visible, 1 = full 1H compensation) controlled by a UI parameter.
4. THE simulator SHALL model the limitation of 1H compensation: vertical edges and fast vertical motion will show "combing" artifacts when compensation is active.

### REQ-10: S-VHS Support

**User Story:** AS a VHS simulation user, I WANT to enable S-VHS mode, SO THAT the simulation produces the higher-quality image characteristic of Super VHS.

#### Acceptance Criteria

1. WHEN S-VHS mode is enabled, the simulator SHALL use FM carrier frequencies of 5.4 MHz (sync tip) to 7.0 MHz (peak white) instead of 3.4-4.4 MHz.
2. WHEN S-VHS mode is enabled, the simulator SHALL increase luma bandwidth to 5.0 MHz and chroma bandwidth to 1.0 MHz.
3. WHEN S-VHS mode is enabled, the simulator SHALL model the metal particle tape formulation: reduced oxide dropouts (-50% rate), higher RF level (+3 dB), lower hiss (-40%).
4. WHEN S-VHS mode is enabled, the simulator SHALL require S-VHS tape media: playing a standard VHS tape in S-VHS mode SHALL produce standard VHS quality (no upconversion).

### REQ-11: CTL Control Track Simulation

**User Story:** AS a VHS simulation user, I WANT the simulator to model the CTL control track and servo system, SO THAT tracking errors and servo lock behavior match real VHS mechanics.

#### Acceptance Criteria

1. THE simulator SHALL model a longitudinal CTL track recorded at the tape edge, containing one pulse per field (59.94 Hz) at the standard CTL amplitude.
2. THE simulator SHALL compute the CTL head readout signal as a function of tape speed, tracking alignment, and tape condition (oxide shedding, sticky shed).
3. WHEN the CTL signal drops below 30% of nominal amplitude, the simulator SHALL reduce tracking_lock_ and increase drum phase error, producing the characteristic "tracking bar" artifact.
4. THE simulator SHALL model the CTL servo response: a first-order low-pass with 8-line time constant, pulling the drum phase toward the CTL reference.

### REQ-12: VHS Audio Simulation (Mono Linear + Hi-Fi FM)

**User Story:** AS a VHS simulation user, I WANT the simulator to produce authentic VHS audio output (mono linear AFM track and/or Hi-Fi FM audio), SO THAT the audio degradation matches the video degradation characteristics.

#### Acceptance Criteria

1. THE simulator SHALL generate a mono linear AFM (Audio Frequency Modulation) audio track at the tape edge, using FM modulation at 1.3 kHz deviation with 50 Hz - 10 kHz bandwidth, independent of any external audio library.
2. THE simulator SHALL generate Hi-Fi FM audio using depth-multiplexed carriers at 1.3 MHz (left) and 1.7 MHz (right) recorded underneath the video signal, with 150 kHz deviation and 20 Hz - 20 kHz bandwidth, only when a 6-head VCR is selected.
3. WHEN tape speed changes, the audio pitch and wow/flutter SHALL scale proportionally with the tape speed, matching the mechanical coupling between tape transport and audio track.
4. WHEN tape degradation increases, the linear AFM audio SHALL exhibit increased hiss, wow, flutter, and dropout before the Hi-Fi FM audio degrades (Hi-Fi is more robust).
5. THE simulator SHALL produce characteristic VHS audio artifacts: tape-speed-dependent pitch wobble (wow/flutter), high-frequency rolloff (linear track limited to ~10 kHz), intermittent dropout clicks, and Hi-Fi crosstalk from the video FM carrier.
6. THE audio output SHALL be available as a separate stream that the pipeline can mux with the video output, replacing the current CapstanVar-based audio processing.
