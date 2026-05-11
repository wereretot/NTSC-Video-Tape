# Implementation Task List

## Phase 1: Core Infrastructure

### Task 1.1: VCR Brand Profile System
- [x] Create `VCRBrandProfile` struct in new header `src/vcr_brand.h`
- [x] Define brand profiles for JVC, Panasonic, Sony, Mitsubishi, Sharp, Toshiba
- [x] Integrate brand profile into `NTSCSimulator` class
- [x] Add brand selection UI in draw_ui.cpp
- [x] Apply brand-specific modifiers to existing wow/flutter/drum/TBE calculations
- [x] Test: verify each brand produces distinct TBE pattern under identical input

### Task 1.2: Head Count Configuration
- [ ] Add `HeadConfig` struct and `HeadCount` enum to `ntsc_simulator.h`
- [ ] Extend `HeadState` array from [2] to [6] maximum
- [ ] Implement head-switching logic for 2-head, 4-head, 6-head modes
- [ ] Add head count UI selector
- [ ] 4-head mode: reduce head-switching transients by 40%
- [ ] 6-head mode: enable Hi-Fi audio flag, reduce azimuth crosstalk 25%
- [ ] Test: verify head count affects expected artifacts

### Task 1.3: Tape Format (VHS / S-VHS)
- [ ] Add `TapeFormat` enum to `ntsc_simulator.h`
- [ ] S-VHS mode: FM carrier 5.4-7.0 MHz instead of 3.4-4.4 MHz
- [ ] S-VHS mode: luma bandwidth 5.0 MHz, chroma bandwidth 1.0 MHz
- [ ] S-VHS mode: metal tape = -50% dropouts, +3 dB RF, -40% hiss
- [ ] Add S-VHS toggle UI
- [ ] Test: verify S-VHS produces wider bandwidth output

## Phase 2: Signal Chain - FM Luma

### Task 2.1: FM Luma Encoder
- [ ] Create `FMLumaProcessor` class in `src/fm_luma.h` / `src/fm_luma.cpp`
- [ ] Implement FM modulation: f(t) = f_carrier + k_deviation * Y(t)
- [ ] Support variable carrier (3.4 MHz SP, 5.4 MHz S-VHS)
- [ ] Support variable deviation (1.0 MHz VHS, 1.6 MHz S-VHS)
- [ ] Pre-compute FM lookup tables for performance
- [ ] Test: verify FM output frequency matches input luma level

### Task 2.2: FM Luma Decoder (Quadrature PLL)
- [ ] Implement quadrature FM demodulator
- [ ] Add PLL loop filter (2nd order)
- [ ] Implement FM threshold effect (sparkle noise below 12 dB SNR)
- [ ] Integrate into decode path in `ntsc_simulator.cpp`
- [ ] Test: encode→decode round-trip within 0.1% RMS error

### Task 2.3: Bandwidth Limiting
- [ ] Implement 6-pole Butterworth LPF for luma (3.0 MHz VHS, 5.0 MHz S-VHS)
- [ ] Implement 4-pole Butterworth LPF for chroma (500 kHz VHS, 1.0 MHz S-VHS)
- [ ] Apply bandwidth limits during encode phase
- [ ] Test: verify frequency response matches Butterworth spec

### Task 2.4: Pre-Emphasis / De-Emphasis
- [ ] Implement pre-emphasis filter before FM encode: H(f) = 1 + j*f/1.5MHz
- [ ] Implement de-emphasis filter after FM decode: H(f) = 1/(1 + j*f/1.5MHz)
- [ ] Verify SNR improvement for high-frequency content
- [ ] Test: verify characteristic ringing on sharp transitions

## Phase 3: Signal Chain - Color-Under

### Task 3.1: Color-Down Converter
- [ ] Create `ColorUnderProcessor` class in `src/color_under.h` / `src/color_under.cpp`
- [ ] Implement 3.58 MHz → 629 kHz downconversion
- [ ] Implement 3:2 phase switching (90° per line, 4-line cycle)
- [ ] Integrate into encode path
- [ ] Test: verify phase advances exactly 90° per line

### Task 3.2: Color-Up Converter
- [ ] Implement 629 kHz → 3.58 MHz upconversion
- [ ] Integrate into decode path
- [ ] Test: downconvert→upconvert round-trip phase accuracy

### Task 3.3: Color-Under Crosstalk
- [ ] Model adjacent-track chroma-under crosstalk
- [ ] Apply phase-inverted crosstalk from adjacent track
- [ ] Test: verify cross-color moiré pattern appears

## Phase 4: Azimuth & Helical-Scan Geometry

### Task 4.1: Azimuth Rejection Filter
- [ ] Create `AzimuthFilter` class in `src/azimuth_filter.h` / `src/azimuth_filter.cpp`
- [ ] Implement sinc-based rejection formula
- [ ] Apply to crosstalk from adjacent tracks
- [ ] LP/EP mode: model no guard bands, rely on azimuth rejection
- [ ] Test: verify ≥ 25 dB rejection at luma FM carrier

### Task 4.2: Helical-Scan Geometry
- [ ] Model 62 mm drum diameter, 180° wrap, 5.016° helix angle
- [ ] Compute head-to-tape relative velocity (4.86 m/s)
- [ ] Compute diagonal track pitch from tape speed and helix angle
- [ ] Update head-switching point to 7 lines into VBI
- [ ] Test: verify TBE patterns match real VHS geometry

## Phase 5: Dropout Compensation

### Task 5.1: 1H Delay Line
- [ ] Create `DropoutCompensator` class in `src/dropout_comp.h` / `src/dropout_comp.cpp`
- [ ] Implement 1H delay buffer for luma and chroma
- [ ] Detect dropouts (signal < 10% for > 5 pixels)
- [ ] Substitute from delay buffer when dropout detected
- [ ] Add UI slider for compensation strength (0-1)
- [ ] Test: verify substitution accuracy with known patterns

## Phase 6: CTL Control Track

### Task 6.1: CTL Track Model
- [ ] Create `CTLTrackModel` class in `src/ctl_track.h` / `src/ctl_track.cpp`
- [ ] Generate CTL pulse (59.94 Hz, 150 μs width)
- [ ] Compute CTL head readout from tape condition
- [ ] Implement CTL servo response (first-order LPF, 8-line time constant)
- [ ] Integrate with existing tracking_lock_ system
- [ ] Test: verify tracking_lock_ converges within 8 lines

## Phase 7: VHS Audio Simulation

### Task 7.1: Mono Linear AFM Audio
- [ ] Create `VHSAudioSimulator` class in `src/vhs_audio.h` / `src/vhs_audio.cpp`
- [ ] Implement linear AFM FM encode/decode (1.3 kHz carrier, ±400 Hz deviation)
- [ ] Implement wow/flutter on audio (shared mechanical oscillators with video)
- [ ] Implement tape hiss (variable by tape condition)
- [ ] Implement dropout clicks
- [ ] Test: verify audio pitch scales with tape speed

### Task 7.2: Hi-Fi FM Audio
- [ ] Implement Hi-Fi FM encode/decode (1.3 MHz left, 1.7 MHz right)
- [ ] Implement depth-multiplexed recording under video signal
- [ ] Implement video carrier crosstalk into Hi-Fi band
- [ ] Enable only for 6-head VCR configuration
- [ ] Test: verify Hi-Fi audio quality > linear AFM quality

### Task 7.3: Audio Pipeline Integration
- [ ] Replace CapstanVar audio in `processing_pipeline.cpp`
- [ ] Output audio as PCM stream synchronized with video frames
- [ ] Update `ProcessParams` to include audio output
- [ ] Remove CapstanVar dependency from CMakeLists.txt
- [ ] Test: verify audio/video sync within 1 frame

## Phase 8: UI Integration

### Task 8.1: VCR Brand Selector
- [ ] Add VCR brand combo box to UI
- [ ] Display brand name and key characteristics
- [ ] Apply brand profile on selection change with 0.5s transition

### Task 8.2: Head Count & Tape Format Selector
- [ ] Add head count selector (2/4/6)
- [ ] Add S-VHS toggle
- [ ] Update UI labels to reflect current configuration

### Task 8.3: Audio Controls
- [ ] Add audio format selector (Linear / Hi-Fi / Both)
- [ ] Add dropout compensation strength slider
- [ ] Display audio signal strength meters (L/R)

## Phase 9: Build & Cleanup

### Task 9.1: Remove CapstanVar Dependency
- [ ] Remove CapstanVar from CMakeLists.txt
- [ ] Remove `#include "engine.hpp"` and `#include "audio_io.hpp"` references
- [ ] Remove `TapeEngine` and `AudioIO` from `app_state.h`
- [ ] Update `processing_pipeline.cpp` to not use CapstanVar
- [ ] Test: project builds without CapstanVar

### Task 9.2: Performance Optimization
- [ ] Profile FM encode/decode hot path
- [ ] Optimize Butterworth filter implementation (SIMD if possible)
- [ ] Pre-compute FM lookup tables
- [ ] Verify frame rate remains stable at 30 fps
