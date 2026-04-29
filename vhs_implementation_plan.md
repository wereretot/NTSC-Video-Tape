# VHS VCR Simulator — Implementation Plan

## Overview

Real VHS records **FM-modulated luma** and **color-under chroma** (heterodyne-shifted to ~629kHz) as separate signals on helical tape tracks. The full simulation covers the complete signal path from tape to phosphor — the VCR's internal signal chain, the analog connection type to the TV, the TV's own front-end processing, and finally the CRT display itself.

```
Tape → VCR Head/FM Decode → Output Stage (RF / Composite / S-Video)
     → Cable & Connector → TV Input Stage → Comb Filter / Decoder
     → CRT Drive Circuits → Electron Beam → Phosphor Screen → Eye
```

Each stage in this chain introduces its own characteristic degradation. The connection type chosen (RF coax, composite RCA, or S-Video) is one of the largest single determinants of output quality and should be a first-class selectable parameter in the UI.

---

## 1. VHS Signal Chain Architecture

### 1a. Luma FM Encode/Decode

- Clamp luma to legal broadcast range (7.5 IRE–100 IRE) before "recording"
- Apply **FM pre-emphasis** — boost highs before encoding, roll them off after — this is the single biggest contributor to the characteristic VHS soft look
- SP: carrier ~4.4MHz, deviation ±1.0MHz → effective bandwidth ~3MHz; LP: carrier ~3.4MHz, narrower BW
- Implement as a simple IIR low-pass on the Y channel: `cutoff = lerp(3.0MHz_equiv, 2.4MHz_equiv, lp_factor)`
- **FM limiting**: clip decoded luma gently around 110 IRE — creates "blooming" on bright objects
- **FM noise floor**: add white noise proportional to `vp.luma_noise` before the decode low-pass — produces the characteristic luma grain texture

### 1b. Chroma Color-Under (Heterodyne) System

This is the source of most chroma artifacts.

- Downconvert chroma subcarrier from 3.58MHz to ~629kHz before "recording"
- The 629kHz carrier has extremely limited bandwidth (~500kHz) — this is why VHS chroma bleeds horizontally
- Apply a **wider low-pass to chroma** than luma: approximately 500kHz equivalent in pixel terms (`~FW * 500/4400` pixels wide)
- **Phase accumulation errors**: the heterodyne process introduces field-to-field and line-to-line chroma phase errors — `field_chroma_phase_accum_` and `inter_field_phase_error` are the right hooks, but they need to modulate actual U/V values
- On playback upconvert, inject `vp.chroma_noise` separately into the U and V channels

### 1c. YIQ ↔ BGR Conversion

Operate in float YUV internally for all signal chain processing:

```cpp
// BGR → YUV
float Y =  0.299*R + 0.587*G + 0.114*B;
float U = -0.147*R - 0.289*G + 0.436*B;
float V =  0.615*R - 0.515*G - 0.100*B;
// ... process Y and UV separately ...
// YUV → BGR
```

---

## 2. Head/Drum Mechanics

### 2a. Head Switching Artifact (HSW)

At `kVHS_HSW_LINE` (7) lines from the bottom of the frame during tracking errors:

- Apply a step-function horizontal displacement offset driven by `vp.head_switch_jitter`
- Add a brief brightness dip at the switch point
- Jitter the exact switch line ±1 based on `drum_eccentricity`

### 2b. Helical Scan Line Timing

Each video line is recorded at a slightly different angle on the tape. Model the per-line horizontal timing offset:

```cpp
float line_time_offset  = (lineY / float(H)) * vp.helical_sweep;
float h_displacement    = sin(line_time_offset + h_roll_phase_) * tracking_error_magnitude;
```

Shift each line horizontally by `h_displacement` pixels using `dsp::bsample`.

### 2c. Head Pre-Echo / Post-Echo

- Real VHS heads have slight magnetic crosstalk between adjacent tracks, visible as a faint ghost 1–2 lines above the current line
- `output_line[y] = lerp(processed[y], processed[y-2], vp.head_pre_echo * 0.15f)`

### 2d. Two-Head vs Four-Head Mode

- SP uses two heads with clean switching
- LP uses slower tape speed — increase `vp.chroma_crosstalk` and `vp.inter_field_phase_error` when `ep.ips_base <= 7.5f`

---

## 3. Tracking Error System

### 3a. CTL Pulse Tracking

Drive the existing `tracking_error_lpf_` with a slow-response filter:

```cpp
float target_tracking = vp.tracking_error + wow_contribution + tape_crease_spike;
tracking_error_lpf_ += (target_tracking - tracking_error_lpf_) * 0.05f;
```

Map `tracking_error_lpf_` to horizontal line displacement (±8 pixels at full error).

### 3b. Noise Bar

- `noise_bar_phase_` controls vertical position; the bar should be ~15–30 lines tall
- Within the bar: replace pixels with snow (high luma noise, desaturated chroma)
- Bar moves upward when tracking is slow, downward when fast

### 3c. Vertical Roll

- `v_roll_accum_` accumulates when sync is lost
- `src_y = (y + int(v_roll_accum_ * H)) % H` — apply as a vertical pixel offset with wraparound
- Add a brief horizontal tear at the wraparound line

### 3d. Tape Crease

- A crease causes a momentary, localized tracking spike modeled as a Gaussian bump in tracking error centered on a specific tape position
- When `tapeTime_` passes the crease position, add a spike to `tracking_error_lpf_` lasting ~0.5 seconds

---

## 4. Mechanical Transport — Wow & Flutter

The existing `wow_phase_[3]` and `flutter_phase_[5]` arrays are the correct structure.

### 4a. Wow (0.1–10 Hz)

```cpp
float wow_freqs[3] = {0.5f, 1.7f, 3.1f};
float wow_amps[3]  = {1.0f, 0.4f, 0.2f};
float wow = 0.f;
for (int i = 0; i < 3; ++i) {
    wow_phase_[i] += wow_freqs[i] * wallDt * 2.f * kPI;
    wow += sin(wow_phase_[i]) * wow_amps[i];
}
mechanical_wow_ = wow * ep.wow_dep;
```

### 4b. Flutter (10–200 Hz)

Five oscillators at 10–200Hz. Flutter primarily affects audio pitch but also causes subtle horizontal chroma smearing — apply to chroma phase:

```cpp
chroma_phase_offset += mechanical_flutter_ * vp.chroma_level * 0.3f;
```

### 4c. Motor Health Degradation

`vp.motor_health > 0` increases wow amplitude and adds intermittent speed lurches — 1–3 frame events where speed drops 10–20% then recovers:

```cpp
if (motor_health > 0.3f && random() < motor_health * 0.001f) trigger_lurch();
```

---

## 5. Tape Degradation Effects

### 5a. Dropout Simulation

Real dropouts are caused by oxide particles missing from the tape surface:

- A dropout replaces 1–6 pixels horizontally with the **previous line's value** (dropout concealment)
- Use `vp.dropout_rate` as a Poisson-process probability per pixel
- Use a burst model — once a dropout starts, continue it for 2–8 pixels
- `vp.dropout_intensity` controls whether the replacement is white, black, or concealed

### 5b. Oxide Shedding

- Shed oxide creates streaks — horizontal bands of elevated noise lasting 1–3 frames
- When `vp.oxide_shedding > 0`, randomly spawn "shed events" affecting a stripe of ~10–30 lines with elevated luma noise and dropout rate

### 5c. Sticky Shed Syndrome

- Causes periodic full-frame brightness drops with increased noise bar activity
- Trigger at intervals driven by `vp.sticky_shed`: every N seconds, fade brightness 30% for 2–4 frames

### 5d. Tape Age / Demagnetization

- `vp.tape_age` → reduce `base_rf_level` and increase `luma_noise`
- `vp.demagnetization` → progressively attenuate high frequencies in luma:
  ```cpp
  luma_hf_attenuation = 1.0f - vp.demagnetization * 0.7f;
  ```

---

## 6. Luma Artifacts

### 6a. Ringing / Edge Enhancement

VHS decks have peaking circuits that cause overshoot on edges:

```cpp
float edge = luma_curr - (luma_prev + luma_next) * 0.5f;
luma_out   = luma_curr + edge * vp.base_rf_level * 0.4f;
```

### 6b. Luminance Delay Line Artifacts

VHS luma has a characteristic ~100ns delay line causing slight horizontal smearing:

```cpp
Y_out[x] = Y[x] * 0.85f + Y_out[x-1] * 0.15f;
```

### 6c. Dot Crawl

The color subcarrier interferes with luma at color transitions. At horizontal edges where chroma changes, add a sinusoidal pattern using `kSC_PX` from `constants.h` for the spatial frequency.

---

## 7. Chroma Artifacts

### 7a. Chroma Horizontal Bleeding

The color-under bandwidth is ~0.5MHz vs luma's ~3MHz — chroma smears ~6x wider:

```cpp
int chroma_radius = int(FW * 0.5f / 4.4f); // ~73px for SP
// Apply box or triangle filter to U and V channels separately
```

### 7b. Chroma Phase Error (Heterodyne Drift)

`field_chroma_phase_accum_` advances each field and rotates U/V:

```cpp
field_chroma_phase_accum_ += vp.inter_field_phase_error * 0.1f;
float U_rot = U*cos(field_chroma_phase_accum_) - V*sin(field_chroma_phase_accum_);
float V_rot = U*sin(field_chroma_phase_accum_) + V*cos(field_chroma_phase_accum_);
```

### 7c. Adjacent Track Crosstalk

In standard VHS, adjacent tracks are recorded with a 90° chroma phase offset for guard-band-less recording. Mix in the adjacent track's phase-offset chroma:

```cpp
U_out += prev_track_U * vp.chroma_crosstalk * sin(adjacent_track_phase_);
```

### 7d. Chroma Noise Bar

In the noise bar region, randomize U/V values independently of Y using `vp.chroma_noise`.

---

## 8. Sync & Timing Artifacts

### 8a. Horizontal Sync Jitter

Precompute a per-line horizontal shift each frame to produce the characteristic "venetian blind" waviness:

```cpp
for (int y = 0; y < H; ++y) {
    h_jitter[y] = rand_normal() * vp.head_switch_jitter * 2.f
                + sin(y * 0.3f + h_roll_phase_) * vp.tracking_error * 3.f;
}
```

### 8b. AFC Error

`afc_error_` models FM carrier frequency drift as a gentle brightness fluctuation across the frame:

```cpp
brightness_mult = 1.0f + sin(y / float(H) * kPI + afc_error_) * 0.05f;
```

### 8c. Vertical Hold / Frame Roll

- Low `sync_hold_failure`: add a slight top-of-frame skew (the classic VHS "breathing")
- Higher values: full vertical roll via `v_roll_accum_`

---

## 9. Snow / RF Noise

### 9a. Proper Snow Generation

Snow is not random noise — it is correlated along scan lines. Extend `generateSnow()` with:

- Horizontal streaking: once a snow pixel starts, extend it 3–8 pixels
- Snow pixels should be near-white (high luma, suppressed chroma)

### 9b. RF Level → Snow Mapping

```cpp
float rf          = vp.base_rf_level * (1.f - vp.tape_age * 0.5f) * (1.f - vp.oxide_shedding * 0.3f);
float snow_prob   = std::max(0.f, (0.4f - rf) * 3.f);   // only appears below RF threshold
float snow_density = snow_prob * (1.f + vp.dropout_rate * 5.f);
```

---

## 10. Scan Line Structure

### 10a. Interlacing

Real VHS is interlaced — implement proper field separation:

- Field 1: even lines (0, 2, 4…); Field 2: odd lines (1, 3, 5…)
- Add a subtle half-line vertical offset between fields
- On motion, add horizontal feathering at moving edges when `vp.inter_field_phase_error > 0.1f` (interlace combing)

### 10b. Scanline Brightness Variation

Each line has slightly different brightness due to head-to-tape contact variation:

```cpp
float line_rf_var = 1.f + (perlin_noise(y, frameNum) * 0.03f * vp.oxide_shedding);
// Scale luma of entire line by line_rf_var
```

---

---

## 11. VCR Output Stage — Connection Types

This is the primary quality selector and should be a first-class enum in `AppState`:

```cpp
enum class VCROutputType { RF_Coax, Composite_RCA, SVideo };
```

Each type encodes the signal differently before passing it to the TV simulation stage. The TV simulation then decodes from that encoded form, accumulating artifacts specific to that path.

### 11a. RF Coax (Channel 3 / Channel 4)

The worst quality path — used when the TV has no direct video inputs.

- The VCR's RF modulator combines the composite video signal onto a carrier at ~61.25MHz (ch3) or ~67.25MHz (ch4) via AM/FM modulation
- The TV's tuner demodulates this, introducing an additional noise floor: add `~0.04f` to the effective luma noise on top of everything the VCR already contributes
- **Carrier interference**: the RF carrier beating against the subcarrier creates a faint herringbone interference pattern — implement as a low-amplitude diagonal sinusoidal grid:
  ```cpp
  float herringbone = sin((x * 0.8f + y * 1.3f + wallTime_ * 60.f) * kPI) * 0.025f * rf_interference;
  Y_out += herringbone;
  ```
- **Multipath / ghosting**: RF travels down the coax and reflects from impedance mismatches — add a faint horizontal ghost offset ~20–40 pixels to the right, attenuated to ~8–15% brightness
- **AGC hunting**: the TV's automatic gain control can't respond instantly — model as a slow (0.3Hz) brightness oscillation of ±3% amplitude
- **Fine tuning drift**: add a very slow (0.05Hz) chroma phase drift simulating the TV not being perfectly tuned — rotate U/V by up to ±15° over time

### 11b. Composite RCA (Yellow Jack)

The standard connection for most VHS setups. Y and C are combined onto a single wire as a composite NTSC signal:

- Encode: `composite = Y + C_modulated` where `C_modulated = U*sin(subcarrier_phase) + V*cos(subcarrier_phase)`
- The TV must separate Y and C — this is done imperfectly by a **comb filter** (see section 13b)
- Composite is cleaner than RF but introduces **cross-color** and **dot crawl** that RF also has but worse
- **Cable quality**: model cable capacitance as an additional mild HF rolloff applied after the VCR output stage — `cutoff_factor = lerp(1.0f, 0.85f, cable_length_param)`
- **Ground loop hum**: a cheap composite cable can introduce 60Hz hum as a horizontal brightness bar that drifts slowly upward — implement as `Y += sin(y/float(H)*kPI*2.f - wallTime_*60.f*2.f*kPI) * 0.03f * hum_level`

### 11c. S-Video (Y/C Separate)

The best practical analog connection available on consumer VHS decks.

- Y and C are carried on separate pins — the TV comb filter is bypassed entirely for luma/chroma separation
- This eliminates dot crawl and cross-color artifacts caused by composite encoding/decoding
- Still susceptible to: chroma bandwidth limits from the VHS format itself, chroma phase errors, and any CRT display artifacts
- Apply a mild S-Video cable rolloff to the chroma channel only: `chroma_cutoff ≈ 3dB at 2.5MHz_equiv`
- **Pin 3/4 crosstalk**: cheap S-Video cables have some Y-into-C leakage — `C_out += Y_signal * 0.02f * svideo_cable_quality`

---

## 12. TV Front-End Processing

This stage runs after the connection-type encoding/decoding and before the CRT simulation.

### 12a. TV Tuner / Input Selector

- Model the input selector relay as a brief (~3-frame) static burst when switching inputs
- Each input type has a different input impedance characteristic — RF is 75Ω terminated, composite/S-Video inputs have slightly different high-frequency response
- **Input noise floor**: even a good TV input adds a small noise floor — `input_noise = lerp(0.04f, 0.008f, float(connection_type))` (RF worst, S-Video best)

### 12b. Composite Comb Filter (Composite path only)

The comb filter is the TV's attempt to separate Y from C in the composite signal. Consumer TVs had several generations of this technology:

- **No comb filter** (cheapest TVs, pre-1985): simple notch filter at 3.58MHz — severe dot crawl and cross-color
  ```cpp
  // Just notch out the subcarrier frequency from luma — blunt and lossy
  Y_out = Y_composite - bandpass(Y_composite, kSC_FREQ, 0.5f);
  C_out = bandpass(Y_composite, kSC_FREQ, 0.5f);
  ```
- **1-line comb filter** (mid-range TVs): averages the current line with the line one H-period earlier — cancels subcarrier but reduces vertical chroma resolution
- **2-line digital comb filter** (better TVs): uses two-line delay for better separation — still leaves some residual dot crawl at diagonal edges
- Select comb filter quality via a `tv_comb_quality` enum that affects how much dot crawl and cross-color leak through

### 12c. TV Color Decoder

- **Hue control**: rotate the U/V vector by `tv_hue_offset` degrees — consumer TVs often had hue slightly off from factory
- **Saturation/color control**: scale the U/V magnitude: `float sat_scale = tv_color * 0.8f + 0.2f` (range ~0.6–1.4×)
- **Color killer**: if chroma signal falls below a threshold, the TV cuts color entirely — goes to B&W. Trigger when `vp.chroma_level < 0.15f`
- **Burst phase lock**: the color decoder locks to the color burst in the back porch of each line. Model burst phase noise as a per-line U/V rotation of ±2–5° driven by `vp.fm_carrier_noise`

### 12d. TV Brightness / Contrast / Sharpness Controls

These are per-TV settings that the user should be able to adjust. Map them onto the signal before CRT rendering:

```cpp
// Applied after color decoding, before CRT simulation
Y_out = (Y_in - 0.5f) * tv_contrast + 0.5f + (tv_brightness - 0.5f) * 0.3f;
// Sharpness: high-frequency boost/cut on luma
float Y_hf   = Y_in - gaussian_blur(Y_in, 1.5f);
Y_out       += Y_hf * (tv_sharpness - 0.5f) * 1.2f;
```

### 12e. TV Vertical / Horizontal Hold

- Model the TV's sync separator circuit as having a hold range — if the VCR's sync pulses drift outside this range the TV loses lock
- **Vertical hold loss**: when `v_roll_accum_` is nonzero, the TV's vertical oscillator drifts — add a slow vertical roll to the display output (distinct from the VCR's own roll)
- **Horizontal AFC**: the TV's horizontal oscillator has its own AFC that can lose lock on very noisy signals — model as a brief horizontal tear that the AFC recovers from within 2–5 lines

---

## 13. CRT Display Simulation

The CRT is the final and most visually distinctive stage. It transforms the decoded video signal into light through physical electron beam and phosphor processes.

### 13a. Electron Beam Scan & Scanline Structure

The CRT draws the image one horizontal line at a time using a focused electron beam:

- **Scanline gap**: the beam has a finite diameter — adjacent scanlines do not fill the full vertical space, leaving a faint dark gap between them. Model as a vertical modulation:
  ```cpp
  float beam_y    = fmod(y_screen, 1.0f); // 0–1 within each scanline row
  float scanline_mask = 1.0f - pow(sin(beam_y * kPI), 8.0f) * scanline_strength * 0.35f;
  pixel_out *= scanline_mask;
  ```
- **Beam focus**: the beam is not perfectly sharp. Apply a mild 2D Gaussian blur (σ ≈ 0.6px) to the entire output before the scanline mask — this softens the image slightly and makes scanline edges blend naturally
- **Beam width variation**: the beam blooms (widens) on bright areas due to space charge repulsion — bright pixels bleed slightly wider than dark ones:
  ```cpp
  float bloom_radius = 0.5f + luma_at_pixel * tv_bloom * 1.5f;
  // Apply as spatially-varying blur radius
  ```

### 13b. Phosphor Properties

Different CRT phosphors have distinct color temperature, persistence, and spectral properties:

- **P22 phosphor** (standard consumer TV): warm slightly yellow-white point, short persistence (~1ms)
- **Persistence / afterglow**: bright pixels leave a faint trail when objects move — implement as a temporal blend:
  ```cpp
  phosphor_buffer[y][x] = pixel_current * (1.f - phosphor_persistence)
                        + phosphor_buffer[y][x] * phosphor_persistence;
  // phosphor_persistence ≈ 0.08–0.18f for P22
  ```
- **Color temperature**: apply a color matrix shift to move the white point from D65 toward a slightly warmer (~6000K) or cooler tone depending on TV model
- **Phosphor triad RGB separation**: at high zoom levels, the three phosphor dots (or stripes for Trinitron) are visible. Model as a very fine (3px period) RGB subpixel pattern multiplied at full render resolution

### 13c. Shadow Mask / Aperture Grille

Consumer CRTs use a shadow mask (dot-triad pattern) or aperture grille (vertical stripe pattern, Trinitron/Diamondtron):

- **Shadow mask**: a fine grid of holes that only allows the beam to hit the correct phosphor color. Visible as a faint dot grid at close range:
  ```cpp
  float mask_x = fmod(x_screen * mask_pitch, 1.0f);
  float mask_y = fmod(y_screen * mask_pitch + (int(x_screen * mask_pitch) % 2) * 0.5f, 1.0f);
  float mask   = 0.75f + 0.25f * smoothstep(0.3f, 0.7f, mask_x) * smoothstep(0.3f, 0.7f, mask_y);
  pixel_out   *= mask;
  ```
- **Aperture grille** (Trinitron): vertical stripe pattern instead of dots, with faint horizontal damper wires visible at ~1/3 and ~2/3 screen height as thin dark horizontal lines
- **Mask pitch**: typically 0.25–0.31mm dot pitch on consumer TVs — map to pixel density of the output resolution

### 13d. CRT Geometry Distortion

The curved CRT screen and deflection circuits introduce geometric distortion:

- **Barrel / pincushion distortion**: early consumer CRTs barrel, later ones corrected but often over-corrected to pincushion:
  ```cpp
  float nx = (x / float(W) - 0.5f) * 2.f;  // -1 to 1
  float ny = (y / float(H) - 0.5f) * 2.f;
  float r2 = nx*nx + ny*ny;
  float distort = 1.f + tv_pincushion * r2 * 0.15f;
  float sx = (nx * distort * 0.5f + 0.5f) * W;
  float sy = (ny * distort * 0.5f + 0.5f) * H;
  pixel_out = bsample(input, sx, sy);
  ```
- **Keystoning**: slight trapezoidal deformation if the deflection yoke is misaligned — add a small horizontal shear that varies with `y`
- **Corner rounding**: the screen bezel clips the corners of the display area — mask the corners with a rounded rectangle at ~95% of frame size

### 13e. Convergence Errors

The CRT has three electron guns (R, G, B) that must converge at every point on the screen. Misconvergence causes color fringing:

- **Static misconvergence**: a global R/G/B offset, typically a pixel or less:
  ```cpp
  R_out = sample(input.R, x + convergence_r.x, y + convergence_r.y);
  G_out = sample(input.G, x + convergence_g.x, y + convergence_g.y);
  B_out = sample(input.B, x + convergence_b.x, y + convergence_b.y);
  ```
- **Dynamic misconvergence**: the offset worsens toward screen corners — scale the R/G/B offset by distance from screen center squared, peaking at ~2–3px at extreme corners
- **Defocus at corners**: the electron beam is less focused at the edges — apply a slightly stronger blur at screen corners using a spatially-varying Gaussian radius

### 13f. CRT Brightness, Glow, and Bloom

- **Halation**: extremely bright areas scatter light inside the glass faceplate — add a large-radius (σ ≈ 20–40px), low-amplitude (2–5%) Gaussian glow on pixels above ~80% brightness
- **Vignetting**: the screen is slightly darker at the edges and corners due to the shadow mask angle and beam landing geometry — apply a radial falloff: `vignette = 1.f - dist_from_center^2 * 0.12f`
- **Black level lift**: a CRT's black is not true black due to ambient phosphor glow — add a `0.02–0.05f` lift to the minimum brightness level (heavier with room lights on)

### 13g. Screen Reflection and Glass

The CRT faceplate is glass — it reflects ambient light and the viewer:

- **Specular reflection**: add a faint static highlight in the upper-right quadrant at ~3–6% intensity (simulates a room light/window)
- **Diffuse room light**: add a uniform `+0.01–0.03f` additive lift scaled by `tv_room_brightness`
- **Anti-glare coating** (some TVs): replace the specular highlight with a diffuse, slightly blurry glow that desaturates nearby pixels

### 13h. CRT Warmup Behavior

A cold CRT takes ~30–60 seconds to reach stable brightness and geometry:

- On startup, start brightness at 40% and ramp to 100% over 45 seconds
- Geometry is slightly unstable for the first 10 seconds — add a slow damped oscillation to the vertical size: `vsize = 1.f + 0.04f * exp(-warmup_time * 0.15f) * sin(warmup_time * 2.f)`
- Color temperature shifts from warm (red-heavy) toward neutral as the tube warms up

---

## 14. TV Model Presets

Bundle the TV simulation parameters into presets representing real TV archetypes. Each preset sets connection quality, comb filter type, CRT geometry, phosphor, and convergence:

| Preset | Era | Comb Filter | Mask Type | Geometry | Notes |
|--------|-----|-------------|-----------|----------|-------|
| **Cheap 13" (RF only)** | 1984 | None (notch) | Shadow mask 0.31mm | Barrel | Heavy dot crawl, noisy RF |
| **Mid-range 19" (Composite)** | 1988 | 1-line analog | Shadow mask 0.28mm | Slight barrel | Typical living room TV |
| **Better 25" (Composite)** | 1992 | 2-line digital | Shadow mask 0.26mm | Near-flat | Cleaner image, slight pincushion |
| **Trinitron 27" (S-Video)** | 1994 | 2-line digital | Aperture grille | Flat | Best consumer CRT quality, damper wires visible |
| **Projection TV** | 1990 | 1-line analog | N/A (3-gun lens) | Keystone | Soft focus, color convergence issues |

---

## 15. Full Signal Chain — Stage-by-Stage Data Flow

Add a `TVParams` struct to house all TV-side parameters, separate from `VideoParams`:

```cpp
struct TVParams {
    VCROutputType   output_type     = VCROutputType::Composite_RCA;
    TVPreset        preset          = TVPreset::MidRange19;

    // Front-end
    float           input_noise     = 0.015f;
    float           cable_quality   = 0.85f;
    float           hum_level       = 0.0f;
    float           rf_interference = 0.0f;

    // Color decoder
    float           tv_hue          = 0.0f;    // degrees
    float           tv_color        = 1.0f;
    float           tv_brightness   = 0.5f;
    float           tv_contrast     = 1.0f;
    float           tv_sharpness    = 0.5f;
    CombFilterType  comb_quality    = CombFilterType::TwoLine;

    // CRT display
    float           scanline_strength = 0.4f;
    float           phosphor_persistence = 0.12f;
    float           tv_bloom        = 0.3f;
    float           tv_pincushion   = 0.1f;
    float           convergence_error = 0.3f;
    float           halation        = 0.15f;
    float           vignette        = 0.6f;
    MaskType        mask_type       = MaskType::ShadowMask;
    float           mask_pitch      = 0.28f;   // mm
    float           warmup_time     = 999.f;   // seconds since power-on
    float           room_brightness = 0.3f;
    bool            aperture_wires  = false;   // Trinitron damper wires
};
```

The processing order in `NTSCSimulator::process()` should become:

```
1.  Split input BGR → float YUV
2.  VCR tape signal chain  (sections 1–10)
3.  VCR output encode      (section 11 — encode to RF/Composite/SVideo)
4.  TV input decode        (section 12a–12b — decode from connection type)
5.  TV color decoder       (section 12c)
6.  TV brightness/contrast (section 12d)
7.  Recombine YUV → BGR
8.  CRT geometry warp      (section 13d)
9.  Beam blur + scanlines  (section 13a–13b)
10. Shadow mask            (section 13c)
11. Convergence errors     (section 13e)
12. Halation + vignette    (section 13f)
13. Screen glass / reflect (section 13g)
14. Phosphor persistence   (section 13b — temporal blend into phosphor buffer)
```

---

---

## 16. Helical Head Drum Simulation

This section documents what the current code gets right, what it gets wrong, and how to fix it to properly simulate a spinning two-head drum reading a moving tape.

### 16a. Current State Assessment

The existing `ntsc_simulator.cpp` has a genuine composite NTSC encode/decode loop, real burst phase extraction via `atan2(sumQ, sumI)`, and physically motivated TBE accumulation from drum wobble, belt slip, motor cogging, and crease shear. These are correct in architecture. However three critical gaps prevent it from sounding like a real helical scan machine:

**Gap 1 — No FM luma encode/decode.** This is the biggest single deficiency. Real VHS FM-encodes luma onto a carrier at 4.4MHz (SP) or 3.4MHz (LP). The decode chain applies FM de-emphasis — a first-order high-frequency rolloff — which is the dominant cause of the VHS soft look. The current code copies pixel luma directly into `Y` with no FM stage. The `fm_carrier_noise_deg` parameter adds additive white noise onto `Y`, which has the wrong spectral shape: FM demodulation noise has a triangular spectrum (rising with frequency), not flat.

**Gap 2 — No chroma color-under heterodyne.** Real VHS downconverts the 3.58MHz chroma subcarrier to ~629kHz before writing to tape, then upconverts on playback. This is why VHS chroma has only ~500kHz of bandwidth — the characteristic wide horizontal color bleed. The current code works at the full NTSC subcarrier frequency throughout. The `chrA = 0.12f` IIR gives some chroma rolloff but it is not calibrated to the correct color-under bandwidth and does not model the heterodyne phase noise introduced by the upconvert.

**Gap 3 — Head sweep slope is a free parameter, not geometry-derived.** The current code uses:
```cpp
float head_sweep_px_per_line = helical_sweep_deg * 1.5f;
float head_sweep_grad        = head_sweep_px_per_line / float(W);
```
This is a UI knob rather than a value derived from the physical geometry. On a healthy tape at SP, the head sweep contribution to TBE should be a fixed, predictable ramp calculated from `kVHS_HEAD_ANGLE_DEG`, drum RPM, and tape speed. The correct derivation is in section 16b.

**Gap 4 — No separate field buffers.** Real VHS records one field per head pass. The current code processes the full frame as a progressive image, using the "field" distinction only to set `field_phase_offset` in chroma. Lines are not split into even/odd fields with their own spatial sampling and a half-line vertical offset between them.

**Gap 5 — RNG is re-seeded 960 times per frame.** A full `std::mt19937` is constructed and seeded for each of the 480 scanlines (twice per line — one for the per-line setup, one `snowRng` in the output loop). Seeding Mersenne Twister requires initializing 624 `uint32_t` values. This prevents vertical noise correlation and is a significant performance cost.

---

### 16b. Physical Drum Geometry — Deriving the TBE Ramp

The drum rotates at exactly 1800 RPM = 30 rev/sec. Two heads are mounted 180° apart, so each head sweeps the tape 30 times per second = 60 field passes per second (matching the 59.94Hz NTSC field rate within the mechanical tolerance of the servo system).

The head is mounted at `kVHS_HEAD_ANGLE_DEG = 5.967°` from the drum equator. The tape wraps approximately 180° around the drum. During one full head sweep, the tape advances by exactly one track pitch (SP: `kVHS_TRACK_PITCH_SP = 0.049mm`). The horizontal timing shift per scanline introduced by the helical geometry is:

```cpp
const float kHeadAngleRad       = kVHS_HEAD_ANGLE_DEG * kPI / 180.f;
// tape speed in mm/sec from IPS
const float tape_speed_mm_s     = ep.ips_base * 25.4f;
// the horizontal component of head-over-tape velocity contributes a timing ramp
// H-period = 1 / kNTSC_HSYNC seconds per line
const float H_period_s          = 1.f / kNTSC_HSYNC;
// distance the tape moves per H-period, projected along the scan direction
float tbe_mm_per_line           = tape_speed_mm_s * std::cos(kHeadAngleRad) * H_period_s;
// convert to pixel offset: track pitch → full line width
float head_sweep_grad_derived   = (tbe_mm_per_line / kVHS_TRACK_PITCH_SP) / float(W);
```

This replaces the `helical_sweep_deg * 1.5f / W` approximation with a value grounded in `kVHS_HEAD_ANGLE_DEG`, `ep.ips_base`, and `kNTSC_HSYNC` — all of which are already defined in `constants.h`. At SP (15 IPS) this gives a ramp of approximately 0.6 pixels across the full line width, which is the correct order of magnitude.

---

### 16c. FM Luma Encode / Decode

Add these two passes around the existing per-pixel YIQ work. They operate on the Y channel only, before composite modulation and after composite demodulation respectively.

**Pre-emphasis (record side):**
```cpp
// First-order high shelf boost — approximates the real FM pre-emphasis network
// preemph_coeff ≈ 0.18 for SP, 0.22 for LP (more HF boost needed at lower carrier)
float lp_factor      = 1.f - (ep.ips_base / 15.f);   // 0=SP, 1=EP
float preemph_coeff  = dsp::lerp(0.18f, 0.26f, lp_factor);
float Y_preemph      = Y + (Y - Y_prev_encoded) * preemph_coeff;
Y_prev_encoded       = Y;
// FM carrier clipping — soft limiter at +110 / −10 IRE
Y_preemph            = std::tanh(Y_preemph * 1.8f) / 1.8f;
```

**FM noise (triangular spectrum):**
Before the decode low-pass, inject noise whose amplitude rises with frequency. Approximate by differencing two white noise samples — this gives a first-order high-shelved spectrum matching FM demodulation noise:
```cpp
float fm_noise_a  = noise_buf[y * W_TOTAL + x];
float fm_noise_b  = noise_buf[y * W_TOTAL + std::max(0, x - 1)];
float fm_noise    = (fm_noise_a - fm_noise_b) * vp.luma_noise * fm_noise_scale;
Y_composite      += fm_noise;
```

**De-emphasis (playback side) — replaces the current `lpfA = 0.35f` constant:**
```cpp
// Speed-dependent cutoff. In pixel-frequency terms:
//   SP bandwidth ≈ 3.0MHz → lpfA ≈ 0.32
//   LP bandwidth ≈ 2.4MHz → lpfA ≈ 0.26
//   EP bandwidth ≈ 2.0MHz → lpfA ≈ 0.22
float lp_factor  = 1.f - (ep.ips_base / 15.f);
float lpfA       = dsp::lerp(0.32f, 0.22f, lp_factor);  // replaces hardcoded 0.35f
lpY             += lpfA * (sig - lpY);
```

This one change — making `lpfA` speed-dependent — is the highest-payoff single edit in the entire codebase. It gives SP its relatively sharp look and EP its distinctively muddy, smeared appearance without any other changes.

---

### 16d. Chroma Color-Under Heterodyne

The color-under system operates in two stages that must be added as explicit passes, one before composite encode and one after composite decode.

**Record-side downconvert (before composite modulation):**

The chroma signal is multiplied by a 629kHz reference oscillator, shifting it from 3.58MHz down to ~629kHz. In pixel-frequency space, 629kHz corresponds to approximately `FW * 629 / 4400 ≈ 91 pixels` per cycle. After the downconvert, a narrow bandpass of ~500kHz width is applied — this is the bandwidth-limiting step that causes the horizontal chroma bleed:

```cpp
// Chroma bandwidth limit before recording (color-under BW ≈ 500kHz)
// In pixel terms: chroma_radius ≈ FW * 500 / 4400 ≈ 73px
int chroma_radius = int(float(W) * 0.5f / 4.4f);  // ~73px SP, ~85px LP
// Apply as a box filter on U and V channels separately before composite modulation
// This is the single operation most responsible for the VHS chroma bleed look
```

**Playback-side upconvert (after composite demodulation):**

The heterodyne upconvert introduces phase noise proportional to the quality of the VCR's color AFC circuit. After the existing `si`/`sq` IIR filters, apply a per-field phase rotation driven by the color-under phase error:

```cpp
// Per-field heterodyne phase error — accumulates each field
// This is distinct from the burst phase error already tracked in field_chroma_phase_accum_
float heterodyne_phase = field_chroma_phase_accum_
                       + vp.inter_field_phase_error * adjacent_track_phase_ * kPI * 0.5f;
float U_out = si * std::cos(heterodyne_phase) - sq * std::sin(heterodyne_phase);
float V_out = si * std::sin(heterodyne_phase) + sq * std::cos(heterodyne_phase);
```

---

### 16e. Two-Head State Machine

Replace the implicit field switching with an explicit head state machine. Each head has its own independently varying RF level and azimuth offset, which causes the slight head-to-head brightness and color variation visible on real VHS:

```cpp
struct HeadState {
    float rf_level       = 1.0f;    // slight variation head to head
    float azimuth_error  = 0.0f;    // degrees — causes HF rolloff when misaligned
    float chroma_phase   = 0.0f;    // each head has independent chroma AFC error
};
HeadState heads_[2];  // add to NTSCSimulator

// Per-frame update (outside the scanline loop):
float drum_angle = std::fmod(tapeTime_ * (kVHS_DRUM_RPM / 60.f), 1.f);
int   active_head = (drum_angle >= 0.5f) ? 1 : 0;

// Head switching transition — brief signal dropout at the switch point
float switch_proximity = std::min(
    std::abs(drum_angle - 0.0f),
    std::abs(drum_angle - 0.5f));
float switch_dropout = std::max(0.f, 1.f - switch_proximity * 60.f);
// switch_dropout > 0 for ~1 line around each head switch
```

Use `active_head` to select the per-head RF level and chroma phase when processing each field, instead of deriving everything from the global `currentField` integer.

---

### 16f. Separate Field Buffers

Add two field-sized buffers to `NTSCSimulator` for proper interlaced processing:

```cpp
// Add to NTSCSimulator private members:
cv::Mat field_buf_[2];   // [0] = even field, [1] = odd field
cv::Mat field_out_[2];

void NTSCSimulator::initBuffers(int w, int h) {
    // existing init ...
    field_buf_[0].create(h / 2, w, CV_8UC3);
    field_buf_[1].create(h / 2, w, CV_8UC3);
    field_out_[0].create(h / 2, w, CV_8UC3);
    field_out_[1].create(h / 2, w, CV_8UC3);
}
```

In `process()`, split the input into two half-height fields before the scanline loop, process each independently (with its own head state, chroma phase, and TBE), then recombine with a half-line vertical offset:

```cpp
// Split: field 0 = even rows, field 1 = odd rows
for (int y = 0; y < H / 2; ++y) {
    in.row(y * 2    ).copyTo(field_buf_[0].row(y));
    in.row(y * 2 + 1).copyTo(field_buf_[1].row(y));
}

// Process each field with its own head parameters
processField(field_buf_[0], field_out_[0], frameNum, ep, vp, tapeSpd, instantSpd, wallDt, 0);
processField(field_buf_[1], field_out_[1], frameNum, ep, vp, tapeSpd, instantSpd, wallDt, 1);

// Recombine with half-line vertical offset (interlace structure)
for (int y = 0; y < H / 2; ++y) {
    field_out_[0].row(y).copyTo(out.row(y * 2));
    field_out_[1].row(y).copyTo(out.row(y * 2 + 1));
}
```

---

## 17. Performance Analysis & Optimization Plan

### 17a. Current Bottleneck Profile

At 640×480, the inner loop processes `W_TOTAL = 784` samples × 480 lines = **376,320 iterations per frame**. The per-frame cost by operation, estimated on a modern x86 CPU with -O2:

| Operation | Per-frame count | Unit cost | Estimated total |
|-----------|----------------|-----------|-----------------|
| `std::normal_distribution` calls (inner loop) | ~376K | ~20ns | **~7.5ms** |
| `std::sin` / `std::cos` for subcarrier `decTheta` | ~376K each | ~5ns | **~3.8ms** |
| `std::mt19937` construction + seed (960/frame) | 960 | ~1µs | **~1.0ms** |
| IIR filter updates (`lpY`, `si`, `sq`) | ~376K | ~2ns | ~0.75ms |
| Burst phase sin/cos loop | 480 × 40 | ~5ns | ~0.1ms |
| RGB reconstruction + clamp | ~376K | ~3ns | ~1.1ms |
| **Estimated total** | | | **~14–16ms** |

The `processing_pipeline.cpp` loop timer gates at **32ms minimum**, capping throughput at ~31fps regardless of how fast the NTSC math runs. This is intentional pacing but should be changed to `1000.f / g_sourceFPS.load()` to track the source frame rate properly.

The dominant cost is the `std::normal_distribution<float>` call inside the per-pixel `x` loop at line 553 (`pll_drift_accum += std::normal_distribution<float>(0,1)(rng) * pll_instability`). This alone accounts for roughly half the frame budget.

---

### 17b. Optimization 1 — Pre-generated Noise Buffer (highest priority)

Replace all per-pixel RNG calls with reads from a pre-generated float buffer. This is the single most impactful change — it eliminates ~7.5ms of per-frame cost, removes the 960 MT seedings, and as a side effect makes noise vertically correlated (which is more physically correct — real tape noise streaks diagonally, not independently per scanline).

```cpp
// Add to NTSCSimulator private members:
std::vector<float> noise_buf_;   // W_TOTAL * H floats, regenerated each frame

// Fast PRNG — xorshift32, ~1ns per call vs ~20ns for normal_distribution + MT
static inline uint32_t xorshift32(uint32_t& s) {
    s ^= s << 13; s ^= s >> 17; s ^= s << 5;
    return s;
}
static inline float xorshift_f(uint32_t& s) {
    return (float)(xorshift32(s) >> 8) * (1.f / 16777216.f) - 1.f; // [-1, 1]
}

// In process(), before the scanline loop:
noise_buf_.resize(W_TOTAL * H);
uint32_t ns = uint32_t(frameNum) * 2654435761u ^ uint32_t(recording_id_) * 40503u;
for (float& v : noise_buf_) v = xorshift_f(ns);

// In the inner x loop, replace std::normal_distribution calls:
// BEFORE: pll_drift_accum += std::normal_distribution<float>(0,1)(rng) * pll_instability;
// AFTER:
pll_drift_accum += noise_buf_[y * W_TOTAL + x] * pll_instability;
```

Apply the same pattern everywhere `rng` is called inside either loop — `scrape_flutter`, `line_flutter_base`, snow generation, and dropout edge noise all become reads from `noise_buf_` at offset positions. Use different stride multipliers to decorrelate them:

```cpp
float scrape_flutter   = noise_buf_[y * W_TOTAL + 0]       * ep.flutter_dep * 0.4f;
float line_flutter     = noise_buf_[y * W_TOTAL + 1]       * flutter_dev * 0.008f;
// per-pixel pll noise:
float pll_noise        = noise_buf_[y * W_TOTAL + x];
// per-pixel snow (use second half of buffer to decorrelate from pll):
float snow_noise       = noise_buf_[(H - 1 - y) * W_TOTAL + (W_TOTAL - 1 - x)];
```

**Expected saving: ~8–9ms per frame (roughly halving total cost).**

---

### 17c. Optimization 2 — Subcarrier sin/cos Lookup Table

The subcarrier phase `decTheta` advances by exactly `kSC_PX` per pixel per line. Precompute a table of length `W_TOTAL` at `initBuffers()` time:

```cpp
// Add to NTSCSimulator private members:
std::vector<float> sc_cos_, sc_sin_;  // length W_TOTAL

// In initBuffers():
sc_cos_.resize(W_TOTAL);
sc_sin_.resize(W_TOTAL);
for (int x = 0; x < W_TOTAL; ++x) {
    float base_theta = float(x) * kSC_PX;
    sc_cos_[x] = std::cos(base_theta);
    sc_sin_[x] = std::sin(base_theta);
}
```

In the inner loop, the full `decTheta` adds a per-line and per-field phase offset on top of this base. Use angle-sum identities to apply that offset without additional sin/cos calls:

```cpp
// Precompute once per line (outside the x loop):
float cos_line_phase = std::cos(linePhase + phase_accum + pll_drift_line_offset + field_phase_offset * 0.5f);
float sin_line_phase = std::sin(linePhase + phase_accum + pll_drift_line_offset + field_phase_offset * 0.5f);

// Inside x loop — angle-sum: cos(A+B) = cosA*cosB - sinA*sinB
float cos_theta = sc_cos_[x] * cos_line_phase - sc_sin_[x] * sin_line_phase;
float sin_theta = sc_sin_[x] * cos_line_phase + sc_cos_[x] * sin_line_phase;
float rawI = 2.0f * chromaSig * cos_theta;
float rawQ = -2.0f * chromaSig * sin_theta;
```

This reduces the inner-loop sin/cos from 2 transcendental calls to 4 multiplies and 2 adds.

**Expected saving: ~3–4ms per frame.**

---

### 17d. Optimization 3 — Decouple Encode and Decode for OpenMP

The current loop has a data dependency via `prev_comp_line` / `curr_comp_line` swap — line `y` reads from line `y-1`'s encoded composite data to extract the adjacent-track crosstalk and head pre-echo. This prevents parallelism across lines.

Restructure into two separate passes:

**Pass 1 (encode — parallel):** For each line independently, compute the encoded composite signal from the source pixels and write it to a full-frame composite buffer `comp_frame[H][W_TOTAL]`. This pass has no inter-line dependencies if you defer the `prev_comp_line` references.

**Pass 2 (decode — parallel):** For each line, read from `comp_frame[y]` and `comp_frame[y-1]` (read-only access to adjacent line — safe for parallel read) and produce the output RGB. The IIR filter state (`lpY`, `si`, `sq`) is local to each line's decode pass and does not cross line boundaries.

```cpp
// Allocate once in initBuffers():
std::vector<float> comp_frame_;  // H * W_TOTAL floats

// Pass 1 — encode, parallelizable:
#pragma omp parallel for schedule(static) if(ADO_OPENMP)
for (int y = 0; y < H; ++y) {
    encodeLineToComposite(y, in, &comp_frame_[y * W_TOTAL], ...);
}

// Pass 2 — decode, parallelizable (adjacent line read-only):
#pragma omp parallel for schedule(static) if(ADO_OPENMP)
for (int y = 0; y < H; ++y) {
    decodeLineFromComposite(y, &comp_frame_[y * W_TOTAL],
                            y > 0 ? &comp_frame_[(y-1) * W_TOTAL] : nullptr,
                            out, ...);
}
```

With 4 threads this alone could bring a 14ms frame down to ~4–5ms.

---

### 17e. Optimization 4 — Fix the Pipeline Gate

The 32ms gate in `processing_pipeline.cpp` is the current throughput ceiling:

```cpp
// CURRENT (line ~47 of processing_pipeline.cpp):
if (elapsed < 32) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    continue;
}

// REPLACE WITH:
float target_frame_ms = 1000.f / std::max(15.f, g_sourceFPS.load());
if (elapsed < long long(target_frame_ms - 1.f)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    continue;
}
```

This allows the pipeline to run at the source frame rate — 29.97fps for NTSC sources, up to 60fps for camera input — rather than always being capped at 31fps.

---

### 17f. Optimization 5 — Speed-Dependent Early Exit

Many expensive effects — tracking error, crease shear, motor noise, PLL instability, guide bounce — are zero or near-zero when the tape is healthy. Gate their computation:

```cpp
const bool has_tracking_error  = tracking_error_lpf_ > 0.01f;
const bool has_motor_defect    = motor_defect > 0.05f;
const bool has_crease          = vp.tape_crease > 0.01f;
const bool has_pll_noise       = pll_instability > 0.001f;

// Inside x loop:
if (has_pll_noise) {
    pll_drift_accum += noise_buf_[y * W_TOTAL + x] * pll_instability;
}
if (has_tracking_error) {
    // h_sync_shear computation
}
```

On a clean tape with no degradation, this eliminates most of the conditional arithmetic in the inner loop. This is also the correct behavior — a high-quality tape at SP should produce clean output with minimal overhead.

---

### 17g. Projected Performance After Optimizations

| Optimization | Saving | Cumulative estimate |
|---|---|---|
| Baseline | — | ~15ms/frame |
| 17b: Pre-generated noise buffer | ~8ms | ~7ms/frame |
| 17c: Subcarrier LUT | ~3.5ms | ~3.5ms/frame |
| 17d: OpenMP 4 threads | ~2ms | ~1.5ms/frame |
| 17e+17f: Gate fix + early exits | ~0.5ms | **~1ms/frame** |

At ~1ms per frame after all optimizations, the pipeline can comfortably run at 60fps with headroom for the TV/CRT simulation stages (sections 11–13) which will add roughly 2–5ms depending on which CRT features are enabled.

---

## 18. Implementation Order (Updated)

| Priority | Stage | Feature | Key Params / Notes |
|----------|-------|---------|------------|
| 1 | Perf | Pre-generated noise buffer | Replaces all inner-loop RNG calls |
| 2 | Perf | Subcarrier sin/cos LUT | Built in `initBuffers()` |
| 3 | VCR | YUV colorspace split | Prerequisite for FM and color-under |
| 4 | VCR | Speed-dependent luma de-emphasis | Replace `lpfA = 0.35f` with `lerp(0.32f, 0.22f, lp_factor)` |
| 5 | VCR | FM pre-emphasis + de-emphasis | First-order high shelf, `preemph_coeff` from `ep.ips_base` |
| 6 | VCR | Chroma color-under bandwidth limit | Box blur ~73px on U/V before composite encode |
| 7 | VCR | Geometry-derived head sweep ramp | Replace `helical_sweep_deg * 1.5f` with section 16b formula |
| 8 | VCR | Two-head state machine | Per-head RF level + chroma phase via `drum_angle` |
| 9 | VCR | Separate field buffers | Split/process/recombine even+odd fields |
| 10 | VCR | Chroma phase rotation per field | `field_chroma_phase_accum_` → actual U/V rotation |
| 11 | VCR | Horizontal line jitter | `tracking_error`, `head_switch_jitter` |
| 12 | VCR | Noise bar with proper snow | `noise_bar_phase_`, `dropout_rate` |
| 13 | VCR | Head switch artifact at line H−7 | `head_switch_jitter`, `drum_eccentricity` |
| 14 | VCR | Dropout bursts | `dropout_rate`, `dropout_intensity` |
| 15 | VCR | FM ringing / edge overshoot | `base_rf_level` |
| 16 | VCR | Wow/flutter on chroma phase | `wow_dep`, `flutter_dep` |
| 17 | VCR | Oxide shedding / sticky shed | `oxide_shedding`, `sticky_shed` |
| 18 | VCR | Interlace combing on motion | `inter_field_phase_error` |
| 19 | Perf | OpenMP encode/decode split | Decouple passes for parallelism |
| 20 | Perf | Pipeline gate fix | Track `g_sourceFPS` not hardcoded 32ms |
| 21 | Output | Connection type encode/decode | `output_type` |
| 22 | Output | Composite dot crawl / cross-color | `comb_quality` |
| 23 | Output | RF herringbone + ghosting | `rf_interference` |
| 24 | TV | Brightness / contrast / sharpness | `tv_brightness`, `tv_contrast` |
| 25 | TV | Color decoder hue + saturation | `tv_hue`, `tv_color` |
| 26 | CRT | Scanline mask + beam blur | `scanline_strength` |
| 27 | CRT | Phosphor persistence | `phosphor_persistence` |
| 28 | CRT | Barrel / pincushion distortion | `tv_pincushion` |
| 29 | CRT | Convergence errors | `convergence_error` |
| 30 | CRT | Shadow mask / aperture grille | `mask_type`, `mask_pitch` |
| 31 | CRT | Halation + vignetting | `halation`, `vignette` |
| 32 | CRT | Screen reflection + glass | `room_brightness` |
| 33 | CRT | Warmup behavior | `warmup_time` |
| 34 | TV | TV model presets | `preset` |

---

## 19. Parameter Plumbing Notes

- **`vp.signal_strength`** is already modified in `processing_pipeline.cpp` based on effects degradation — use it as a master RF level multiplier throughout the VCR signal chain. It should feed into `TVParams::input_noise` as well: weaker VCR signal means the TV's AGC has to work harder
- **`tapeTime_` and `wallTime_`** should drive deterministic degradation events (creases, shed events). With the noise buffer approach (section 17b), seed the noise buffer with `recording_id_` XOR `frameNum` so per-frame noise is unique but per-recording events (crease positions) remain consistent
- **`recording_id_` / `recording_blend_` / `recording_transition_time_`** model generational tape recording — each new recording ID should slightly increase `tape_age` and introduce new fixed crease positions seeded to that ID
- **SP vs LP** is already parameterized through `ep.ips_base` — all bandwidth and crosstalk values must scale off this. The key derived quantity is `float lp_factor = 1.f - (ep.ips_base / 15.f)` which gives 0 at SP (15 IPS), 0.5 at LP (7.5 IPS), and 1.0 at EP (3.75 IPS). `lpfA`, `preemph_coeff`, `chroma_radius`, and `head_sweep_grad_derived` all depend on it
- **`noise_buf_`** should be allocated once in `initBuffers()` at size `W_TOTAL * H` and reused each frame — avoid `resize()` in the hot path
- **`comp_frame_`** (the full-frame composite buffer for the encode/decode split) should similarly be pre-allocated in `initBuffers()` at `W_TOTAL * H * sizeof(float)` — at 640×480 this is `784 * 480 * 4 = ~1.5MB`, well within L3 cache on most CPUs, which is important for the parallel decode pass
- **`field_buf_`** and `field_out_`** buffers need to live inside `NTSCSimulator` as persistent `cv::Mat` objects and be allocated in `initBuffers()` — avoid per-frame `create()` calls
- **Phosphor buffer** needs to live inside `NTSCSimulator` as a persistent `cv::Mat` so it accumulates correctly across frames — add alongside the existing `W_`, `H_BLANK` members
- **CRT geometry warp** should use a pre-computed `cv::Mat` of `cv::Vec2f` sample coordinates built once at init and rebuilt only when `tv_pincushion` or screen size changes — warping every pixel every frame via `bsample` at full resolution is expensive
- **TV preset switching** should trigger a brief static burst + color instability (2–4 frames) to simulate the TV's input relay and AGC settling
