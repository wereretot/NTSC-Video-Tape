# VHS / VCR Technical Reference for NTSC Simulation
### A Complete Signal-Level Guide

---

## Table of Contents

1. [Overview of the VHS/VCR Signal Chain](#1-overview-of-the-vhsvcr-signal-chain)
2. [NTSC Broadcast Signal Fundamentals](#2-ntsc-broadcast-signal-fundamentals)
3. [The Composite Video Signal](#3-the-composite-video-signal)
4. [VHS Recording: Signal Processing & Modulation](#4-vhs-recording-signal-processing--modulation)
5. [The VHS FM Luminance Carrier](#5-the-vhs-fm-luminance-carrier)
6. [Chroma Signal Handling: The Color-Under System](#6-chroma-signal-handling-the-color-under-system)
7. [Audio Tracks on VHS](#7-audio-tracks-on-vhs)
8. [Hi-Fi Audio (VHS HiFi)](#8-hi-fi-audio-vhs-hifi)
9. [The Tape: Physical Medium and Magnetic Properties](#9-the-tape-physical-medium-and-magnetic-properties)
10. [The VHS Head Drum and Helical Scan](#10-the-vhs-head-drum-and-helical-scan)
11. [Control Track and Servo Systems](#11-control-track-and-servo-systems)
12. [Playback Signal Processing](#12-playback-signal-processing)
13. [Noise, Artifacts, and Degradation](#13-noise-artifacts-and-degradation)
14. [SP vs LP vs EP Speed Modes](#14-sp-vs-lp-vs-ep-speed-modes)
15. [Sync Signals and Timing](#15-sync-signals-and-timing)
16. [Generation Loss and Dub Artifacts](#16-generation-loss-and-dub-artifacts)
17. [VCR Mechanical Transport States and Their Visual Effects](#17-vcr-mechanical-transport-states-and-their-visual-effects)
18. [Signal Summary Table](#18-signal-summary-table)

---

## 1. Overview of the VHS/VCR Signal Chain

A VCR does not record the original NTSC broadcast composite signal directly. Instead, it **separates, transforms, and re-encodes** the signal components before writing them to magnetic tape. The two primary signal components — **luminance (Y)** and **chrominance (C)** — are handled by completely different carrier systems at completely different frequencies.

### Full Signal Flow (Record)

```
NTSC Composite In (1 Vpp, 75 Ω)
        │
        ▼
  Y/C Separator
  ┌─────────────────┐
  │  Luminance (Y)  │──► Pre-emphasis ──► FM Modulator ──► High-band FM (3.4–4.4 MHz)
  │                 │
  │  Chrominance (C)│──► Frequency Converter ──► Color-Under (~629 kHz) ──► AM-encoded onto tape
  └─────────────────┘
        │
   Audio (linear)──────────────────────────────────────────────► Linear audio track (top/bottom edge)
   Audio (HiFi)   ──────► FM Modulator (1.3/1.7 MHz) ──────────► Depth-multiplexed under video
        │
        ▼
  Combined Signal ──► Record Amplifier ──► Video Heads ──► Tape
```

### Full Signal Flow (Playback)

```
Tape ──► Video Heads ──► Pre-amplifier
        │
        ├─ Hi-Pass Filter ──► FM Demodulator ──► De-emphasis ──► Luminance (Y)
        │
        ├─ Low-Pass Filter ──► Frequency Up-converter ──► Chroma decoder ──► Chrominance (C)
        │
        └─ Y + C ──► Comb filter / recombine ──► Composite out or S-Video out
```

---

## 2. NTSC Broadcast Signal Fundamentals

NTSC (National Television System Committee) is the analog color television standard used in North America. Understanding its baseline is essential before understanding how VHS modifies it.

### Frame and Line Structure

| Parameter | Value |
|---|---|
| Frame rate | 29.97 fps (technically 30000/1001) |
| Fields per frame | 2 (interlaced) |
| Field rate | 59.94 Hz |
| Total lines per frame | 525 |
| Active (visible) lines | ~480 (NTSC) |
| Blanking lines per field | ~21 |
| Horizontal line frequency | 15,734.26 Hz |
| Line period | ~63.5 µs |
| Active line time | ~52.6 µs |
| Blanking interval | ~10.9 µs |

### Interlaced Scanning

NTSC uses **2:1 interlace**. Each frame is divided into two fields:

- **Field 1 (Odd):** Scans lines 1, 3, 5, … 525
- **Field 2 (Even):** Scans lines 2, 4, 6, … 524

A new field is written to tape every ~16.68 ms (at 59.94 Hz). Each field contains approximately 262.5 lines. The half-line offset between fields is what produces the interlace; the vertical serration pulses in the sync signal manage the timing handoff between fields.

### Signal Levels (IRE Units)

IRE (Institute of Radio Engineers) units are used to measure NTSC signal amplitude:

| Level | IRE | Voltage (75 Ω) |
|---|---|---|
| Peak white | 100 IRE | ~714 mV |
| Blanking level | 0 IRE | 0 mV |
| Sync tip | -40 IRE | -286 mV |
| Black level | 7.5 IRE (NTSC setup) | ~53 mV |
| Total signal | 140 IRE | ~1.0 Vpp |

> **Note:** The 7.5 IRE "setup" (pedestal) is unique to NTSC in North America. Japan's NTSC-J uses 0 IRE black. This affects the apparent brightness floor when simulating "American" VHS vs. Japanese VHS.

---

## 3. The Composite Video Signal

The NTSC composite signal (often called CVBS — Composite Video Baseband Signal) encodes all video information onto a single waveform with the following components:

### Horizontal Sync Pulse

- Duration: **4.7 µs**
- Amplitude: **-40 IRE** (40 IRE below blanking)
- Occurs once per line at the beginning of the horizontal blanking interval
- Used by the display (or VCR) to reset the horizontal scan position

### Vertical Sync Pulse / Equalizing Pulses

- Occurs every half-field (~262.5 lines)
- Contains **pre-equalizing, vertical sync, and post-equalizing** pulses
- Vertical sync pulse width: **27.1 µs** (much wider than horizontal)
- Equalizing pulse rate: **2× line frequency** (~31.5 kHz)
- The vertical sync block spans lines 1–9 of each field

### Horizontal Blanking

- Total blanking interval: **~10.9 µs**
- Front porch: **~1.5 µs** (between active video end and sync leading edge)
- Back porch: **~4.7 µs** (after sync, before active video)
- The **color burst** is placed on the back porch

### Color Burst

- Frequency: **3.579545 MHz** (the NTSC subcarrier frequency, often rounded to 3.58 MHz)
- Amplitude: **~40 IRE** peak-to-peak
- Phase: **180° ± 0°** (reference phase for chroma demodulation)
- Duration: **8–10 cycles** per line
- Purpose: Phase and frequency reference for the chroma decoder

### Active Video / Luminance (Y)

- Occupies the ~52.6 µs active portion of each line
- Represents brightness: 7.5 IRE (black) to 100 IRE (white)
- Bandwidth on broadcast: **~4.2 MHz**

### Chrominance Subcarrier (C)

- Quadrature-modulated onto the **3.579545 MHz subcarrier**
- Two quadrature components: **I** (in-phase) and **Q** (quadrature)
  - **I channel** bandwidth: ~1.3 MHz (carries most color info)
  - **Q channel** bandwidth: ~0.5 MHz
- The I/Q axes correspond to orange-cyan and green-magenta respectively
- Alternatively expressed as **U/V** (B-Y and R-Y color difference signals)
- Hue is encoded as subcarrier **phase**, saturation as subcarrier **amplitude**

#### NTSC Subcarrier Phase Relationship

The subcarrier frequency (3.579545 MHz) is not arbitrary — it is locked to the horizontal line rate:

```
fsc = (455/2) × fH = 227.5 × 15,734.26 Hz = 3,579,545 Hz
```

The half-integer multiplier (227.5) means the subcarrier phase alternates by 180° every line. This phase alternation is what allows the **comb filter** to separate luma from chroma (they appear on opposite phases on adjacent lines).

---

## 4. VHS Recording: Signal Processing & Modulation

Before the signal reaches the tape, the VCR performs significant signal conditioning.

### Y/C Separation

The incoming composite signal is split by:

- A **low-pass filter** (cutoff ~500 kHz–3 MHz) for the chroma component
- A **high-pass filter** (cutoff ~500 kHz–3 MHz) for the luminance component

The quality of this separation is a key factor in VHS picture quality. Poor Y/C separation causes **cross-color** (chroma noise on luminance edges) and **cross-luminance** (luma noise on color areas).

### Luminance Pre-Emphasis (Record)

Before FM modulation, the luminance signal is passed through a **pre-emphasis filter**. This is a high-frequency boost (shelving filter) that lifts the high-frequency luma components. On playback, a matching **de-emphasis** curve is applied, which suppresses high-frequency tape noise and noise introduced by the FM demodulator. The net result is improved SNR at high luma frequencies.

VHS uses a **modified 75 µs pre-emphasis** curve (similar to but not identical to broadcast). The filter starts boosting at roughly **500 kHz** and continues beyond **2 MHz**.

### Sync-Tip Clamp and White Clip

- The sync tip is clamped to a fixed level before modulation (sets carrier rest frequency)
- White peaks are **clipped at approximately 100–110 IRE** to prevent over-deviation of the FM carrier
- Over-deviation on tape causes **white smearing** artifacts on playback (the carrier can't be perfectly recovered)

---

## 5. The VHS FM Luminance Carrier

VHS encodes luminance using **Frequency Modulation (FM)**. This was chosen because FM is relatively immune to amplitude variations — tape dropout and non-linear amplitude response have far less impact on FM than on AM.

### VHS FM Deviation Map (NTSC SP)

| Video Level | Carrier Frequency |
|---|---|
| Sync tip (-40 IRE) | **3.400 MHz** |
| Blanking / Black (0 IRE) | **3.800 MHz** (approx.) |
| Black level (7.5 IRE) | ~3.854 MHz |
| White peak (100 IRE) | **4.400 MHz** |

- **Total FM deviation:** 1.0 MHz (3.4 → 4.4 MHz)
- **Deviation per IRE:** ~7.1 kHz / IRE
- **Carrier rest frequency** (at sync tip): 3.4 MHz

> In practice the instantaneous frequency varies continuously as the video signal changes, sweeping between 3.4 and 4.4 MHz 15,734 times per second (once per line).

### FM Bandwidth on Tape

The FM luminance carrier and its sidebands occupy a band from approximately **2 MHz to 6 MHz** on tape. Because VHS tape has limited high-frequency response, this limits horizontal resolution.

**VHS luma bandwidth** is approximately **3 MHz** (usable), corresponding to roughly **240 lines of horizontal resolution** (versus ~330 for S-VHS or ~480 for broadcast). The 3 dB point of the FM carrier is around 4.4 MHz; sidebands above ~5.5 MHz are significantly attenuated by tape dropout and head response.

---

## 6. Chroma Signal Handling: The Color-Under System

The chrominance signal cannot be recorded at its original 3.58 MHz frequency because it would directly overlap with the FM luminance carrier. VHS solves this with the **color-under** (or down-converted chroma) technique.

### Frequency Down-Conversion

The 3.58 MHz NTSC chroma subcarrier is heterodyned (mixed) down to a much lower frequency:

```
fchroma_tape = 629 kHz  (VHS NTSC)
```

The conversion uses a reference oscillator and mixer. The exact conversion formula is:

```
fconvert = 40 × fH = 40 × 15,734.26 = 629,370 Hz ≈ 629 kHz
```

The down-converted chroma signal occupies roughly **300 kHz to 1.2 MHz** on tape — well below the FM luminance band.

### Phase Correction: The Heterodyne Process

The most critical aspect of the color-under system is **phase stability**. The original 3.58 MHz subcarrier must be perfectly phase-coherent with the color burst to decode correctly. However, any **tape speed variation (jitter, wow, flutter)** will introduce phase errors in the recovered chroma, causing **hue shifts**.

VHS (and most consumer VCRs) addresses this with an **Automatic Phase Control (APC)** loop and a **color killer** circuit during playback. The down-converted chroma is upconverted back to 3.58 MHz using the head drum's rotation-based reference, and the burst phase is used to lock the oscillator.

### Color Phase Alternation (PAL vs NTSC)

In **NTSC**, the chroma subcarrier's phase relationship is maintained line-to-line. This is more vulnerable to phase errors than PAL (which reverses the V component each line to cancel phase errors). On VHS, this means NTSC chroma is particularly sensitive to tape head-switching noise at the field boundary.

### Color-Under Artifacts

The color-under system introduces several characteristic artifacts:

- **Chroma noise:** The low-frequency color signal has poorer SNR than the FM luminance
- **Chroma smear / low chroma bandwidth:** VHS limits chroma horizontal bandwidth to ~500 kHz (after upconversion), corresponding to roughly **40 lines** of color resolution — much worse than luma
- **Hue instability:** Small tape speed variations produce subcarrier phase jitter → hue wobble
- **Color fringing:** On sharp transitions, the slow color response creates a visible horizontal color smear to the right of colored objects

---

## 7. Audio Tracks on VHS

### Linear Audio Tracks

VHS uses two **stationary (linear) audio tracks** recorded along the edges of the tape by fixed heads:

| Track | Position |
|---|---|
| CH1 (Left) | Top edge of tape |
| CH2 (Right) | Bottom edge of tape |

Track width: approximately **1.0 mm** in SP mode  
Track width in LP: approximately **0.5 mm**  
Track width in EP: approximately **0.33 mm**

#### Linear Audio Characteristics

- Recorded using **AC bias** (standard magnetic recording technique)
- Frequency response: **50 Hz – 10 kHz** (approximate, varies by VCR)
- Dynamic range: ~**40–45 dB** (SNR referenced to peak level)
- Wow and flutter: Follows tape transport speed variation (~0.1–0.3% wow)
- The audio is typically delayed relative to the video on the tape to account for the physical distance between the audio head and video head drum

#### Linear Audio Head Position

The **erase head, linear audio head, and control track head** are stationary. The physical offset between the audio head and the video drum means there is a built-in **audio/video sync offset** of approximately 16–20 video frames (depending on speed mode), which the VCR compensates for internally during playback.

---

## 8. Hi-Fi Audio (VHS HiFi)

Introduced around 1984, **VHS HiFi** dramatically improved audio quality by recording audio via the **rotating video heads**, depth-multiplexed *below* the video signal on the tape using FM.

### HiFi FM Carrier Frequencies (NTSC)

| Channel | Carrier Frequency |
|---|---|
| Left (CH1) | **1.3 MHz** |
| Right (CH2) | **1.7 MHz** |

- **Deviation:** ±150 kHz (±100 kHz typical program level)
- **Frequency response:** 20 Hz – 20,000 Hz (±1 dB)
- **Dynamic range:** ~**90 dB** (with Dolby, or 80 dB without)
- **Wow and flutter:** < 0.005% WRMS (essentially inaudible; it tracks with the video)
- **SNR:** ~80–90 dB (A-weighted)

### Depth Multiplexing

HiFi audio and video occupy the **same physical tape track** but at different magnetic depths:

- **HiFi audio** is recorded first (deeper in the magnetic layer) at higher write current
- **Video (FM luma + color-under chroma)** is recorded on top, partially overwriting the upper layer but leaving the deep HiFi layer intact
- On playback, **HiFi heads** (which have a different azimuth angle and gap depth) read the deep layer, while **video heads** read only the shallow video layer

The HiFi carriers at 1.3 and 1.7 MHz sit below the color-under chroma (~629 kHz–1.2 MHz) and below the FM video carrier (3.4–4.4 MHz). The overlapping frequency region with chroma is managed by filtering during record and play.

---

## 9. The Tape: Physical Medium and Magnetic Properties

### Tape Construction

VHS tape is a **polyester base film** coated with a magnetic oxide layer:

| Layer | Material | Thickness |
|---|---|---|
| Base film | Polyester (PET) | ~14–19 µm |
| Magnetic layer | γ-Fe₂O₃ (iron oxide), Cr₂O₃, or Metal | ~4–6 µm |
| Back coating | Carbon-loaded compound | ~1 µm |

- **Tape width:** 12.65 mm (0.498 inches)
- **Tape thickness (total):** ~20–25 µm depending on formulation

### Magnetic Coercivity

**Coercivity** (Hc) determines how hard the magnetic particles are to magnetize and demagnetize — a higher coercivity means lower noise but requires more record current:

| Formulation | Coercivity | Notes |
|---|---|---|
| Standard (Type I, γ-Fe₂O₃) | ~300–600 Oe | Original VHS tape |
| High-grade (HG) | ~600–800 Oe | Improved SNR |
| Metal particle (MP) | ~1000–1500 Oe | S-VHS required |
| Metal evaporated (ME) | ~1500–2000 Oe | Camcorder formats |

### Retentivity and Remanence

**Remanence (Br)** is the residual magnetization after the recording field is removed. Higher Br means stronger playback signal.

### Tape Speed (SP Mode NTSC)

- **Tape speed:** 3.335 cm/s (SP), 1.668 cm/s (LP), 1.112 cm/s (EP/SLP)
- **Head-to-tape speed** (helical scan): approximately **5.8 m/s**
- This high relative speed is what allows the high-frequency FM video to be recorded despite the slow physical tape motion

### Oxide Particle Size and HF Response

Smaller oxide particles enable recording and playback of shorter wavelengths (higher frequencies). The maximum frequency recordable is determined by:

```
fmax ≈ v / (2 × gap)
```

Where `v` is head-to-tape speed and `gap` is the head gap length. VHS heads typically have gap lengths of **~0.3–0.6 µm** for video.

---

## 10. The VHS Head Drum and Helical Scan

### Drum Geometry

VHS uses a **upper/lower drum assembly** with a rotating upper drum:

- **Drum diameter:** 62 mm
- **Drum rotation speed:** 1,800 RPM (30 fields/sec × 60 sec = 1,800 RPM)
- **Head-to-tape contact arc:** ~180° (each head scans half the drum circumference per pass)
- **Wrap angle of tape:** ~180° around the drum
- **Track angle:** approximately **5.96°** to the tape edge (helical angle)

### Video Heads

- **Number of heads:** 2 (standard); 4 in many VCRs (2 for SP + 2 for LP, or 2 standard + 2 for special effects)
- **Head gap:** ~0.3–0.5 µm for standard VHS video
- **Azimuth angle:** ±6° (alternating between the two heads — see below)

### Azimuth Recording

One of the most important techniques in VHS is **azimuth recording**. The two video heads have their gaps angled in **opposite directions** (±6° from perpendicular to the track). This means:

- When Head A reads its own track, the ±6° tilt gives maximum output
- When Head A inadvertently reads the adjacent track (recorded by Head B at -6°), the 12° total azimuth difference causes destructive cancellation of the high-frequency signal

This "azimuth loss" is so severe at video frequencies that **guard bands are unnecessary** — adjacent tracks can be laid directly next to each other with no gap, maximizing tape utilization.

**Azimuth loss formula:**
```
L = sinc(π × d × sin(Δθ) / λ)
```

Where `d` is track width, `Δθ` is azimuth difference, and `λ` is the recorded wavelength. At VHS frequencies, this provides > 30 dB of adjacent-track rejection.

### Head Switching

The two heads alternate on opposite sides of the drum. As the tape wraps ~180°, **Head A scans from top to bottom** of the tape (recording one field), then **Head B scans** (recording the next field). The transition between heads occurs during the **vertical blanking interval**.

The **head switch point** is timed to occur approximately **6–10 lines before** the active vertical sync pulse, well within the blanking period. If head switching timing is off, a **horizontal banding artifact** appears near the bottom of the picture (the "head switching noise band").

---

## 11. Control Track and Servo Systems

### Control Track

A dedicated **control track** (CTL) is recorded on the bottom edge of the tape by a stationary control head. It contains:

- **One pulse per field** (59.94 Hz)
- Generated by the head drum rotation (once per revolution, 30 Hz — but since there are two heads per revolution, it's effectively 60 Hz field rate)
- Acts as a **synchronization reference** for playback

The CTL pulse is a simple squarewave burst at 59.94 Hz. On the tape it appears as a series of short magnetic flux transitions.

### Drum Servo

During playback, the **drum servo** locks the head drum rotation to:
1. The incoming reference signal (from the CTL track), and
2. An external sync signal (from the video source, or an internal crystal reference)

This ensures the video heads hit the correct tracks on the tape.

### Capstan Servo

The **capstan servo** controls tape transport speed. During playback:

1. It reads the CTL pulses from tape
2. Compares their timing to a reference (crystal or external sync)
3. Adjusts capstan motor speed to keep CTL pulses phase-locked to the reference

If the CTL track is damaged, the capstan servo loses reference, resulting in **severe picture instability or loss of sync**.

### Tracking Adjustment

The **tracking control** on a VCR adjusts the phase offset between the CTL pulse and the head position. Mis-tracking causes the video heads to read the wrong tracks, introducing **noise bars** (horizontal bands of video noise) into the picture. Manual tracking is required when:

- Playing a tape recorded on a different VCR with slight mechanical differences
- The tape has stretched or shrunk
- The CTL track reference is off

---

## 12. Playback Signal Processing

### Pre-Amplification

The signal from the video heads is extremely small (on the order of **microvolts to millivolts**). A **high-gain, low-noise preamplifier** boosts the signal before any processing.

### Dropout Compensation (DOC)

A **dropout** is a momentary loss of signal caused by a missing or damaged oxide patch on the tape. During playback, the AGC detects a rapid signal level drop and:

1. Detects the dropout (signal below a threshold for > ~2–4 µs)
2. **Substitutes the signal from the previous scan line** (stored in a 1H delay line)
3. This is called **dropout compensation** and conceals small dropouts as a slight smear rather than a white or black flash

Without DOC, dropouts appear as bright white horizontal streaks (the head loses the FM carrier and the demodulator defaults to maximum or minimum deviation).

### FM Demodulation and De-emphasis

The recovered FM luminance signal is:
1. Band-pass filtered to isolate the 3.4–4.4 MHz carrier
2. Passed through a **limiter** (to remove AM variations)
3. Demodulated (FM discriminator or PLL)
4. Passed through **de-emphasis** (inverse of the record pre-emphasis curve)
5. **Sync tip clamped** back to 0 IRE reference

### Chroma Upconversion

The 629 kHz color-under signal is:
1. Filtered to isolate the chroma band (~300 kHz – 1.5 MHz)
2. Heterodyned back up to **3.58 MHz** using the drum reference oscillator
3. Phase-corrected via the APC loop (using the color burst as reference)
4. Recombined with the demodulated luminance

### Comb Filtering (Luma/Chroma Separation)

Some higher-end VCRs include a **comb filter** in playback to improve Y/C separation of the output composite signal. This uses the fact that the subcarrier on adjacent lines is phase-inverted, allowing cancellation of chroma in the luma path and vice versa. This reduces **cross-color** and **dot crawl** on the composite output.

---

## 13. Noise, Artifacts, and Degradation

This section catalogs the specific visual and audio defects that define the "VHS look."

### Luminance (Luma) Noise

- **Source:** Thermal noise in tape oxide, head electronics, FM demodulator
- **Appearance:** Fine grain pattern, predominantly on uniform areas
- **Character:** Spatially uncorrelated; follows the scan lines (horizontal streaks at a fine scale)
- **Frequency:** Broadband, but concentrated at higher luma frequencies (above 1 MHz)
- **Amplitude (typical SP):** ~40–45 dB SNR (luma)

### Chroma Noise

- **Source:** The down-converted color-under signal has poor SNR due to low carrier frequency
- **Appearance:** Colored speckle/grain, most visible in saturated color areas
- **Character:** Slower temporal variation than luma noise; tends to appear as colored shimmer
- **SNR:** ~35–40 dB (chroma) — noticeably worse than luma

### Horizontal Resolution Limit (Luma Smear)

- VHS SP horizontal resolution: **~240 TVL** (TV lines)
- High-frequency detail beyond ~3 MHz is rolled off by the tape/head response
- Appears as a subtle softening/blurring of fine horizontal detail
- Fine diagonal lines exhibit **Moiré patterns** near the resolution limit

### Chroma Smear (Low Color Bandwidth)

- Effective chroma bandwidth after upconversion: **~500–800 kHz** (equivalent to ~40 TVL)
- Sharp color transitions have a **right-side trailing smear** several pixels wide
- High-contrast color edges (red on black, for example) show the most visible smear

### Head Switching Noise Bar

- Occurs at the transition between the two video heads (~6 lines before vertical sync)
- Manifests as a thin horizontal band of noise (usually 2–6 scan lines wide)
- Positioned at the **very bottom** of the active picture (or hidden in overscan)
- More visible on screens with less overscan
- Timing can shift slightly with tape tracking, moving the bar up or down

### Tracking Noise Bars

- Caused by misaligned tracking (head reads between tracks)
- Multiple horizontal noise bands, often brighter or differently textured than the picture
- Move up or down with tracking adjustment
- Can be severe enough to disrupt sync

### Dropout Artifacts

- **Pre-compensation failure:** Tiny white or dark horizontal flashes (1–10 µs wide)
- **DOC streaks:** When DOC activates, visible as a faint horizontal smear (correct color from previous line substituted — sometimes noticeable if scene changes rapidly)
- **Prolonged dropouts:** Result in sync loss, picture roll, color loss

### Wow and Flutter

- **Wow:** Low-frequency speed variations (< 2 Hz) causing slow undulation of image
- **Flutter:** Higher-frequency speed variations (2–100 Hz) causing rapid geometric distortion
- **VHS specification:** < 0.05% WRMS (weighted wow and flutter)
- In chroma: manifests as slow hue drift (since chroma is phase-encoded)
- In audio: pitch wavering

### Luminance Ringing / Edge Enhance

Many VCRs include an **aperture correction** circuit that sharpens horizontal edges by adding a differentiated (peaking) component. This can introduce:

- **Pre-shoot:** A brief dark band just before a bright edge
- **Post-ring:** One or more bright/dark oscillations after the edge (most commonly: one bright "ghost" to the right of a white-on-black edge)

### Temporal Smear / Low-Pass Filtering

The FM luminance response rolls off above ~3 MHz, which adds a characteristic temporal (horizontal) blur to the image. This is often described as the picture looking "soft" compared to broadcast.

### Noise Rolloff at Head Gap

At the maximum recorded frequency (near the white-peak carrier of 4.4 MHz), the tape/head combination attenuates the signal. This causes **luminance compression at peak whites**, slightly reducing high-frequency content in bright areas more than in dark areas.

### Color Fringing / Hue Errors at Transitions

Because the color-under system has limited bandwidth, sharp luminance transitions can "bleed" into the chroma path and vice versa. This creates colored fringes around high-contrast black/white edges.

### Video "Breathing" / AGC Pumping

VCRs use **AGC (Automatic Gain Control)** on playback to compensate for head output variations. When a dark (low-signal) section of tape is followed by a bright section, the AGC may be briefly over-corrected, causing a momentary flash or luminance step.

### Macrovision (Copy Protection)

Tapes with **Macrovision** anti-copy protection have modified AGC-disrupting signals added to the vertical blanking interval:
- **Pulse Anti-Recording (PAR):** High-amplitude pulses during the VBI disrupt AGC on recording VCRs
- **Colorstripe (CS):** Phase-reversed burst added to the picture area confuses chroma decoders
- Playback on a TV is unaffected; recording to a second VCR produces a rolling, unstable picture

---

## 14. SP vs LP vs EP Speed Modes

VHS offers three tape speed modes, all using the same head drum speed (1,800 RPM). Speed reduction is achieved by slowing only the **capstan/tape speed**.

| Mode | Tape Speed | Track Pitch | Play Time (T-120) | Luma SNR | Chroma SNR | HRes |
|---|---|---|---|---|---|---|
| SP (Standard Play) | 3.335 cm/s | ~58 µm | 2 hours | Best | Best | ~240 TVL |
| LP (Long Play) | 1.668 cm/s | ~29 µm | 4 hours | Moderate | Moderate | ~230 TVL |
| EP/SLP (Extended Play) | 1.112 cm/s | ~19 µm | 6 hours | Worst | Worst | ~210 TVL |

### Speed Mode Effects on Signal

**Slower tape speed** means:

- **Narrower tracks** → more susceptible to tracking error noise
- **Higher noise** → reduced oxide per unit of head scan, fewer magnetic particles contributing to signal
- **Reduced guard band** (effectively zero in all modes due to azimuth) → more adjacent track crosstalk at lower azimuth loss frequencies
- **Worse dropout performance** → smaller physical defects cause larger relative dropouts

**FM deviation** remains identical across speed modes — the carrier frequencies (3.4–4.4 MHz) do not change. Only the physical track geometry changes.

### LP Compatibility Issues

Tapes recorded at LP on one VCR may exhibit **tracking errors** when played on another, because the narrower track pitch magnifies any mechanical tolerance differences. LP recordings also suffer more from **head clog artifacts** than SP.

---

## 15. Sync Signals and Timing

### Horizontal Sync in VHS Context

The VCR must internally regenerate sync signals because the recovered signal from tape has jitter. The **sync separator** extracts sync from the composite signal, and a **sync regenerator / TBC** (Time Base Corrector) creates clean sync pulses phase-locked to a crystal reference.

Consumer VCRs have **no TBC** — they output raw, jittery video. Professional VCRs include full TBC circuits. The absence of TBC is the primary reason VHS output causes instability on high-end monitors.

### Horizontal Sync Timing Accuracy

On a typical consumer VCR playback:
- Horizontal jitter: **~100–500 ns** peak-to-peak (varies with tape condition and VCR quality)
- This jitter causes visible **horizontal line displacement** (the "wobbly edge" effect on VHS)
- Fast-moving objects near edges of the picture show this most visibly

### Vertical Sync and Field Lock

The drum servo normally locks the vertical sync to a reference, but in the absence of an external reference (when playing standalone), it free-runs at the crystal frequency (~29.97 Hz). This is why VCRs output stable vertical sync even on dirty or damaged tapes, as long as the CTL track is intact.

### Color Subcarrier Regeneration

During playback, the **3.579545 MHz color subcarrier** is not directly recovered from the tape (the down-converted chroma has been at 629 kHz). Instead, it is **regenerated** by a crystal oscillator in the VCR. The chroma upconversion process phase-locks the regenerated subcarrier to the recovered color burst.

In the absence of a proper color burst (severely damaged tape, or non-color recording), the chroma output is suppressed by the **color killer** circuit.

---

## 16. Generation Loss and Dub Artifacts

Each time a VHS tape is duplicated (dub), the signal is decoded and re-encoded, accumulating noise and artifacts.

### Generation 1 → Generation 2 Losses

- **Luma SNR:** Decreases ~4–6 dB per generation
- **Chroma SNR:** Decreases ~3–5 dB per generation
- **Chroma phase errors** accumulate (each re-encode introduces new phase jitter)
- **Edge ringing** increases (aperture correction adds overshoot which gets re-encoded)
- **Sync quality** degrades (jitter accumulates)

### Generational Artifacts

| Artifact | Appearance |
|---|---|
| Increased luma noise | More visible grain in all areas |
| Chroma smear | Colors increasingly blur rightward |
| White ringing | Brighter pre/post-ringing on white edges |
| Color shift | Overall hue may drift (each dub introduces small phase bias) |
| Black level drift | Sync tips and black level may shift slightly per generation |
| Sync instability | Picture roll or H-sync jitter increases |

By the **3rd–4th generation**, VHS video is recognizably degraded. By the **5th–7th generation**, horizontal resolution drops below 200 TVL and chroma noise is severe.

### Dub of a Macrovision-Protected Tape

Attempting to dub a Macrovision-protected tape produces:
- Intermittent **picture brighten/darken cycling** (AGC pumping)
- Color saturation changes
- Possible vertical sync instability (picture roll)

---

## 17. VCR Mechanical Transport States and Their Visual Effects

### Play

Normal playback state. Heads scan tape at correct speed. Picture is stable (assuming good tape and tracking).

### Pause / Still

Tape stops moving. Both video heads continue rotating and **repeatedly scan the same two tracks**. Because the two tracks were recorded by heads with opposite azimuth, the still image shows a characteristic **noise band** (or two bands) at the point where the head crosses from one track to its neighbor. In many VCRs, pause mode uses a special "still" head to minimize this.

### Fast-Forward / Rewind

Tape moves rapidly in one direction. Heads may or may not contact the tape. In **high-speed shuttle**, a partial video signal can be obtained — this is the basis of "high-speed search" (picture search mode).

### Picture Search (Cue / Review)

Tape moves at 5×–9× normal speed. The heads scan the tape at an angle that crosses multiple tracks per pass. The result is:
- Multiple horizontal noise bars (as the head crosses many tracks)
- A visible (though noise-barred) image can be seen
- The image appears compressed vertically (each head pass covers more than one frame)

### Slow Motion

Tape moves at a fraction of normal speed (e.g., 1/5 speed). The drum still runs at 1,800 RPM. Each track is scanned multiple times, producing a **repeated frame** effect interspersed with noise bars between valid track scans.

### Record

The erase head runs ahead of the video drum, AC-erasing the tape before the video heads re-record it.

---

## 18. Signal Summary Table

| Signal | Frequency / Band | Encoding | Notes |
|---|---|---|---|
| FM Luminance (white peak) | 4.400 MHz | FM carrier | Sync tip at 3.4 MHz |
| FM Luminance (sync tip) | 3.400 MHz | FM carrier | Rest (unmodulated) frequency |
| FM Luminance sidebands | ~2.0 – 6.0 MHz | FM sidebands | Bandwidth limited by tape |
| Color-Under Chroma | ~629 kHz (center) | Down-conv. AM | Original 3.58 MHz → 629 kHz |
| Chroma band (on tape) | ~300 kHz – 1.2 MHz | Heterodyned chroma | Includes sidebands |
| HiFi Audio Left | 1.3 MHz | FM carrier | ±150 kHz deviation |
| HiFi Audio Right | 1.7 MHz | FM carrier | ±150 kHz deviation |
| Linear Audio CH1 | DC – 10 kHz | AC bias magnetic | Top edge of tape |
| Linear Audio CH2 | DC – 10 kHz | AC bias magnetic | Bottom edge of tape |
| Control Track | 29.97 / 59.94 Hz | Flux pulses | Bottom edge, stationary head |
| NTSC Color Subcarrier | 3.579545 MHz | QAM on composite | Regenerated on playback |
| Horizontal line rate | 15,734.26 Hz | Sync pulses | 525 lines / 29.97 fps |
| Vertical field rate | 59.94 Hz | Sync pulse | Head drum locked to this |

---

## Appendix A: Key Constants for NTSC VHS Simulation

```
NTSC frame rate:           29.97002997... fps  (30000/1001)
NTSC field rate:           59.94005994... Hz   (60000/1001)
Horizontal line rate:      15,734.264 Hz
Color subcarrier:          3,579,545 Hz        (455/2 × fH)
Total lines per frame:     525
Active lines per frame:    480 (approximately)
Blanking lines per field:  ~21

VHS FM carrier (sync tip): 3,400,000 Hz
VHS FM carrier (white):    4,400,000 Hz
VHS FM deviation total:    1,000,000 Hz (1 MHz)
VHS FM deviation/IRE:      ~7,143 Hz/IRE

VHS color-under frequency: 629,370 Hz          (40 × fH)
VHS HiFi left carrier:     1,300,000 Hz
VHS HiFi right carrier:    1,700,000 Hz

Head drum diameter:        62 mm
Head drum RPM:             1,800 RPM (SP/LP/EP)
Head-to-tape speed:        ~5.8 m/s
Azimuth angle:             ±6° (±6 from perpendicular, 12° total between heads)
Track pitch (SP):          ~58 µm
Track pitch (LP):          ~29 µm
Track pitch (EP):          ~19 µm
Tape speed (SP):           3.335 cm/s
Tape speed (LP):           1.668 cm/s
Tape speed (EP):           1.112 cm/s
Tape width:                12.65 mm

Horizontal resolution (SP): ~240 TVL
Chroma bandwidth:           ~500 kHz (~40 TVL equivalent)
Luma SNR (SP, typical):    ~44 dB
Chroma SNR (SP, typical):  ~38 dB
```

---

## Appendix B: Simulation Implementation Notes

### Simulating the FM Luminance Response

To accurately simulate VHS luma:

1. **Soft-clip** the input luma at approximately 100–110 IRE (white clipping)
2. Apply a **pre-emphasis shelving boost** starting around 500 kHz
3. **Limit horizontal bandwidth** to ~3 MHz (apply a Gaussian or Butterworth LPF)
4. Add **FM-characteristic noise**: predominately high-frequency, slightly correlated along scan lines
5. Simulate **head frequency response rolloff** above 3.5 MHz

### Simulating Chroma

1. **Bandwidth-limit chroma** to ~500 kHz (strong LPF on the I/Q or U/V channels)
2. Add **chroma noise** (lower SNR than luma, colored speckle)
3. Apply a slight **hue wobble** (low-frequency sinusoidal phase variation, ~0.5–2 Hz, ~±2–5°)
4. Add **chroma-to-luma bleed** at sharp transitions (smear colored pixels slightly rightward)

### Simulating Composite Artifacts

1. **Dot crawl**: The color subcarrier dots crawl horizontally at the luma/chroma boundary — especially visible on fine horizontal stripes or text
2. **Cross-color**: Fine luma patterns near 3.58 MHz get mistakenly decoded as chroma — add color speckle on fine high-frequency luma patterns
3. **Ringing**: Apply a slight overshoot on sharp horizontal edges (FIR with a mild resonance)

### Simulating Tape Noise and Dropout

1. **Grain/noise texture**: Apply a per-frame noise layer; use a horizontal-directional noise (stronger in the scan direction)
2. **Dropouts**: Occasional brief horizontal white or black flashes, ~2–20 µs wide, replaced in the next line (DOC)
3. **Head switching artifact**: A 2–6 line wide noise band near the bottom of the picture (approximately lines 477–483)

### Simulating Tracking Error

1. One or more **horizontal noise bars**, moving vertically when tracking is incorrect
2. Bars have a different luminance texture (salt-and-pepper or banded noise)
3. At severe mistrack, bars expand to fill most of the picture

### Simulating Temporal Effects

1. **Wow**: Apply a slow (0.5–2 Hz) sinusoidal horizontal position displacement of ~±2–4 pixels
2. **Flutter**: Apply a faster (5–20 Hz) displacement of ~±1 pixel with higher frequency
3. Both affect chroma phase as well (hue shifts follow the same modulation)

---

## 19. Tape and VCR Damage: Signal Effects and Simulation Methods

This section catalogs physical damage conditions — to the tape itself or to the VCR mechanism — and describes exactly how each manifests in the signal and how to implement the effect in a simulator.

---

### 19.1 Tape Oxide Shedding / Head Clog

#### Physical Cause
Oxide particles flake off the tape backing and accumulate in the video head gap. Even a small amount of debris partially or fully blocks the gap, reducing or eliminating signal output from one or both heads.

#### Signal Effect
- Gradual or sudden **reduction in playback amplitude** (partial clog) or complete loss (full clog)
- Affects one head at a time: since the two heads alternate fields, a single-head clog causes every other field to drop out → **alternating fields of noise and valid video**
- The FM carrier is lost or severely attenuated → the demodulator produces **maximum deviation noise** (random white/black flicker) or defaults to a fixed carrier frequency (producing a gray or black field)
- Chroma is lost simultaneously (no carrier to upconvert from)
- Linear audio is usually unaffected (separate stationary head)

#### Severity Progression
| Stage | Effect |
|---|---|
| Light clog | Slightly increased luma noise, occasional dropout flashes |
| Moderate clog | Repeated horizontal noise bands on one alternating field |
| Heavy clog | Every other field is full-screen noise or near-black |
| Full clog | Complete loss of video on one or both heads; solid noise or black |

#### Simulation Method
1. Define a `clog_level` parameter (0.0 = clean, 1.0 = fully clogged) per head (head A and head B independently)
2. For each field, attenuate the recovered signal amplitude: `signal *= (1.0 - clog_level)`
3. Below an amplitude threshold (~0.2), switch the FM demodulator into **free-run mode**: replace the demodulated output with broadband noise (uniform distribution, full luma range)
4. For partial clog: add burst-mode amplitude dropout events — random regions of a scan line where signal drops to near-zero for 5–30 µs, triggering DOC (substitute from previous line)
5. For alternating field clog: apply full attenuation only to fields scanned by the affected head (every other field)
6. Optionally: **gradually worsen** the clog over time (increase `clog_level` at a slow rate per frame to simulate oxide buildup)

---

### 19.2 Tape Crease / Fold Damage

#### Physical Cause
A physical crease, fold, or wrinkle in the tape base film causes the oxide surface to contact the head at the wrong angle or to lose contact entirely as it passes over the drum.

#### Signal Effect
- A **vertical stripe** of dropout spanning multiple consecutive scan lines (the crease runs perpendicular to the scan direction)
- Unlike point dropouts, a crease produces a **wide vertical band** of damage that repeats on the same horizontal position every pass
- If the fold is sharp, the oxide on the fold edge may be cracked away entirely → **permanent vertical stripe** of missing signal
- The stripe width corresponds to the time the creased section takes to pass the head gap (creases of ~1–5 mm → stripe ~50–300 µs wide at SP speed)
- A severe crease can also damage the head itself (see 19.7)

#### Simulation Method
1. Define `crease_x_start` and `crease_x_end` (horizontal pixel positions) and `crease_severity` (0–1)
2. On every line, within that horizontal range:
   - Attenuate luma and chroma toward zero (severity 1 = full dropout)
   - Apply DOC (substitute from previous line) — producing a **smeared/frozen horizontal stripe** in that column
3. Add slight **edge fringing** just outside the crease boundaries (a few pixels of extra ringing/noise) from the abrupt head-tape contact change
4. The crease position is **stable frame to frame** (it's a physical tape feature), but may shift slightly on rewind/replay (the tape sits slightly differently each pass) — add ±2–5 pixel jitter to `crease_x_start` per play pass
5. For a fold (worse than a crease): apply full white flash (FM carrier lost, demodulator max output) rather than black dropout

---

### 19.3 Tape Stretch / Elongation

#### Physical Cause
Tape that has been stored under tension, wound too tightly, or pulled by a malfunctioning transport will stretch longitudinally. Polyester base film stretches without fully recovering.

#### Signal Effect
- **CTL pulse spacing is elongated** → the capstan servo, locked to the original CTL rate, must run slightly faster to play back at the correct speed → the effective track pitch read by the heads is slightly off
- Results in **mild to moderate mistrack** (heads read between tracks) → noise bars
- **Audio pitch is lowered** (the linear audio was recorded with stretched tape; on a non-stretched transport, it plays back at slightly faster speed relative to audio reference, but the elongation means audio is slightly low)
- Horizontal geometry is slightly **compressed** (the image appears very slightly narrower than normal if the stretch is large)
- Stretch is usually **non-uniform** — certain sections of the tape are more stretched → intermittent tracking problems (noise bars that come and go)

#### Simulation Method
1. Define a `stretch_map` — a per-frame (or per-field) array of stretch factors, e.g., sinusoidal or random low-frequency variation: `stretch[f] = 1.0 + stretch_amount * noise_LFO(f)`
2. Apply horizontal scaling to each line by `stretch[f]`: stretch > 1.0 compresses the image slightly, < 1.0 expands it
3. Modulate the CTL phase offset proportional to cumulative stretch → drives tracking offset → apply tracking noise bars (see 19.4)
4. For audio: apply a matching pitch shift of `1 / stretch[f]` (stretched tape plays back slightly fast → pitch drops slightly if stretch occurred during record)
5. Add **intermittent noise bars** at stretched sections (frames where `stretch > threshold`)

---

### 19.4 Mistrack / Tracking Error (Mechanical or Tape Damage)

#### Physical Cause
- Worn capstan pinch roller (rubber hardens with age, loses grip)
- Worn or dirty capstan shaft
- Damaged CTL track (oxide worn off at edge)
- Playing a tape recorded on a VCR with different mechanical calibration
- Tape shrinkage or stretch (see 19.3)
- Foreign debris on the tape path guides

#### Signal Effect
- When the head reads **between two adjacent tracks** (partial overlap with both):
  - Signal from both tracks is mixed; since they have opposite azimuth (±6°), the result is **high-frequency luma cancellation** in the mixed zone
  - The read signal is at **lower amplitude** → increased noise
  - One or more **horizontal noise bands** appear in the picture; their vertical position corresponds to where on the head scan the tracking error begins
- **Severe mistrack**: The head reads entirely on the wrong track → heavy noise throughout the field scanned by that head; the field recorded by the other head may be clean → alternating clean/noisy fields

#### Noise Bar Characteristics
- Width: 2–30 lines depending on severity
- Texture: salt-and-pepper noise with slight horizontal streaking
- Position: determined by phase of CTL offset
  - CTL leading by X µs → noise bar shifts toward the bottom of picture
  - CTL lagging by X µs → noise bar shifts toward the top
- Movement: if the tracking error drifts slowly, the noise bar moves up or down at a rate corresponding to the CTL drift rate

#### Simulation Method
1. Compute a `tracking_offset` value (in µs or fractional track widths): 0 = perfect, ±1 = one full track off
2. For each scan line, calculate **how far the head is from the center of its assigned track**:
   `overlap = |tracking_offset| / track_pitch`
3. Apply a noise blend on affected lines:
   `output_luma[y] = lerp(clean_luma[y], noise[y], overlap_fraction[y])`
4. At the transition point (where the head crosses from one track to another), apply full-noise for 2–4 lines
5. Optionally animate `tracking_offset` as a slow LFO (0.1–1 Hz) to simulate a worn pinch roller that can't hold lock

---

### 19.5 Tape Humidity Damage (Sticky Shed Syndrome)

#### Physical Cause
Certain tape formulations (primarily those using polyester-urethane binders, common in tapes from the 1970s–1980s) absorb moisture over time. The binder breaks down (hydrolysis), causing the oxide layer to become **sticky and soft**. When the tape is played, the oxide sheds onto the heads and guides rapidly.

#### Signal Effect
- **Rapid, severe head clog** within seconds to a few minutes of play
- Audio and video degrade simultaneously
- Characteristics: initially looks like gradual clog (see 19.1), then quickly escalates to complete signal loss
- A **high-pitched squeal** from the audio output is common as the sticky tape drags against the guides
- After the tape stops, the heads are coated with shed oxide → subsequent tape plays may also be affected
- The tape may **stick to the drum** momentarily, causing a brief complete freeze of the image

#### Simulation Method
1. Model as a **time-varying clog** that accelerates: `clog_level(t) = min(1.0, (t / T_clog)^2)` where T_clog might be 30–120 seconds of play time
2. Add **intermittent full-frame stalls**: every N seconds, freeze the video for 1–3 frames (tape stuck to drum)
3. Add a **broad-spectrum audio noise component** (the squeal) that increases in amplitude with clog level: high-frequency (4–8 kHz) sine wave with amplitude proportional to `clog_level`
4. After a full clog event, subsequent play of other tapes: apply a residual moderate clog that clears gradually (over 10–20 seconds of play, simulating head self-cleaning)

---

### 19.6 Tape Cinching / Slack Wind

#### Physical Cause
If a tape is fast-forwarded or rewound and then the transport stops abruptly (or if the tape pack is loose from storage), the tape can **cinch** — layers slip against each other, causing longitudinal wrinkles or creases along the tape width. This is different from a single crease; cinching produces multiple **parallel lengthwise buckles**.

#### Signal Effect
- Multiple thin vertical dropout stripes distributed across the width of the picture
- The stripes are consistent across many consecutive frames (the damage runs along the tape length)
- Affected scan lines show **intermittent short dropout bursts** (the buckled tape lifts off the head contact surface repeatedly as it passes)
- Chroma and luma drop out simultaneously in the affected stripe positions
- Audio may have brief interruptions if the linear audio track edge is affected

#### Simulation Method
1. Generate `N_cinch_stripes` (typically 3–12) horizontal positions distributed pseudo-randomly
2. Each stripe has a width of 2–15 pixels and a `severity` (0–1)
3. For each frame, for each stripe:
   - Apply dropout at the stripe position: attenuate luma/chroma toward zero within the stripe bounds
   - Add **jitter to the stripe position** per line (±1–3 pixels per scan line) — the buckle is not perfectly straight
   - Apply DOC substitution (previous line content fills the stripe)
4. Fade the stripe damage in/out over many frames (cinch damage often worsens and then partially resolves as the tape relaxes)

---

### 19.7 Worn or Damaged Video Heads

#### Physical Cause
Video heads wear through normal use. The ferrite or metal-in-gap material at the tip erodes, increasing the effective gap length and rounding the gap edges. Severe wear or physical impact can crack the head tip.

#### Signal Effect

**Worn heads (gradual):**
- Reduced output at **high frequencies** (shorter recorded wavelengths → less flux per unit length → less recovered voltage)
- The FM luminance upper sidebands are attenuated → **horizontal resolution degrades softly**
- White peaks (highest FM deviation) are more attenuated than mid-grey → **white compression** (bright areas look slightly murkier)
- SNR decreases uniformly → picture appears slightly noisier
- The effect is identical on both heads (symmetrical wear)

**Asymmetrically worn heads:**
- One head wears faster than the other (common) → alternating fields have different SNR/resolution
- Creates a subtle **field-alternating flicker** — one field is slightly sharper/brighter than the other
- At 30 fps (interlaced), this appears as a low-amplitude 30 Hz flicker

**Cracked/chipped head tip:**
- Produces a **permanent horizontal band** of heavy noise at a fixed vertical position in every field scanned by that head
- The band corresponds to the point in the scan where the chipped edge contacts the tape
- Width: 1–8 lines depending on chip size

#### Simulation Method
1. Define a per-head `wear_level` (0.0 = new, 1.0 = fully worn)
2. Apply a **high-frequency rolloff** that increases with wear: model as a first-order LPF whose cutoff moves from 3.5 MHz (new) down to 2.0–2.5 MHz (worn) as `wear_level` increases
3. Apply **white compression**: for luma values above 80 IRE, apply a soft-clip curve that increases in strength with wear
4. For asymmetric wear: apply the rolloff independently to odd/even fields
5. For a chipped head: add a **fixed horizontal noise band** (2–8 lines wide) at a defined vertical position, present only on the fields scanned by the damaged head; model band as salt-and-pepper noise at full amplitude

---

### 19.8 Warped or Cupped Tape

#### Physical Cause
Tape stored in improper conditions (high humidity, temperature cycling) can permanently deform across its width, taking a **concave or convex cross-section** (cupping). This changes the contact angle between the tape and the cylindrical head drum.

#### Signal Effect
- The tape no longer lies flat against the drum → **head-tape contact is intermittent**
- Produces a characteristic **periodic horizontal noise banding** that repeats at the drum rotation rate (30 Hz for each head = 60 Hz total → one noise band per field)
- The noise band sweeps through the frame slowly if the cupping is mild, or is fixed near the top or bottom of the picture if severe
- Chroma is affected at least as much as luma (contact loss kills both simultaneously)
- Audio is usually unaffected (linear audio head has different geometry)

#### Simulation Method
1. Model the contact loss as a **sinusoidal envelope** modulating the playback signal amplitude, synchronized to the head drum rotation:
   `contact(y) = 1.0 - cup_depth * sin(π * y / active_lines)^2`
2. Where `cup_depth` is 0 (flat) to 1 (severe cupping), and the sinusoid peaks at the vertical center of the field (where the head is furthest from the center of the tape)
3. Attenuate luma/chroma signal by `(1 - contact(y))` at each line
4. Below a contact threshold (~0.4), trigger DOC substitution
5. For very severe cupping: replace lost regions with broadband FM noise (demodulator loses lock)

---

### 19.9 Magnetic Print-Through

#### Physical Cause
When tape is stored for a long time tightly wound, the magnetic field of one layer of tape **imprints weakly** onto adjacent layers. The signal on one layer magnetically induces a ghost copy on the layers wound above and below it.

#### Signal Effect
- A **ghosted, low-amplitude echo** of the video signal appears displaced in time
- The echo is typically **~1–3 seconds ahead of or behind** the main signal (because a few layers of tape wrap corresponds to several seconds of content)
- The ghost is always present, not a dropout event — it is a **permanent double-exposure** effect on the recording
- Ghost amplitude: typically **–30 to –45 dB** relative to main signal (barely visible to visible depending on severity and storage time)
- The ghost is in the same orientation (not horizontally mirrored) but may be slightly smeared (the print-through weakens HF content more than LF)

#### Simulation Method
1. Define `printthrough_delay_frames` (typically 30–90 frames at 29.97 fps ≈ 1–3 seconds)
2. Define `printthrough_level` (linear gain, typically 0.02–0.1 = –34 to –20 dB)
3. `output[t] = main[t] + printthrough_level * LPF(main[t - printthrough_delay_frames])`
4. Apply a **low-pass filter** (cutoff ~1.5 MHz equivalent) to the ghost — print-through loses high frequencies
5. Optionally add both pre-echo (`t + delay`) and post-echo (`t - delay`) at slightly different levels (the layer on each side of the main signal contributes independently)

---

### 19.10 Magnetization Loss / Partial Erasure

#### Physical Cause
- Proximity to a magnetic field (speaker magnet, degausser, CRT monitor edge, transformer)
- Repeated playback without re-recording (heads slowly partially erase the tape — negligible under normal use but significant on very old tapes with many plays)
- Improperly bulk-erased tape (erase field too weak or incorrect orientation)

#### Signal Effect
- **Uniform reduction in tape remanence** → attenuated playback signal level
- FM luma: carrier amplitude drops → less headroom above noise floor → SNR decreases; signal still demodulates correctly until attenuation is severe
- At moderate erasure: picture looks noticeably **grainier and softer** (similar to late-stage head clog, but bilateral — both heads equally affected)
- At severe erasure: FM carrier collapses into noise → picture is partially or fully lost
- **Color erasure is more pronounced**: the low-frequency color-under signal is more easily disrupted than the higher-amplitude FM carrier

#### Simulation Method
1. Apply a global `erase_level` (0.0 = unerased, 1.0 = fully erased)
2. Reduce signal amplitude: `signal *= (1.0 - erase_level)`
3. Boost noise floor: `noise_amplitude *= 1.0 + (erase_level * 6.0)` (noise becomes dominant)
4. Apply **extra chroma attenuation**: `chroma_signal *= (1.0 - erase_level * 1.4)` — chroma erases faster than luma
5. For partial/localized erasure (e.g., a magnet held near part of a reel): apply the above within a defined time region of the tape, with smooth fade in/out at the boundaries (the magnetic field transition is gradual)

---

### 19.11 Splice / Edit Glitch

#### Physical Cause
Physical tape splices (from manual editing) or electronic edit points (where recording was stopped and restarted) produce discontinuities in the signal and CTL track.

#### Signal Effect

**Physical splice:**
- The splice point passes the heads: **one or two frames** of complete signal loss as the tape joint crosses the head
- Followed immediately by **normal video** from the new tape section
- The CTL track may have a **missing pulse** at the splice → capstan servo momentarily loses lock → 1–3 frames of tracking instability after the splice
- A slight **tape tension change** at the splice → brief horizontal geometry distortion just before and after

**Electronic edit (insert/assemble edit point):**
- The VCR was paused during record and restarted → the FM carrier phase and chroma subcarrier phase are slightly mismatched at the edit point
- Results in a **single-frame color flash** (chroma phase jump → brief wrong hue for one field)
- May have a **1–2 line vertical sync glitch** (if the record VCR's sync timing was slightly off when restarting)

#### Simulation Method

**Physical splice:**
1. At the splice frame: set luma and chroma to zero for 2–4 frames (complete loss)
2. For 3–8 frames after: apply horizontal position jitter (±5–20 pixels, decaying exponentially) simulating capstan servo hunting
3. Apply a slight vertical roll for 1–2 frames (sync re-lock)

**Edit glitch:**
1. At the edit frame: inject a chroma phase step of ±45–180° for exactly one field
2. On the same field: shift horizontal sync by ±3–10 µs (one-frame H-sync glitch)
3. Apply a brief (2–4 line) luminance step (black or flash) at the edit point line

---

### 19.12 Capstan / Pinch Roller Failure

#### Physical Cause
- **Hardened pinch roller**: The rubber roller hardens with age and loses its grip on the tape
- **Worn capstan shaft**: The shaft develops flat spots or surface roughness
- **Belt-driven capstan slippage**: The motor coupling belt stretches and slips

#### Signal Effect
- **Periodic wow and flutter** at the rotational frequency of the capstan or roller
  - Capstan shaft diameter: ~3 mm → 1 rotation per ~9.4 mm of tape → at SP (3.335 cm/s): ~3.5 Hz
  - Pinch roller diameter: ~16–20 mm → 1 rotation per ~50–63 mm → ~0.5–0.7 Hz per rotation
- Hardened roller: reduced tape tension → the tape moves slightly slower than commanded → **audio pitch drops slightly**, picture **horizontally smears** on fast motion
- Belt slippage: **intermittent speed glitches** — brief 1–5 frame moments of slightly wrong tape speed, then correction; resembles slow wow but with discrete step changes

#### Simulation Method
1. Define a `capstan_wow` parameter as a superposition of periodic components:
   ```
   speed_variation(t) = A1 * sin(2π * 3.5 * t + φ1)    // capstan shaft
                      + A2 * sin(2π * 0.6 * t + φ2)    // pinch roller
                      + A3 * noise_LFO(t)               // random component
   ```
2. Apply to tape playback as a horizontal position displacement:
   `x_displacement(t) = integral(speed_variation(t)) * pixels_per_second_of_delay`
3. Apply the same modulation to audio pitch: `pitch_shift(t) = 1.0 + speed_variation(t)`
4. For belt slippage: add occasional discrete `speed_step` events (random, ~once per 2–15 seconds): instantly shift `speed_variation` by ±0.005–0.02 for 3–8 frames, then return

---

### 19.13 Demagnetized / Saturated Heads

#### Physical Cause
- **Demagnetized heads**: Residual magnetization builds up in the head core over time (especially if the VCR was improperly stored near a field). Paradoxically, a demagnetized head has better HF response than a partially re-magnetized one.
- **Magnetically saturated heads**: Exposure to a strong DC field can partially magnetize the head core, adding a DC bias to the playback signal.

#### Signal Effect

**DC magnetized head:**
- A **DC offset** in the playback signal shifts the FM carrier rest frequency
- This shifts the apparent luma level: a positive DC shift pushes the carrier above 3.4 MHz → black level rises (brighter minimum) → reduced contrast in dark areas
- The chroma is unaffected (it's at a completely different frequency)
- Appears as a **raised black level** or slightly crushed shadow detail

**Demagnetized head (too clean):**
- Slightly improved HF response → picture looks slightly sharper than normal (not necessarily a "damage" artifact)

#### Simulation Method
1. Define `head_dc_offset` (positive or negative, in IRE units, typically –5 to +10 IRE)
2. Apply as a luma offset: `luma_output = luma_signal + head_dc_offset`
3. Clamp the output to valid luma range (–40 to 100 IRE)
4. A positive offset raises black level → reduce effective contrast ratio in shadows:
   `shadow_luma = max(luma, dc_offset)` — shadows can never go below the offset level

---

### 19.14 VCR Power Interruption / Mid-Record Dropout

#### Physical Cause
Power is interrupted during recording (VCR turned off, power outage, tape ran out and was force-stopped). The record head current is cut off abruptly.

#### Signal Effect
- The last few frames before the power cut have **progressively fading signal** (record current drops as capacitors discharge): typically 1–4 frames of fading
- The exact endpoint has a **partial-line record cutoff**: the last scan line on tape may be cut mid-line, producing a horizontal split effect on the final frame (top portion is valid signal, bottom is noise/unrecorded)
- After the cutoff, the tape has **unerased/old content or blank tape** (depending on whether this section was previously recorded)
- On playback, there is a **sharp transition** from the recorded content to noise (blank) or a jarring cut to old content (over-recorded tape)

#### Simulation Method
1. At the designated cutoff point, over 4–8 frames, **reduce record-level amplitude exponentially**: `record_gain(t) = exp(-t / T_decay)` where `T_decay` ≈ 2 frames
2. Apply the gain to both luma and chroma, making the picture fade toward noise (not black — the FM carrier is fading, not the video)
3. On the final frame, split the frame at a random horizontal line `y_cut`:
   - Lines 0 to `y_cut`: valid (faded) video
   - Lines `y_cut` to end: full broadband noise
4. After the cutoff: transition to either blank tape noise (flat spectral noise), or old content from a previous recording

---

### 19.15 Azimuth Misalignment

#### Physical Cause
One of the video head mounting screws has loosened, or the head drum has been improperly serviced, causing one head's gap to be at the wrong azimuth angle.

#### Signal Effect
- Because azimuth loss is **frequency-dependent** (worse at higher frequencies), a misaligned head produces a characteristic **HF rolloff on every other field** (the one scanned by the misaligned head)
- Mild misalignment (1–2°): slight softening of every other field → flickering apparent sharpness
- Moderate misalignment (3–5°): noticeable picture softness on alternating fields, chroma reduced
- Severe misalignment (> 6°): extensive crosstalk from adjacent tracks → increased noise on alternating fields

**Azimuth loss formula (per field):**
```
Attenuation(f) = sinc(π × d × sin(Δθ) / (v / f))
```
Where `Δθ` is the azimuth error, `d` is the track width, `v` is head-to-tape speed, and `f` is the frequency of interest.

#### Simulation Method
1. Define `azimuth_error_degrees` (0 = correct, up to ~10° for severe misalignment)
2. Compute an attenuation curve across the FM frequency band (3.4–4.4 MHz):
   - At each frequency f: `atten(f) = sinc(π * d * sin(Δθ_rad) * f / v)`
   - With d = 58e-6 m (SP track width), v = 5.8 m/s
3. Apply this frequency-domain attenuation to every other field (the one scanned by the misaligned head)
4. Translate the FM frequency attenuation to equivalent luma attenuation:
   - Higher FM frequencies correspond to brighter luma values
   - A rolloff above 4 MHz → compression of white peaks → use a white-compressor curve matching the attenuation profile

---

### 19.16 Summary of Damage Effects vs. Signal Domain

| Damage Type | Luma Affected | Chroma Affected | Audio Affected | Temporal Pattern | Spatial Pattern |
|---|---|---|---|---|---|
| Head clog (partial) | ✓ (noise, dropout) | ✓ | ✗ | Every Nth field | Random H bands |
| Tape crease | ✓ | ✓ | Rarely | Consistent frame to frame | Fixed vertical stripe |
| Tape stretch | ✓ (mild noise) | ✓ (phase drift) | ✓ (pitch) | Intermittent | Horizontal noise bars |
| Mistrack | ✓ | ✓ | ✗ | Persistent or drifting | Horizontal noise bands |
| Sticky shed | ✓ (worsening) | ✓ | ✓ (squeal) | Accelerating over time | Spreading from top |
| Tape cinching | ✓ | ✓ | Sometimes | Consistent section | Multiple V stripes |
| Worn heads | ✓ (soft) | ✓ (mild) | ✗ | Every field (uniform) | HF detail loss |
| Tape cupping | ✓ (bands) | ✓ | ✗ | Per-field periodic | H band mid-frame |
| Print-through | ✓ (ghost) | ✓ (ghost) | ✓ (echo) | Constant echo | Full frame ghost |
| Partial erasure | ✓ (noise) | ✓✓ (heavy) | ✗ | Section of tape | Uniform degradation |
| Splice/edit glitch | ✓ (brief) | ✓ (flash) | Sometimes | 1–4 frames | Full frame |
| Capstan failure | ✓ (geometry) | ✓ (phase) | ✓ (pitch/wow) | Periodic/intermittent | Horizontal jitter |
| DC head magnetization | ✓ (black level) | ✗ | ✗ | Persistent | Full frame offset |
| Power cutoff | ✓ (fade/cut) | ✓ (fade/cut) | ✓ (cut) | End of recording | Split frame |
| Azimuth misalignment | ✓ (HF on alt fields) | ✓ (alt fields) | ✗ | Every other field | HF smear |

---

### 19.17 Horizontal Tearing, Skew Error, and H-Sync Instability

This section covers the family of artifacts visible when tape tension, servo lock, or H-sync integrity is compromised — resulting in horizontally displaced, skewed, or zigzagging scan lines. These are among the most visually dramatic VHS artifacts and involve several distinct but related mechanisms that can occur simultaneously.

---

#### 19.17.1 Skew Error

##### Physical Cause
**Skew** is caused by a mismatch between the tension of the tape as it enters the head drum versus how the VCR's sync circuits expect it to arrive. As the tape wraps ~180° around the drum, any variation in **back-tension** (applied by the supply reel brake) changes the effective tape path length across the drum surface.

At the **start of each field scan** (when the video head first makes contact with the tape), if back-tension is too high or too low, the tape is effectively pulled forward or backward relative to where the servo expects it. This shifts the **horizontal sync phase** of the first lines of the field, with the error decaying as the capstan servo compensates across the field. The result is a **characteristic lean or wedge**: the top of the frame is horizontally displaced in one direction and the displacement tapers to near-zero by the bottom of the frame (or vice versa, depending on tension direction).

##### Signal-Level Mechanism
- Each scan line's horizontal sync pulse arrives earlier or later than expected
- Early sync = line shifts **left** (the active video has already started before the display resets)
- Late sync = line shifts **right** (display resets before active video begins → left side is black, content pushed right)
- The displacement follows the tape-drum contact arc: it is **maximum at the top of the field** (first lines of the head scan) and decays toward zero over the next 30–80 lines as servo feedback corrects it
- The decay curve is approximately **exponential** (first-order servo response)

##### Causes That Produce Skew
| Cause | Direction | Typical Displacement |
|---|---|---|
| Supply reel brake too tight | Lines shift right at top | 5–40 pixels |
| Supply reel brake too loose | Lines shift left at top | 5–30 pixels |
| Worn/loose back-tension band | Alternating direction per field | 3–15 pixels |
| Cold/stiff tape (cold environment) | Right shift at top | 5–20 pixels |
| Playing a pre-record tape on mismatched VCR | Either direction | 2–25 pixels |

##### Simulation Method
1. Define `skew_amplitude` (in pixels, signed: positive = right shift, negative = left)
2. Define `skew_decay_lines` (how many lines for the error to decay to ~1/e ≈ 37% — typically 30–80 lines)
3. For each scan line `y` in the field, compute the horizontal offset:
   ```
   skew_offset(y) = skew_amplitude * exp(-y / skew_decay_lines)
   ```
4. Apply this as a **horizontal shift** to each scan line: shift the pixel data left or right by `round(skew_offset(y))` pixels, filling the exposed edge with black or wrapped content
5. For sub-pixel accuracy, apply a **bilinear horizontal interpolation** rather than integer shift — this avoids staircase aliasing on the skew edge
6. The skew amplitude can vary **field to field**: add a slow LFO or random walk to `skew_amplitude` to simulate a worn brake that doesn't apply consistent tension:
   ```
   skew_amplitude(field) = skew_base + skew_variation * sin(2π * 0.3 * field / fps)
   ```
7. For the skew to look correct, also apply a slight **luminance level shift** at the skewed top lines: back-tension variation causes slight head-tape contact variation → mild brightness change at the top 5–10 lines

---

#### 19.17.2 H-Sync Jitter and Horizontal Tearing

##### Physical Cause
When the capstan servo loses phase lock — or never properly acquires it — the tape speed oscillates. Because H-sync timing is derived from the tape content itself (not from a crystal), any tape speed variation directly translates into **H-sync arrival time variation**. Each scan line's sync pulse arrives at a slightly different time, causing the display to reset its horizontal scan at the wrong moment for each line.

This is distinct from skew: skew is a **field-level low-frequency offset** (decaying over the top of the field), while H-sync jitter is **line-by-line independent variation** that can be high frequency.

##### Types of H-Sync Timing Error

**Random jitter (common):**
- Each line's H-sync is displaced by an independent random amount
- Range: ±1–10 pixels (mild) to ±20–60 pixels (severe)
- Produces a "shimmering" or "wobbly" appearance on vertical edges

**Correlated jitter (capstan servo hunting):**
- The servo is oscillating — it overcorrects in one direction, then the other
- Produces **coherent wave patterns** across consecutive lines: lines shift right, then left, then right, in a wave that propagates downward through the frame
- This is the **zigzag / sawtooth pattern** visible in the lower half of the reference image
- The oscillation frequency is the servo loop's natural frequency: typically **40–200 Hz** (meaning 0.4–2 line periods of displacement per Hz at 15,734 lines/sec)
- Visually: **vertical edges in the picture become zigzagging sine waves**

**Step-function tearing:**
- A discrete jump in H-sync phase at a specific line
- All lines below the jump point are shifted by a fixed amount
- Lines above are normal
- Caused by: tape splice, CTL pulse glitch, or brief loss of servo lock at one point in the field

##### Signal-Level Mechanism
The H-sync pulse position in time determines where the display (or digitizer) considers the left edge of the picture to be. The VCR outputs sync at whatever time the tape delivers it — without a TBC, this is the raw recovered timing. Variation in this timing appears directly as horizontal pixel displacement:

```
pixel_displacement = H_sync_time_error × active_pixel_rate
                   = H_sync_time_error (µs) × 720 pixels / 52.6 µs
                   ≈ H_sync_time_error × 13.7 pixels/µs
```

So a 1 µs H-sync error → ~14 pixel horizontal displacement.

##### Simulation Method

**Step 1 — Build a per-line timing error array:**

For each scan line `y` in the frame, compute `h_error[y]` (in pixels):

```
// Random component (baseline VHS jitter)
h_jitter[y] = gaussian_noise() * jitter_sigma        // jitter_sigma: 1–8 px for mild, 10–40 for severe

// Servo hunt component (produces the zigzag)
hunt_freq = servo_hunt_hz / line_rate                // normalized frequency (cycles per line)
h_hunt[y] = hunt_amplitude * sin(2π * hunt_freq * y + hunt_phase)

// Skew component (from 19.17.1 — field-level, slow)
h_skew[y] = skew_amplitude * exp(-y / skew_decay_lines)

// Total
h_error[y] = h_jitter[y] + h_hunt[y] + h_skew[y]
```

Typical parameter ranges for the **zigzag artifact** (as seen in the image):
```
hunt_amplitude:  15–50 pixels
servo_hunt_hz:   60–180 Hz  (produces ~4–12 lines per cycle of zigzag)
hunt_phase:      varies per field (random or slow drift)
```

**Step 2 — Apply horizontal shift per line:**

For each line `y`:
```python
shift = round(h_error[y])
if shift > 0:
    line_out[y] = [BLACK] * shift + line_in[y][0 : width - shift]
elif shift < 0:
    line_out[y] = line_in[y][-shift : width] + [BLACK] * (-shift)
else:
    line_out[y] = line_in[y]
```

For sub-pixel smoothness (recommended), use fractional shift with linear interpolation:
```python
frac = h_error[y] - floor(h_error[y])
line_out[y][x] = lerp(line_in[y][x - floor], line_in[y][x - floor - 1], frac)
```

**Step 3 — Modulate hunt characteristics across the frame:**

The zigzag in the reference image is stronger in the lower half and absent in the upper half. This is because:
- The servo has partially re-locked at the start of the field (during vertical blanking)
- As the field progresses and the servo drifts, oscillation builds up
- Model this as an **amplitude envelope** on `h_hunt`:
  ```
  hunt_envelope(y) = smoothstep(0, envelope_start_line, y)
  h_hunt[y] *= hunt_envelope(y)
  ```
  Where `envelope_start_line` might be lines 120–200 (the upper portion of the frame is stable, lower is unstable)

**Step 4 — Add associated luma distortion:**

When H-sync timing is severely off, the FM demodulator's phase tracking also becomes unstable. Add a correlated **luma brightness variation** to the affected lines:
```
luma_offset[y] = luma_brightness_variation * sin(2π * hunt_freq * y) * hunt_envelope(y)
```
Amplitude: ±3–8 IRE. This gives the slightly brighter/darker banding that follows the zigzag wave.

---

#### 19.17.3 The Split-Frame / Partial Vertical Sync Loss

##### Physical Cause
When tracking is severely off or the CTL track is damaged, the **vertical sync pulse** in the playback signal may be corrupted or arrive at the wrong time. The TV or display re-acquires vertical sync mid-frame, causing the image to **roll** or **split**:

- The upper portion of the frame shows content from one field position
- The lower portion shows content from a different field position
- A **dark bar** (the vertical blanking interval) is visible as a horizontal black band at the split point

This is the dark horizontal band visible near the bottom of the reference image, with valid video both above and below it — the TV has re-locked vertical sync at the wrong point.

##### Simulation Method
1. Define `vsync_lock_line` — the line at which the display re-acquires vertical sync (e.g., line 380 in a 480-line frame)
2. Recompose the output frame:
   - Lines 0 to `vsync_lock_line`: content from the current field, starting at `current_field_line_offset`
   - The black band: insert 4–12 lines of black (the VBI) at `vsync_lock_line`
   - Lines after the band: content from the *same field* but starting at line 0 (the display has re-locked to the top of the field) — effectively a **vertical wrap-around** of the content
3. Apply a slight horizontal sync instability (±5–15 px of jitter) for 10–20 lines around the black band — the servo is hunting as it re-locks
4. The position of `vsync_lock_line` can drift slowly upward frame by frame (the display is close to losing lock and is adjusting its V-hold): `vsync_lock_line(t) -= drift_rate` — until it reaches line 0 and the frame stabilizes or fully rolls

---

#### 19.17.4 Putting It Together: The Reference Image Artifact Stack

The example image shows the following layered effects, from top to bottom:

```
Lines   0 – 120:  Mild skew (upper-left lean), low H-jitter, image mostly intact
                  → Apply skew_amplitude ≈ +12 px with decay over ~80 lines
                  → Apply h_jitter with sigma ≈ 3 px

Lines 120 – 310:  Skew has decayed, but servo hunt begins to build
                  → hunt_envelope ramps up from 0 → 1 over this region
                  → hunt_amplitude grows from 0 → ~30 px
                  → hunt_freq ≈ 100 Hz → ~6–7 lines per zigzag cycle

Lines 310 – 410:  Full servo hunt — maximum zigzag
                  → hunt_amplitude ≈ 35–50 px at peak
                  → luma_offset oscillation visible as brightness banding
                  → chroma phase fully lost in this region → grayscale / desaturated

Lines 410 – 430:  Dark band — VBI / partial V-sync re-lock
                  → 15–20 lines of near-black

Lines 430 – 480:  Bottom portion — display has re-locked, but at wrong phase
                  → image continues from a different vertical position
                  → same zigzag but at lower amplitude (new lock is still unstable)
```

**Combined parameter set for this effect:**

```javascript
// Skew
skew_amplitude:        +14        // pixels, right-shift at top
skew_decay_lines:       70        // exponential decay constant

// H-sync jitter
jitter_sigma:            3.5      // pixels, random per line (always present)

// Servo hunt (zigzag)
hunt_amplitude:         40        // pixels at peak
hunt_freq_hz:          105        // Hz (servo natural frequency)
hunt_start_line:       130        // line at which hunt envelope begins
hunt_full_line:        290        // line at which envelope reaches 1.0

// Luma brightness modulation from hunt
luma_hunt_amplitude:     5        // IRE, follows hunt wave

// Chroma loss in hunt region
chroma_loss_start_line: 280       // chroma fades out below this
chroma_loss_end_line:   410       // chroma is fully gone by this line

// V-sync re-lock / split frame
vsync_split_line:       415       // line where dark band appears
vbi_band_height:         18       // lines of black in the band
post_split_h_jitter:     10       // extra H jitter sigma for lines after band
```

---

#### 19.17.5 Edge-Wrap and Black Fill Behavior

When a scan line is shifted significantly to the right, the left edge of the output has no source pixels. The correct VHS behavior at this boundary depends on the circuit:

- **Most VCRs**: Left edge fills with **the last valid pixel** of the previous line (a 1H delay artifact — produces a smeared edge rather than black)
- **Some VCRs**: Left edge fills with **black** (blanking level, ~7.5 IRE)
- **Digitizing from VHS**: The capture card may wrap pixels from the right edge to the left, or fill with noise

For simulation, **smear-fill** (repeat last pixel) is more authentic than black-fill:
```python
if shift > 0:  # right shift → fill left edge
    fill_value = line_in[y][0]   # repeat leftmost pixel (or last line's rightmost)
    line_out[y] = [fill_value] * shift + line_in[y][0 : width - shift]
```

For large shifts (> 30 px), optionally blend the fill from solid to noisy:
```python
fill_noise = lerp(fill_value, random_noise(), clamp((shift - 20) / 40, 0, 1))
```

---

When multiple damage effects are active simultaneously, apply them in the following order to the signal pipeline to achieve physically accurate results:

```
1. Source video (decoded composite / Y+C)
        │
2. Apply TAPE DAMAGE to signal amplitude/phase
   (print-through addition, partial erasure attenuation, crease/cinch dropouts)
        │
3. Apply HEAD DAMAGE to frequency response
   (head clog amplitude, worn head HF rolloff, azimuth attenuation, DC offset)
        │
4. Apply TRANSPORT ERRORS to geometry/timing
   (capstan wow/flutter → horizontal position offset, tracking error → noise bars)
        │
5. Apply DROPOUT COMPENSATION
   (substitute previous line for any detected dropouts from steps 2–3)
        │
6. Apply CHROMA RECONSTRUCTION ERRORS
   (phase jitter from stretch/wow → hue instability, loss from clog → color killer)
        │
7. Apply SYNC REGENERATION ERRORS
   (splice glitches → H-sync displacement, power cutoff → frame split)
        │
8. Composite output (with all damage applied)
```

This ordering mirrors the actual signal path inside a VCR and ensures that interactions between damage types are physically plausible — for example, dropout compensation (step 5) correctly conceals head clog dropouts (step 3) but cannot conceal transport-level geometry errors (step 4).

---

*End of VHS/VCR NTSC Technical Reference*
