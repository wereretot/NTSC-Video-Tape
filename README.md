# ADO-8500 Digital Video Effects Processor
### Analog Tape Simulation + Warp Effects · C++17 · Multi-threaded

Inspired by the Ampex/Grass Valley ADO series of Digital Video Effects
processors from the mid-1980s, with a full NTSC/VHS analog tape simulation
pipeline layered on top.

---

## Features

### NTSC / VHS Tape Simulation
- **YIQ signal path** — full Y/I/Q separation, all filtering done in signal space
- **Luma bandwidth limiting** — IIR lowpass (VHS SP ~ 2.4 MHz)
- **Chroma bandwidth limiting** — heavy IIR lowpass (VHS ~ 629 kHz) causing the
  characteristic horizontal color smear
- **Luma & chroma noise** — independent Gaussian noise on each channel
- **Sync pulse jitter** — correlated per-scanline horizontal displacement
- **Head switching noise** — wavy distortion on the last 8-12 scanlines
  (vertical blanking interval artifact)
- **Tape dropouts** — probabilistic white/black line events
- **Tracking instability** — sinusoidal horizontal banding at tape speed
- **Composite ghosting** — attenuated horizontal RF echo
- **VHS edge enhancement** — unsharp mask sharpening circuit simulation
- **Interlace field simulation** — odd/even field blending
- **Black crush / white clip** — IRE level clamping
- **Color saturation rolloff** — tape degradation model

### VHS Helical Scan Head Simulation
- **Diagonal head sweep** — gradual horizontal offset across each scanline
  simulating the 5.97-degree helical scan angle; creates the characteristic
  gradual horizontal drift visible on misadjusted VCRs
- **Head switching boundary** — simulates the video head switch at the VBI
  with glitch/tear artifacts and per-switch jitter; the two video heads
  (A/B) produce slightly different signal characteristics
- **FM luminance carrier noise** — models the VHS FM luminance carrier
  (3.4-4.4 MHz) noise causing luminance streaking and jitter in the
  demodulated signal
- **Chroma underband crosstalk** — simulates the VHS color under recording
  system where chroma is downconverted to ~629 kHz; adjacent-track
  crosstalk bleeds through with a characteristic 40-line beating pattern
- **Inter-field chroma phase error** — per-field phase discontinuity from
  the two video heads having slightly different azimuth alignments;
  produces subtle color flicker between fields
- **Head pre-echo / post-echo** — ghost signal from adjacent helical
  track passes leaking through, visible as faint horizontal echoes
- **Head drum eccentricity** — simulates mechanical wobble of the rotating
  video head drum (1800 RPM), producing periodic vertical displacement

### Warp Effects (12)
BYPASS · TUMBLE · PAGE TURN · RIPPLE · SPHERE · SQUEEZE · MOSAIC · TRAILS ·
MIRROR · CUBE SPIN · KALEIDOSCOPE · SHATTER

### Threading
```
[Capture Thread]  →  [Raw Queue]  →  [Processing Thread]  →  [Output Queue]  →  [Main/Render]
                                          ↑ OpenMP
                                    (intra-frame rows)
```
- **Capture thread** — camera, video file, or procedural SMPTE test signal
- **Processing thread** — warp effect → NTSC sim pipeline; drop-oldest for real-time
- **Main thread** — SDL2 event loop, ImGui controls, OpenGL display
- **OpenMP** — per-scanline row loops in NTSC sim & pixel effects

---

## Dependencies

| Package    | Version    | Install                                |
|------------|------------|----------------------------------------|
| SDL2       | ≥ 2.0.18   | `apt install libsdl2-dev`              |
| OpenCV     | ≥ 4.5      | `apt install libopencv-dev`            |
| OpenGL     | ≥ 3.3 Core | Usually bundled with drivers           |
| Dear ImGui | v1.90.6    | Auto-fetched by CMake FetchContent     |
| OpenMP     | any        | `apt install libomp-dev` (optional)    |

**macOS (Homebrew):**
```bash
brew install sdl2 opencv
```

**Windows (vcpkg):**
```bash
vcpkg install sdl2 opencv4 opengl
```

---

## Build

```bash
git clone <repo>
cd ado8500
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
./build/ado8500
```

**Debug build with ThreadSanitizer:**
```bash
cmake -B build-dbg -DCMAKE_BUILD_TYPE=Debug -DADO_TSAN=ON
cmake --build build-dbg
./build-dbg/ado8500
```

**Without OpenMP (single-threaded processing):**
```bash
cmake -B build -DADO_OPENMP=OFF
```

---

## Controls

| UI Section      | Description                              |
|-----------------|------------------------------------------|
| INPUT SOURCE    | Camera / Video file / SMPTE test signal  |
| EFFECT SELECT   | 12 warp effects + bypass                 |
| TAPE SPEED      | SP / LP / EP presets (quality preset)    |
| NTSC PARAMS     | Fine-tune every tape artifact            |
| HELICAL SCAN    | Head sweep, FM noise, crosstalk, etc.   |
| EFFECT PARAMS   | Amount, Speed, Depth per effect          |
| MOTION          | Auto / Play / Pause / Reverse phase      |
| TIMECODE        | Live SMPTE timecode display              |

---

## NTSC Simulation Reference

The simulation follows the NTSC composite signal path:

```
RGB Input
  └─→ [RGB→YIQ] ──→ Y channel ──→ [LP filter 2.4MHz] ──→ [+ luma noise] ──→ [+ FM carrier noise]
                │                                                │
                └─→ I channel ──→ [LP filter 629kHz]  ──→ [+ chroma noise] ──→ [+ crosstalk]
                │                                                │
                └─→ Q channel ──→ [LP filter 629kHz]  ──→ [+ chroma noise] ──→ [+ crosstalk]
                                                                  │
       [sync jitter]  [head switching]  [dropouts]  [tracking] ←┘
                                                                  │
       [helical sweep]  [field phase]  [head echo]  [drum ecc] ←┘
                                                                  │
                                                       [YIQ→RGB] ─┘
                                                                  │
                   [edge enhance]  [ghost/echo]  [IRE clamp]  ←──┘
                                                                  │
                                                          RGB Output
```

VHS IIR filter coefficients (first-order, causal):
- Luma:   `α = exp(-2π · 2.4/13.5) = 0.327`  → `y[n] = 0.673·x[n] + 0.327·y[n-1]`
- Chroma: `α = exp(-2π · 0.629/13.5) = 0.746` → `y[n] = 0.254·x[n] + 0.746·y[n-1]`

Helical scan physical constants:
- Head drum speed: 1800 RPM (30 Hz)
- Azimuth angle: ±5.97° (A/B heads)
- FM luminance carrier: 3.4-4.4 MHz (SP)
- Chroma under frequency: 629.375 kHz
- Head switch point: line 7 of VBI

---

## License
MIT — do whatever you want with it.
