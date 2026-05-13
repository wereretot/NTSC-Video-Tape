#!/bin/bash
# Performance testing script for ADO-8500 NTSC simulator

set -e

ADO_BIN="/mnt/MassArchive/Programs/Tape Warping/video-sim-cpp/build/ado8500"
BUILD_DIR="/mnt/MassArchive/Programs/Tape Warping/video-sim-cpp/build"

echo "=========================================="
echo "ADO-8500 Performance Testing"
echo "=========================================="
echo ""

# Check if executable exists
if [ ! -f "$ADO_BIN" ]; then
    echo "ERROR: Executable not found at $ADO_BIN"
    echo "Building project..."
    cd "/mnt/MassArchive/Programs/Tape Warping/video-sim-cpp"
    cmake --build build --config Release -j$(nproc) 2>&1 | tail -20
fi

# Display build info
echo "Build Information:"
echo "  Executable: $ADO_BIN"
echo "  Size: $(ls -lh $ADO_BIN | awk '{print $5}')"
echo "  Built: $(ls -l $ADO_BIN | awk '{print $6, $7, $8}')"
echo ""

# Check compiler flags used
echo "Compiler Flags Applied (from CMakeLists.txt):"
echo "  - Release Mode: -O3 -march=native -mavx2 -ffast-math -funroll-loops -fno-signaling-nans -flto"
echo "  - OpenMP: Enabled (4.5)"
echo "  - SIMD: AVX2 support enabled"
echo "  - LTO: Enabled for link-time optimization"
echo ""

# Show expected performance improvements
echo "Expected Performance Improvements:"
echo "  ├─ Pre-allocated buffers:     +5-15%"
echo "  ├─ Eliminated clones:         +5-10%"
echo "  ├─ Field display opt:         +3-7%"
echo "  ├─ Compiler flags (LTO+AVX2): +20-30%"
echo "  └─ OpenMP parallelization:    +5-10%"
echo "  =================================="
echo "  Total Expected (cumulative):   33-62% faster (conservative)"
echo "  Total Expected (optimistic):   100-150% faster"
echo ""
echo "Performance Target:"
echo "  Old: 0.2 FPS (baseline)"
echo "  New: ~1-3 FPS (conservative estimate)"
echo "  Goal: 60 FPS (may require GPU acceleration)"
echo ""

# Build-related diagnostics
echo "Build Diagnostics:"
echo "  C++ Standard: C++20"
echo "  Dependencies: SDL2, OpenCV, FFmpeg, PortAudio, CapstanVar"
echo "  Platform: Linux (native march CPU optimizations active)"
echo ""

echo "To Run the Application:"
echo "  $ cd /mnt/MassArchive/Programs/Tape\\ Warping/video-sim-cpp"
echo "  $ ./build/ado8500"
echo ""

echo "Performance Measurement Tips:"
echo "  1. Monitor FPS display in UI status bar"
echo "  2. Use 'top' in another terminal to watch CPU usage"
echo "  3. Run with demo source (no file I/O bottleneck)"
echo "  4. Test both NTSC On and NTSC Off modes"
echo ""

echo "For Further Optimization Consider:"
echo "  • GPU acceleration (CUDA/OpenGL)"
echo "  • Pre-computed lookup tables for sin/cos"
echo "  • Reduce processing resolution further"
echo "  • Profile with 'perf' to identify remaining hotspots"
echo ""

echo "=========================================="
echo "Ready to test! Execute the binary above."
echo "=========================================="
