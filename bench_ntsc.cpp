//#define ADO_OPENMP 1
#include "src/ntsc_simulator.h"
#include "src/constants.h"
#include <chrono>
#include <cstdio>
#include <opencv2/core.hpp>

int main() {
    NTSCSimulator sim;
    sim.initBuffers(FW, FH);

    cv::Mat in(FH, FW, CV_8UC3, cv::Scalar(128, 128, 128));
    cv::Mat out;

    EngineParams ep{};
    VideoParams vp{};
    vp.tracking_error = 0.3f;
    vp.dropout_rate = 0.05f;
    vp.motor_health = 0.2f;
    vp.tape_crease = 0.1f;
    vp.helical_sweep = 0.5f;
    vp.head_switch_jitter = 0.3f;

    // Measure warm + timed frames
    const int NUM_FRAMES = 50;
    const int WARMUP = 5;

    for (int iter = 0; iter < 3; ++iter) {
        for (int i = 0; i < WARMUP; ++i)
            sim.process(in, out, i, ep, vp, 1.f, 1.f, 1.f / 60.f, 0);

        auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < NUM_FRAMES; ++i)
            sim.process(in, out, i, ep, vp, 1.f, 1.f, 1.f / 60.f, 0);
        auto end = std::chrono::steady_clock::now();

        double total = std::chrono::duration<double>(end - start).count();
        printf("Run %d: %.1f FPS (%.2f ms)\n", iter+1, NUM_FRAMES/total, total*1000/NUM_FRAMES);
    }
    return 0;
}
