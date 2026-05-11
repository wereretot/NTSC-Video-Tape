#pragma once
#include "constants.h"
#include "processing_pipeline.h"
#include "capture_thread.h"
#include "sdl_utils.h"
#include "video_effects.h"
#include "engine.hpp"
#include "audio_io.hpp"
#include <functional>
#include <string>
#include <atomic>

struct AppState {
    CaptureThread      capture;
    ProcessingPipeline pipeline;
    ProcessParams      pp;
    SDLTexture         outTex, rawTex;
    std::mutex         frameMu;
    cv::Mat            lastOut, lastRaw;
    std::mutex         fileMu;
    std::string        filePath;
    std::mutex         audioMu;
    std::unique_ptr<TapeEngine> tapeEngine;
    std::unique_ptr<AudioIO>    audioIO;
    std::string        audioWavPath;
    ExportContext      exportCtx;
    FrameQueue<std::vector<float>> audioCaptureQ;
    FrameQueue<cv::Mat> recordQ;

    int   selectedFx = 0, tapeSpeedIdx = 0;
    bool  ntscEnabled = true;
    float vuL = 0.f, vuR = 0.f;
    uint64_t startMs = SDL_GetTicks64();
    VideoParams videoParams;
    EngineParams baseParams;
    float av_sync_offset_ms{0.f};

    std::atomic<int> currentRecordingId{0};
    std::atomic<int> pipelineGeneration{0};

    void setParam(std::function<void(EngineParams&)> fn) {
        fn(baseParams);
    }

    void switchToNewRecording() {
        ++currentRecordingId;
        ++pipelineGeneration;
    }
};
