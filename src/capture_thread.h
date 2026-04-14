#pragma once
#include "constants.h"
#include "processing_pipeline.h"
#include <atomic>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

enum class SourceType { Demo = 0, Camera, File };

class DemoSource {
public:
    cv::Mat generate(int w, int h);

private:
    float phase_ = 0.f;
};

class CaptureThread {
public:
    std::atomic<SourceType> sourceType{SourceType::Demo};
    std::mutex*  fileMuPtr = nullptr;
    std::string* filePathPtr = nullptr;
    std::function<void(const std::string&)> onAudioFileReady;

    void start(FrameQueue<cv::Mat>& q);
    void stop();
    float fps() const;

private:
    std::atomic<bool>  running_{false};
    std::thread        thread_;
    std::atomic<float> fps_{0.f};
    DemoSource         demo_;

    void loop(FrameQueue<cv::Mat>& outQ);
};

std::string extractAudioToWav(const std::string& vp);
