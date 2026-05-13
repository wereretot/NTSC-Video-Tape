#pragma once
#include "constants.h"
#include "ntsc_simulator.h"
#include "video_effects.h"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

struct TapeEngine;

enum class ExportFormat { H264_MP4, FFV1_MKV, FFV1_AVI };

struct ExportContext {
    std::atomic<bool> active{false};
    ExportFormat      format = ExportFormat::H264_MP4;
    std::string       path;
    cv::VideoWriter   writer;
    std::atomic<int>  frameCount{0};
    std::string       tempVideoPath;
    std::string       tempAudioPath;
    std::mutex        writerMu;
    FILE*             audioFp = nullptr;
    uint32_t          audioFramesWritten = 0;
    std::atomic<bool> isClosing{false};
    TapeEngine*       tapeEnginePtr = nullptr;
};

struct ProcessParams {
    std::mutex        mu;
    Effects::Params   fx;
    std::atomic<bool> ntscEnabled{true};
    std::function<void(const cv::Mat&)> onRawFrame;
    EngineParams      epSnap;
    VideoParams       vpSnap;
    float             tapeSpd{1.f};
    float             instantSpd{1.f};
    std::atomic<bool> engValid{false};
    std::atomic<int>  spdIdx{0};
    float             av_sync_offset_ms{0.f};
    ExportContext*    exPtr = nullptr;
    int               recording_id{0};
    std::atomic<int>  engineGeneration{0};
};

template<typename T>
class FrameQueue {
public:
    explicit FrameQueue(size_t cap = Q_DEPTH) : cap_(cap) {}
    void push(T v) {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (q_.size() >= cap_) q_.pop_front();
            q_.push_back(std::move(v));
        }
        cv_.notify_one();
    }
    bool pop(T& out, int ms = 50) {
        std::unique_lock<std::mutex> lk(mu_);
        if (!cv_.wait_for(lk, std::chrono::milliseconds(ms), [this] { return !q_.empty(); }))
            return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }
    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lk(mu_);
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop_front();
        return true;
    }
    void clear() {
        std::lock_guard<std::mutex> lk(mu_);
        q_.clear();
    }
    size_t size() const {
        std::lock_guard<std::mutex> lk(mu_);
        return q_.size();
    }

private:
    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::deque<T> q_;
    size_t cap_;
};

class ProcessingPipeline {
public:
    FrameQueue<cv::Mat> rawQ{Q_DEPTH};
    FrameQueue<cv::Mat> outQ{Q_DEPTH};
    NTSCSimulator       ntsc_;

    void start(ProcessParams& pp);
    void stop();
    float fps() const;

private:
    std::atomic<bool>  running_{false};
    std::thread        thread_;
    std::atomic<float> fps_{0.f};
    int                frameNum_{0};

    void loop(ProcessParams& pp);
};

extern std::atomic<int64_t> g_audioSamplePos;
extern std::atomic<float>   g_sourceFPS;
extern std::atomic<bool>    g_videoReset;
extern std::atomic<bool>    g_pipelineFlush;
extern std::atomic<bool>    g_videoEnded;
