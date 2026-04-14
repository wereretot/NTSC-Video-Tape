#include "capture_thread.h"
#include <SDL2/SDL.h>

cv::Mat DemoSource::generate(int w, int h) {
    cv::Mat f(h, w, CV_8UC3);
    phase_ += .016f;
    static const cv::Scalar bars[] = {
        {192, 192, 192}, {192, 192, 0}, {0, 192, 192},
        {0, 192, 0}, {192, 0, 192}, {192, 0, 0}, {0, 0, 192}
    };
    int bw = w / 7;
    for (int i = 0; i < 7; ++i)
        cv::rectangle(f, {i * bw, 0}, {(i + 1) * bw, (int)(h * .75)}, bars[i], -1);
    cv::rectangle(f, {0, (int)(h * .75)}, {(int)(w * .16), h}, {192, 0, 0}, -1);
    cv::rectangle(f, {(int)(w * .16), (int)(h * .75)}, {(int)(w * .26), h}, {255, 255, 255}, -1);
    cv::rectangle(f, {(int)(w * .26), (int)(h * .75)}, {(int)(w * .36), h}, {192, 0, 192}, -1);
    cv::rectangle(f, {(int)(w * .36), (int)(h * .75)}, {(int)(w * .66), h}, {12, 12, 12}, -1);
    int cx = (int)(w * .5f + std::sin(phase_) * w * .2f);
    int cy = (int)(h * .45f + std::cos(phase_ * .7f) * h * .12f);
    cv::circle(f, {cx, cy}, 36, {255, 255, 255}, -1);
    cv::circle(f, {cx, cy}, 30, {0, 200, 64}, -1);
    using namespace std::chrono;
    long ms = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
    char tc[48];
    std::snprintf(tc, sizeof(tc), "ADO-8500  %02ld:%02ld:%02ld:%02ld",
        (ms / 3600000) % 24, (ms / 60000) % 60, (ms / 1000) % 60, (ms % 1000) / 33);
    cv::rectangle(f, {4, 4}, {286, 26}, {0, 0, 0}, -1);
    cv::putText(f, tc, {8, 20}, cv::FONT_HERSHEY_PLAIN, 1., {0, 255, 64}, 1, cv::LINE_AA);
    return f;
}

std::string extractAudioToWav(const std::string& vp) {
    static std::atomic<int> counter{0};
    std::string wav = "temp_ado_" + std::to_string(SDL_GetTicks64()) + "_" + std::to_string(counter++) + ".wav";
    char cmd[2048];
    std::snprintf(cmd, sizeof(cmd),
        "ffmpeg -y -i \"%s\" -vn -acodec pcm_s16le -ar %d -ac 2 \"%s\" 2>/dev/null",
        vp.c_str(), AUDIO_SR, wav.c_str());
    if (std::system(cmd) != 0) return "";
    FILE* f = std::fopen(wav.c_str(), "rb");
    if (!f) return "";
    std::fseek(f, 0, SEEK_END);
    long sz = std::ftell(f);
    std::fclose(f);
    return sz >= 44 ? wav : "";
}

void CaptureThread::start(FrameQueue<cv::Mat>& q) {
    running_ = true;
    thread_ = std::thread([this, &q] { loop(q); });
}

void CaptureThread::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

float CaptureThread::fps() const {
    return fps_.load();
}

void CaptureThread::loop(FrameQueue<cv::Mat>& outQ) {
    cv::VideoCapture cap;
    SourceType lastSrc = SourceType::Demo;
    std::string lastFile;
    auto tPrev = std::chrono::steady_clock::now();
    int fpsCount = 0;
    double srcFPS = kNTSC_FPS;
    auto wallClock = std::chrono::steady_clock::now();

    while (running_) {
        SourceType src = sourceType.load();
        std::string fp;
        if (fileMuPtr && filePathPtr) {
            std::lock_guard<std::mutex> lk(*fileMuPtr);
            fp = *filePathPtr;
        }

        if (src != lastSrc || (src == SourceType::File && fp != lastFile)) {
            cap.release();
            if (src == SourceType::Camera) {
                cap.open(0);
                if (!cap.isOpened()) {
                    sourceType.store(SourceType::Demo);
                    src = SourceType::Demo;
                } else {
                    srcFPS = cap.get(cv::CAP_PROP_FPS);
                    if (srcFPS <= 0) srcFPS = kNTSC_FPS;
                    g_sourceFPS.store(float(srcFPS));
                    g_videoReset.store(true);
                }
            } else if (src == SourceType::File && !fp.empty()) {
                std::string wav = extractAudioToWav(fp);
                if (!running_.load()) return;

                cap.open(fp);
                if (!cap.isOpened()) {
                    if (fileMuPtr) {
                        std::lock_guard<std::mutex> lk(*fileMuPtr);
                    }
                    sourceType.store(SourceType::Demo);
                    src = SourceType::Demo;
                } else {
                    if (!running_.load()) return;
                    if (!wav.empty() && onAudioFileReady) onAudioFileReady(wav);

                    outQ.clear();
                    g_pipelineFlush.store(true);
                    g_videoReset.store(true);

                    srcFPS = cap.get(cv::CAP_PROP_FPS);
                    if (srcFPS <= 0) srcFPS = kNTSC_FPS;
                    g_sourceFPS.store(float(srcFPS));
                    g_videoReset.store(true);
                }
            }
            lastSrc = src;
            lastFile = fp;
            wallClock = std::chrono::steady_clock::now();
        }

        if (g_videoReset.exchange(false)) {
            if (cap.isOpened()) cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            g_audioSamplePos.store(0);
        }

        cv::Mat frame;
        if (src == SourceType::Demo) {
            auto now = std::chrono::steady_clock::now();
            float el = std::chrono::duration<float, std::milli>(now - wallClock).count();
            float iv = 1000.f / float(srcFPS);
            if (el < iv) std::this_thread::sleep_for(std::chrono::microseconds(int((iv - el) * 1000)));
            wallClock = std::chrono::steady_clock::now();
            frame = demo_.generate(FW, FH);
        } else if (cap.isOpened()) {
            int64_t apos = g_audioSamplePos.load();
            int64_t latencyComp = 2048;
            int64_t adjApos = std::max((int64_t)0, apos - latencyComp);
            double spf = AUDIO_SR / srcFPS;
            int64_t targetFrame = int64_t(double(adjApos) / spf);
            int64_t curFrame = int64_t(cap.get(cv::CAP_PROP_POS_FRAMES));
            int64_t delta = targetFrame - curFrame;
            if (delta > 4) cap.set(cv::CAP_PROP_POS_FRAMES, double(targetFrame));
            else if (delta < -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(4));
                continue;
            }
            if (!cap.read(frame) || frame.empty()) {
                cap.release();
                g_videoEnded.store(true);
                continue;
            }
            cv::resize(frame, frame, {FW, FH});
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        outQ.push(frame);
        ++fpsCount;
        auto now2 = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now2 - tPrev).count();
        if (dt >= 1.f) {
            fps_ = fpsCount / dt;
            fpsCount = 0;
            tPrev = now2;
        }
    }
}
