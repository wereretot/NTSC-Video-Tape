#include "processing_pipeline.h"
#include "engine.hpp"
#include <SDL2/SDL.h>
#include <chrono>

std::atomic<int64_t> g_audioSamplePos{0};
std::atomic<float>   g_sourceFPS{kNTSC_FPS};
std::atomic<bool>    g_videoReset{false};
std::atomic<bool>    g_pipelineFlush{false};
std::atomic<bool>    g_videoEnded{false};

void ProcessingPipeline::start(ProcessParams& pp) {
    running_ = true;
    thread_ = std::thread([this, &pp] { loop(pp); });
}

void ProcessingPipeline::stop() {
    running_ = false;
    rawQ.push(cv::Mat());
    if (thread_.joinable()) thread_.join();
}

float ProcessingPipeline::fps() const {
    return fps_.load();
}

void ProcessingPipeline::loop(ProcessParams& pp) {
    auto tPrev = std::chrono::steady_clock::now();
    int fpsCount = 0;
    ntsc_.initBuffers(FW, FH);
    cv::Mat last_raw;
    auto loop_timer = std::chrono::steady_clock::now();
    auto wallPrev = std::chrono::steady_clock::now();
    int last_recording_id = -1;
    
    // Debug logging
    static int debug_frame_count = 0;
    static auto debug_start_time = std::chrono::steady_clock::now();

    while (running_) {
        debug_frame_count++;
        auto frame_start = std::chrono::steady_clock::now();
        
        if (g_pipelineFlush.exchange(false)) {
            rawQ.clear();
            outQ.clear();
            last_raw = cv::Mat{};
            frameNum_ = 0;
        }

        int64_t apos = g_audioSamplePos.load();
        float sf = g_sourceFPS.load();

        auto now_time = std::chrono::steady_clock::now();
        long long elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now_time - loop_timer).count();
        float target_frame_ms = 1000.f / std::max(15.f, g_sourceFPS.load());
        if (elapsed < static_cast<long long>(target_frame_ms - 1.f)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        loop_timer = now_time;
        
        cv::Mat raw;
        if (g_sourceFPS.load() > 30.0f) {
            size_t qsz = rawQ.size();
            while (qsz > 2) {
                cv::Mat discard;
                if (rawQ.try_pop(discard)) --qsz;
                else break;
            }
            if (qsz > 2) {
                if (last_raw.empty()) continue;
                raw = last_raw;
            }
        }

        if (raw.empty() && !rawQ.try_pop(raw)) {
            if (last_raw.empty()) continue;
            raw = last_raw;
        } else {
            if (raw.empty()) continue;

            int sw = raw.cols, sh = raw.rows;
            float srcAR = float(sw) / float(sh);
            constexpr float kAR43 = 4.f / 3.f;
            if (std::abs(srcAR - kAR43) > 0.02f) {
                int cw, ch;
                if (srcAR > kAR43) { ch = sh; cw = int(sh * kAR43); }
                else { cw = sw; ch = int(sw / kAR43); }
                int ox = std::clamp((sw - cw) / 2, 0, sw - 1);
                int oy = std::clamp((sh - ch) / 2, 0, sh - 1);
                cw = std::clamp(cw, 1, sw - ox);
                ch = std::clamp(ch, 1, sh - oy);
                raw = raw(cv::Rect(ox, oy, cw, ch)).clone();
            }
            last_raw = raw;
        }

        if (pp.onRawFrame && !raw.empty()) {
            pp.onRawFrame(raw.clone());
        }

        Effects::Params fx;
        bool ntscOn;
        EngineParams ep{};
        VideoParams vp{};
        float spd = 1.f, iSpd = 1.f;
        bool hasEng = false;
        int si = 0;
        int recording_id = 0;
        int engine_gen = 0;
        {
            std::lock_guard<std::mutex> lk(pp.mu);
            fx = pp.fx;
            ntscOn = pp.ntscEnabled.load();
            ep = pp.epSnap;
            vp = pp.vpSnap;
            spd = pp.tapeSpd;
            iSpd = pp.instantSpd;
            hasEng = pp.engValid;
            si = pp.spdIdx.load();
            recording_id = pp.recording_id;
            engine_gen = pp.engineGeneration.load();
        }

        if (recording_id != last_recording_id) {
            ntsc_.setRecordingId(recording_id);
            last_recording_id = recording_id;
        }

        if (!hasEng) {
            switch (si) {
                case 1: ep.ips_base = 7.5f; break;
                case 2: ep.ips_base = 3.75f; break;
                default: ep.ips_base = 15.f;
            }
        }
        static float ph = 0.f;
        ph += .01f;
        if (ph > 1.f) ph -= 1.f;
        fx.phase = ph;
        cv::Mat eff, proc;
        auto effects_start = std::chrono::steady_clock::now();
        Effects::apply(raw, eff, fx);
        auto effects_end = std::chrono::steady_clock::now();

        float effectDegradation = Effects::calcSignalDegradation(fx);
        float combinedSignalStrength = vp.signal_strength * (1.0f - effectDegradation);
        vp.signal_strength = std::max(0.15f, combinedSignalStrength);

        auto wallNow = std::chrono::steady_clock::now();
        float wallDt = std::chrono::duration<float>(wallNow - wallPrev).count();
        wallPrev = wallNow;
        if (wallDt < 0.001f) wallDt = kNTSC_FRAME_S;

        auto ntsc_start = std::chrono::steady_clock::now();
        if (ntscOn) {
            if (!hasEng) {
                EngineParams def{};
                def.ips_base = 15.f;
                def.hiss = .003f;
                def.wow_dep = .3f;
                ntsc_.tapeTime_ = (float)frameNum_ / kNTSC_FPS;
                ntsc_.process(eff, proc, frameNum_, ep, vp, 1.f, 1.f, wallDt, recording_id);
            } else {
                float syncOffset = pp.av_sync_offset_ms;
                int64_t latencyComp = int64_t(syncOffset * AUDIO_SR / 1000.0f);
                int64_t adjApos = std::max((int64_t)0, g_audioSamplePos.load() - latencyComp);

                float histSpd = spd;

                std::lock_guard<std::mutex> lk(pp.mu);
                if (pp.engineGeneration.load() == engine_gen && pp.exPtr && pp.exPtr->tapeEnginePtr) {
                    uint64_t idx = uint64_t(adjApos) % TransportDynamics::SPEED_HIST_SIZE;
                    histSpd = pp.exPtr->tapeEnginePtr->transport.speed_history[idx];
                    if (!std::isfinite(histSpd)) histSpd = spd;
                }

                ntsc_.tapeTime_ = (float)adjApos / AUDIO_SR;
                ntsc_.process(eff, proc, frameNum_, ep, vp, spd, histSpd, wallDt, recording_id);
            }
        } else {
            proc = eff;
        }
        auto ntsc_end = std::chrono::steady_clock::now();
        
        // Debug logging every 30 frames
        if (debug_frame_count % 30 == 0) {
            float effects_ms = std::chrono::duration<float, std::milli>(effects_end - effects_start).count();
            float ntsc_ms = std::chrono::duration<float, std::milli>(ntsc_end - ntsc_start).count();
            float total_elapsed = std::chrono::duration<float>(std::chrono::steady_clock::now() - debug_start_time).count();
            SDL_Log("[Pipeline] Frame %d - Effects: %.1fms, NTSC: %.1fms, Total time: %.1fs, FPS: %.2f, raw size: %dx%d",
                debug_frame_count, effects_ms, ntsc_ms, total_elapsed, 
                float(debug_frame_count) / total_elapsed,
                raw.cols, raw.rows);
        }

        outQ.push(std::move(proc));
        ++frameNum_;
        ++fpsCount;
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - tPrev).count();
        if (dt >= 1.f) {
            fps_ = fpsCount / dt;
            fpsCount = 0;
            tPrev = now;
        }
    }
}
