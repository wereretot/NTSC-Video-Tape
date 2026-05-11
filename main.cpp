#include "include/common.h"
#include "src/app_state.h"
#include "src/processing_pipeline.h"
#include "src/capture_thread.h"
#include "src/sdl_utils.h"
#include "src/ui.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <thread>

int main(int, char**) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_AUDIO) < 0) {
        SDL_Log("SDL_Init: %s", SDL_GetError());
        return 1;
    }
    SDL_Window* win = SDL_CreateWindow(
        "ADO-8500  ·  BROADCAST VIDEO EFFECTS PROCESSOR",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1520, 900, SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!win) {
        SDL_Log("Window: %s", SDL_GetError());
        return 1;
    }
    SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);
    if (!ren) {
        SDL_Log("Renderer: %s", SDL_GetError());
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.IniFilename = nullptr;
    io.Fonts->AddFontDefault();
    styleBroadcast();
    ImGui_ImplSDL2_InitForSDLRenderer(win, ren);
    ImGui_ImplSDLRenderer2_Init(ren);

    AppState app;
    app.outTex.create(ren, FW, FH);
    app.rawTex.create(ren, FW / 2, FH / 2);

    app.capture.onAudioFileReady = [&app](const std::string& wav) {
        auto eng = std::make_unique<TapeEngine>(42);
        if (!eng->load_file(wav)) {
            SDL_Log("TapeEngine: Failed to load %s", wav.c_str());
            return;
        }

        auto aio = std::make_unique<AudioIO>(*eng);
        if (!aio->open()) {
            SDL_Log("AudioIO: Failed to open");
            return;
        }
        aio->play_forward();
        eng->is_playing.store(true);

        aio->set_capture_callback([&app](const std::vector<float>& samples) {
            app.audioCaptureQ.push(samples);
        });

        {
            std::lock_guard<std::mutex> lk(app.audioMu);
            if (app.audioIO) {
                app.audioIO->stop();
                app.audioIO->close();
            }

            {
                std::lock_guard<std::mutex> flk(app.fileMu);
                app.audioWavPath = wav;
            }
            app.tapeEngine = std::move(eng);
            app.audioIO = std::move(aio);
            g_audioSamplePos.store(0);
            app.baseParams = app.tapeEngine->params;

            {
                std::lock_guard<std::mutex> flk(app.frameMu);
                app.lastOut = cv::Mat{};
                app.lastRaw = cv::Mat{};
            }

            app.switchToNewRecording();

            std::lock_guard<std::mutex> lk2(app.pp.mu);
            app.pp.epSnap = app.baseParams;
            app.pp.tapeSpd = app.baseParams.tape_speed_mult;
            app.pp.recording_id = app.currentRecordingId.load();
            app.pp.engineGeneration = app.pipelineGeneration.load();
            app.pp.engValid.store(true);
        }
        SDL_Log("Audio: engine started, %d samples, recording_id %d", 
                app.tapeEngine->total_samples, app.currentRecordingId.load());
    };

    app.capture.fileMuPtr = &app.fileMu;
    app.capture.filePathPtr = &app.filePath;
    app.pp.exPtr = &app.exportCtx;

    app.pp.onRawFrame = [&app](const cv::Mat& f) {
        std::lock_guard<std::mutex> lk(app.frameMu);
        app.lastRaw = f;
    };

    app.capture.start(app.pipeline.rawQ);
    app.pipeline.start(app.pp);

    std::atomic<bool> drainRun{true};
    std::thread drainThread([&] {
        cv::Mat out;
        while (drainRun) {
            if (app.pipeline.outQ.pop(out, 10)) {
                if (app.exportCtx.active && !app.exportCtx.isClosing) {
                    app.recordQ.push(out.clone());
                }
                {
                    std::lock_guard<std::mutex> lk(app.frameMu);
                    app.lastOut = out;
                }
            }
        }
    });

    std::atomic<bool> recordRun{true};
    std::thread recordThread([&] {
        cv::Mat f;
        std::vector<float> residue;
        while (recordRun) {
            if (app.recordQ.pop(f, 20)) {
                std::lock_guard<std::mutex> wlk(app.exportCtx.writerMu);
                if (app.exportCtx.writer.isOpened()) {
                    app.exportCtx.writer.write(f);
                    app.exportCtx.frameCount++;

                    if (app.exportCtx.audioFp) {
                        double totalExpected = (double)app.exportCtx.frameCount * (SR / kNTSC_FPS);
                        int toWriteTotal = (int)totalExpected - (int)app.exportCtx.audioFramesWritten;

                        if (toWriteTotal > 0) {
                            std::vector<int16_t> pcm;
                            pcm.reserve(toWriteTotal * 2);
                            int fromRes = std::min((int)residue.size() / 2, toWriteTotal);
                            for (int i = 0; i < fromRes * 2; ++i) {
                                float s = residue[i];
                                if (s > 1.f) s = 1.f;
                                if (s < -1.f) s = -1.f;
                                pcm.push_back((int16_t)(s * 32767.f));
                            }
                            residue.erase(residue.begin(), residue.begin() + fromRes * 2);
                            int remaining = toWriteTotal - fromRes;
                            std::vector<float> block;
                            while (remaining > 0 && app.audioCaptureQ.try_pop(block)) {
                                int fromBlock = std::min((int)block.size() / 2, remaining);
                                for (int i = 0; i < fromBlock * 2; ++i) {
                                    float s = block[i];
                                    if (s > 1.f) s = 1.f;
                                    if (s < -1.f) s = -1.f;
                                    pcm.push_back((int16_t)(s * 32767.f));
                                }
                                if (fromBlock * 2 < (int)block.size()) {
                                    residue.insert(residue.end(), block.begin() + fromBlock * 2, block.end());
                                }
                                remaining -= fromBlock;
                            }
                            if (remaining > 0) {
                                for (int i = 0; i < remaining * 2; ++i) pcm.push_back(0);
                            }
                            std::fwrite(pcm.data(), sizeof(int16_t), pcm.size(), app.exportCtx.audioFp);
                            app.exportCtx.audioFramesWritten += toWriteTotal;
                            if (residue.size() > 44100 * 2) residue.clear();
                        }
                    }
                }
            } else if (app.exportCtx.isClosing && app.recordQ.size() == 0) {
                app.exportCtx.active = false;
            }
        }
    });

    bool running = true;
    SDL_Event evt;
    auto lastPoll = std::chrono::steady_clock::now();
    double wallTimeAccum = 0.0;

    while (running) {
        {
            auto now = std::chrono::steady_clock::now();
            double dt = std::chrono::duration<double>(now - lastPoll).count();
            wallTimeAccum += dt;
            g_wallTimeSec.store(wallTimeAccum);
            lastPoll = now;

            std::lock_guard<std::mutex> a_lk(app.audioMu);
            bool engineRunning = app.tapeEngine && app.audioIO && app.audioIO->is_open();
            
            EngineParams active = app.baseParams;

            float tracking_deg = std::clamp(app.videoParams.tracking_error, 0.0f, 1.0f);
                float dropout_deg = std::clamp(app.baseParams.dropout_rate * 10.0f, 0.0f, 1.0f);
                float motor_deg = std::clamp(app.baseParams.motor_health, 0.0f, 1.0f);
                float crease_deg = std::clamp(app.videoParams.tape_crease, 0.0f, 1.0f);
                float oxide_deg = std::clamp(app.baseParams.oxide_shedding, 0.0f, 1.0f);
                float demag_deg = std::clamp(app.baseParams.demagnetization, 0.0f, 1.0f);
                float sticky_deg = std::clamp(app.baseParams.sticky_shed, 0.0f, 1.0f);

                float rf_level = 1.0f - tracking_deg * 0.7f
                                        - dropout_deg * 0.3f
                                        - oxide_deg * 0.4f
                                        - crease_deg * 0.5f;

                float chroma_atten = 1.0f - tracking_deg * 0.6f
                                         - motor_deg * 0.3f
                                         - demag_deg * 0.4f;

                float luma_noise = tracking_deg * 0.5f
                                 + dropout_deg * 0.3f
                                 + oxide_deg * 0.4f
                                 + sticky_deg * 0.2f;

                float chroma_noise = tracking_deg * 0.6f
                                  + motor_deg * 0.4f
                                  + demag_deg * 0.5f;

                float dropout_intensity = dropout_deg * 0.8f
                                       + oxide_deg * 0.6f
                                       + tracking_deg * 0.4f;

                float signal_loss = tracking_deg * 0.6f
                                + dropout_deg * 0.3f
                                + oxide_deg * 0.4f
                                + crease_deg * 0.5f
                                + sticky_deg * 0.3f;

                static float signal_lpf = 1.0f;
                signal_lpf += (1.0f - signal_loss - signal_lpf) * 0.1f;
                float signal_strength = std::max(0.05f, signal_lpf);

                static float hifi_blend_lpf = 0.0f;
                hifi_blend_lpf += (signal_loss - hifi_blend_lpf) * 0.04f;
                float hifi_blend = hifi_blend_lpf;

                if (hifi_blend > 0.05f) {
                    float buzz_t = (float)g_audioSamplePos.load() / AUDIO_SR;
                    float buzz_env = hifi_blend * (1.0f - hifi_blend) * 4.0f;
                    active.print_through += buzz_env * 0.15f;
                    active.hiss += buzz_env * 0.006f;
                    float buzz_phase = std::fmod(buzz_t * 60.0f, 1.0f);
                    if (buzz_phase > 0.85f)
                        active.dropout_rate = std::min(active.dropout_rate + buzz_env * 0.015f, 0.09f);
                }

                if (hifi_blend > 0.1f) {
                    float t = std::clamp((hifi_blend - 0.1f) / 0.9f, 0.0f, 1.0f);
                    active.azimuth_drift = std::max(active.azimuth_drift, t);
                    float linear_cutoff = 8000.0f - t * 3000.0f;
                    active.cutoff_base = std::min(active.cutoff_base, linear_cutoff);
                    active.hiss += t * 0.010f;
                }

                if (tracking_deg > 0.05f) {
                    active.hiss += tracking_deg * 0.015f;
                    active.cutoff_base *= std::max(0.02f, 1.0f - tracking_deg * 0.95f);
                    active.azimuth_drift += tracking_deg;
                    active.dropout_rate = std::min(active.dropout_rate + tracking_deg * 0.05f, 0.09f);
                }

                if (crease_deg > 0.01f) {
                    float t_sec = (float)g_audioSamplePos.load() / AUDIO_SR;
                    float cp = std::fmod(t_sec * 0.1f, 1.0f);
                    if (cp > 0.4f && cp < 0.6f) {
                        active.dropout_rate += crease_deg * 0.5f;
                        active.flutter_dep += crease_deg * 2.0f;
                    }
                }

                if (motor_deg > 0.05f) {
                    active.wow_dep += motor_deg * 3.0f;
                    active.flutter_dep += motor_deg * 0.5f;
                    active.dropout_rate += motor_deg * 0.03f;
                }

                if (sticky_deg > 0.01f) {
                    active.cutoff_base *= std::max(0.1f, 1.0f - sticky_deg * 0.7f);
                    active.hiss += sticky_deg * 0.008f;
                }

                if (demag_deg > 0.01f) {
                    active.print_through += demag_deg * 0.3f;
                    active.hiss += demag_deg * 0.005f;
                }

                float spd = 1.f;
                float instantSpd = 1.f;
                
                if (engineRunning) {
                    std::lock_guard<std::mutex> lk(app.tapeEngine->lock);
                    float inertia_ramp = app.audioIO->current_speed_mult();
                    spd = inertia_ramp;

                    float m_eng = app.tapeEngine->params.motor_engage;
                    bool  m_rev = app.tapeEngine->params.is_reversed;
                    app.tapeEngine->params = active;
                    app.tapeEngine->params.tape_speed_mult = spd;
                    app.tapeEngine->params.motor_engage    = m_eng;
                    app.tapeEngine->params.is_reversed     = m_rev;

                    spd = std::clamp(spd, .01f, 4.f);
                    
                    g_audioSamplePos.store(int64_t(app.tapeEngine->play_head));
                    int64_t total = app.tapeEngine->total_samples;
                    if (total > 0 && g_audioSamplePos.load() > total) g_audioSamplePos.store(0);
                    
                    instantSpd = app.tapeEngine->transport.last_instant_speed;
                }
                
                {
                    std::lock_guard<std::mutex> lk(app.pp.mu);
                    app.pp.epSnap = active;
                    app.pp.vpSnap = app.videoParams;
                    app.pp.vpSnap.signal_strength = signal_strength;
                    app.pp.vpSnap.base_rf_level = rf_level;
                    app.pp.vpSnap.chroma_level = chroma_atten;
                    app.pp.vpSnap.luma_noise = luma_noise;
                    app.pp.vpSnap.chroma_noise = chroma_noise;
                    app.pp.vpSnap.dropout_intensity = dropout_intensity;
                    app.pp.vpSnap.dropout_rate = app.baseParams.dropout_rate;
                    app.pp.vpSnap.motor_health = app.baseParams.motor_health;
                    app.pp.vpSnap.oxide_shedding = app.baseParams.oxide_shedding;
                    app.pp.vpSnap.demagnetization = app.baseParams.demagnetization;
                    app.pp.vpSnap.sticky_shed = app.baseParams.sticky_shed;
                    app.pp.vpSnap.tape_age = 0.0f;
                    app.pp.av_sync_offset_ms = app.av_sync_offset_ms;
                    app.pp.ntscEnabled = app.ntscEnabled;
                    app.pp.tapeSpd = spd;
                    app.pp.instantSpd = instantSpd;
                    app.pp.recording_id = app.currentRecordingId.load();
                    app.pp.engValid.store(engineRunning);
                    if (engineRunning) app.pp.exPtr->tapeEnginePtr = app.tapeEngine.get();
                }
        }

        if (g_videoEnded.exchange(false)) {
            g_sourceFPS.store(kNTSC_FPS);
        }

        {
            std::lock_guard<std::mutex> lk(app.frameMu);
            // DONT overwrite lastRaw!
        }

        while (SDL_PollEvent(&evt)) {
            ImGui_ImplSDL2_ProcessEvent(&evt);
            if (evt.type == SDL_QUIT) running = false;
            if (evt.type == SDL_WINDOWEVENT && evt.window.event == SDL_WINDOWEVENT_CLOSE
                && evt.window.windowID == SDL_GetWindowID(win)) running = false;
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        drawUI(app, ren);
        ImGui::Render();
        SDL_SetRenderDrawColor(ren, 8, 8, 8, 255);
        SDL_RenderClear(ren);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(ren);
    }

    drainRun = false;
    drainThread.join();
    app.pipeline.stop();
    app.capture.stop();
    if (app.audioIO) {
        app.audioIO->stop();
        app.audioIO->close();
    }
    if (!app.audioWavPath.empty()) std::remove(app.audioWavPath.c_str());
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 0;
}
