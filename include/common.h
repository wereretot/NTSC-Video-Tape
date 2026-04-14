#pragma once

#define _CMATH_
#include "engine.hpp"
#include "audio_io.hpp"
#include "dsp_types.hpp"
#undef _CMATH_

#include <SDL2/SDL.h>
#include <SDL2/SDL_audio.h>
#include "imgui.h"
#include "imgui_impl_sdl2.h"
#include "imgui_impl_sdlrenderer2.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/samplefmt.h>
#include <libswresample/swresample.h>
}

#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#ifdef ADO_OPENMP
#  include <omp.h>
#endif

struct AppState;
void startGlobalAudioCapture(AppState& app, const std::string& path);
void stopGlobalAudioCapture(AppState& app);
void writeWavHeader(FILE* f, uint32_t total_frames);
