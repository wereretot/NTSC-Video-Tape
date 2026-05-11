#pragma once
#include <string>
#include <cstdint>

struct AppState;

void startGlobalAudioCapture(AppState& app, const std::string& path);
void stopGlobalAudioCapture(AppState& app);
void writeWavHeader(FILE* f, uint32_t total_frames);
