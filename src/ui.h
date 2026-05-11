#pragma once
#include "constants.h"
#include "processing_pipeline.h"
#include "sdl_utils.h"
#include <string>
#include "imgui.h"
#include <SDL2/SDL.h>

struct AppState;


void styleBroadcast();
void sectionHdr(const char* lbl);
void vuBar(float lv, float w, float h, bool clip = false);
void monitorFrame(SDL_Texture* tex, float w, float h, const char* label, ImU32 ledCol);
std::string timecode(uint64_t s0);
void drawUI(AppState& app, SDL_Renderer* ren);
