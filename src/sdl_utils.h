#pragma once
#include <SDL2/SDL.h>
#include "constants.h"

struct SDLTexture {
    SDL_Texture* tex = nullptr;
    int w = 0, h = 0;

    void create(SDL_Renderer* r, int W, int H);
    void update(const cv::Mat& f);
    void drawLetterbox(SDL_Renderer* renderer, int x, int y, int w, int h, float targetAR);
    ~SDLTexture();
};
