#include "sdl_utils.h"

void SDLTexture::create(SDL_Renderer* r, int W, int H) {
    if (tex) SDL_DestroyTexture(tex);
    w = W;
    h = H;
    tex = SDL_CreateTexture(r, SDL_PIXELFORMAT_BGR24, SDL_TEXTUREACCESS_STREAMING, W, H);
}

void SDLTexture::update(const cv::Mat& f) {
    if (!tex || f.empty()) return;
    cv::Mat rs;
    if (f.cols != w || f.rows != h) cv::resize(f, rs, {w, h});
    else rs = f;
    void* px;
    int pitch;
    SDL_LockTexture(tex, nullptr, &px, &pitch);
    for (int y = 0; y < h; ++y) memcpy((uchar*)px + y * pitch, rs.ptr(y), w * 3);
    SDL_UnlockTexture(tex);
}

void SDLTexture::drawLetterbox(SDL_Renderer* renderer, int x, int y, int w, int h, float targetAR) {
    float windowAR = float(w) / float(h);
    int rw, rh, rx, ry;
    if (windowAR > targetAR) {
        rh = h;
        rw = int(h * targetAR);
        rx = x + (w - rw) / 2;
        ry = y;
    } else {
        rw = w;
        rh = int(w / targetAR);
        rx = x;
        ry = y + (h - rh) / 2;
    }
    SDL_Rect dst = {rx, ry, rw, rh};
    SDL_RenderCopy(renderer, tex, nullptr, &dst);
}

SDLTexture::~SDLTexture() {
    if (tex) SDL_DestroyTexture(tex);
}
