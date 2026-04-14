#include "video_effects.h"

namespace Effects {

static cv::Mat s_trail;

static void fx_none(const cv::Mat& s, cv::Mat& d, const Params&) {
    s.copyTo(d);
}

static void fx_tumble(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    float ang = p.phase * kPI * 2.f, cA = std::cos(ang), sA = std::sin(ang);
    float scX = std::abs(cA);
    if (scX < .02f) scX = .02f;
    bool bk = cA < 0.f;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        float fy = dsp::clamp(float(y) - sA * 20.f, 0.f, float(H - 1));
        for (int x = 0; x < W; ++x) {
            float sx = (x - W * .5f) / scX + W * .5f;
            if (bk) sx = W - 1 - sx;
            auto px = dsp::bsample(s, sx, fy);
            float sh = 1.f - (1.f - scX) * p.depth * .6f;
            if (bk) sh *= .5f;
            dr[x * 3 + 0] = dsp::clamp8(px[0] * sh);
            dr[x * 3 + 1] = dsp::clamp8(px[1] * sh);
            dr[x * 3 + 2] = dsp::clamp8(px[2] * sh);
        }
    }
}

static void fx_pageturn(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    s.copyTo(d);
    float cx2 = W - (std::sin(p.phase * kPI * 2.f) * .5f + .5f) * W * p.amount;
    int cx = int(cx2);
    if (cx >= W) return;
    cv::rectangle(d, {cx, 0}, {W - 1, H - 1}, {16, 16, 16}, -1);
    for (int x = std::max(0, cx - 24); x < std::min(W, cx + 6); ++x) {
        float tt = float(x - (cx - 24)) / 30.f, hl = std::sin(tt * kPI) * 75.f;
        for (int y = 0; y < H; ++y) {
            uchar* p2 = d.ptr(y) + x * 3;
            p2[0] = dsp::clamp8(p2[0] + hl);
            p2[1] = dsp::clamp8(p2[1] + hl);
            p2[2] = dsp::clamp8(p2[2] + hl);
        }
    }
    for (int y = 0; y < H; ++y) {
        const uchar* sr = s.ptr(y);
        uchar* dr = d.ptr(y);
        for (int x = cx; x < W; ++x) {
            int sx = W - 1 - (x - cx);
            if (sx < 0) continue;
            float sh = .52f - float(x - cx) / float(W - cx) * .28f;
            dr[x * 3 + 0] = dsp::clamp8(sr[sx * 3 + 0] * sh);
            dr[x * 3 + 1] = dsp::clamp8(sr[sx * 3 + 1] * sh);
            dr[x * 3 + 2] = dsp::clamp8(sr[sx * 3 + 2] * sh);
        }
    }
}

static void fx_ripple(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3);
    float amp = p.amount * 20.f, freq = p.depth * .09f, spd2 = p.phase * kPI * 4.f;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        for (int x = 0; x < W; ++x) {
            float dx = x - W * .5f, dy = y - H * .5f, dist = std::sqrt(dx * dx + dy * dy);
            float off = std::sin(dist * freq + spd2) * amp;
            auto px = dsp::bsample(s, x + off * (dx / (dist + 1.f)), y + off * (dy / (dist + 1.f)));
            dr[x * 3 + 0] = px[0];
            dr[x * 3 + 1] = px[1];
            dr[x * 3 + 2] = px[2];
        }
    }
}

static void fx_sphere(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3);
    float R = std::min(W, H) * .44f * (.3f + p.amount * .7f), cx = W * .5f, cy = H * .5f, R2 = R * R;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        for (int x = 0; x < W; ++x) {
            float dx = x - cx, dy = y - cy, r2 = dx * dx + dy * dy;
            if (r2 < R2) {
                float r = std::sqrt(r2), th = std::atan2(r / R, std::sqrt(1.f - r2 / R2));
                float ss = R * std::sin(th);
                auto px = dsp::bsample(s, cx + dx * ss / (r + .001f), cy + dy * ss / (r + .001f));
                float sh = std::cos(th);
                dr[x * 3 + 0] = dsp::clamp8(px[0] * sh);
                dr[x * 3 + 1] = dsp::clamp8(px[1] * sh);
                dr[x * 3 + 2] = dsp::clamp8(px[2] * sh);
            } else {
                dr[x * 3 + 0] = 18;
                dr[x * 3 + 1] = 18;
                dr[x * 3 + 2] = 18;
            }
        }
    }
}

static void fx_squeeze(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3);
    float sq = 1.f - p.amount * .8f, bg = 1.f + p.depth * 2.f;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        for (int x = 0; x < W; ++x) {
            float nx = (x - W * .5f) / (W * .5f), ny = (y - H * .5f) / (H * .5f);
            float r2 = nx * nx + ny * ny, warp = 1.f + (bg - 1.f) * std::exp(-r2 * 4.f);
            auto px = dsp::bsample(s, (nx * sq * warp + 1.f) * W * .5f, (ny * sq * warp + 1.f) * H * .5f);
            dr[x * 3 + 0] = px[0];
            dr[x * 3 + 1] = px[1];
            dr[x * 3 + 2] = px[2];
        }
    }
}

static void fx_mosaic(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int bs = std::max(2, int(p.amount * 60.f) + 2);
    cv::Mat sm;
    cv::resize(s, sm, {s.cols / bs, s.rows / bs}, 0, 0, cv::INTER_NEAREST);
    cv::resize(sm, d, s.size(), 0, 0, cv::INTER_NEAREST);
}

static void fx_trails(const cv::Mat& s, cv::Mat& d, const Params& p) {
    float bl = 1.f - p.amount * .92f;
    if (s_trail.empty() || s_trail.size() != s.size()) s.copyTo(s_trail);
    cv::addWeighted(s, bl, s_trail, 1.f - bl, 0., d);
    d.copyTo(s_trail);
}

static void fx_mirror(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    int spx = std::clamp(int(W * (.5f + p.amount * .5f)), 1, W - 1);
    s.copyTo(d);
    for (int y = 0; y < H; ++y)
        for (int x = spx; x < W; ++x) {
            int from = spx - (x - spx);
            if (from < 0) from = 0;
            const uchar* sp = s.ptr(y) + from * 3;
            uchar* dp = d.ptr(y) + x * 3;
            dp[0] = sp[0];
            dp[1] = sp[1];
            dp[2] = sp[2];
        }
}

static void fx_cubespin(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    float ang = p.phase * kPI * 2.f, cA = std::cos(ang), scX = std::abs(cA);
    if (scX < .01f) scX = .01f;
    bool fl = cA < 0.f;
    int hW = W / 2;
    float shift = std::sin(ang) * hW * .5f * p.amount;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        for (int x = 0; x < W; ++x) {
            int sx = int((x - hW) / scX + shift + hW);
            if (fl) sx = W - 1 - sx;
            if (sx >= 0 && sx < W) {
                auto px = s.at<cv::Vec3b>(y, sx);
                float sh = .5f + .5f * scX;
                dr[x * 3 + 0] = dsp::clamp8(px[0] * sh);
                dr[x * 3 + 1] = dsp::clamp8(px[1] * sh);
                dr[x * 3 + 2] = dsp::clamp8(px[2] * sh);
            }
        }
    }
}

static void fx_kaleidoscope(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3);
    int seg = 6 + int(p.depth * 14);
    float segA = 2.f * kPI / seg, cx = W * .5f, cy = H * .5f;
    for (int y = 0; y < H; ++y) {
        uchar* dr = d.ptr(y);
        for (int x = 0; x < W; ++x) {
            float dx = x - cx, dy = y - cy, r = std::sqrt(dx * dx + dy * dy);
            float th = std::atan2(dy, dx), s2 = std::fmod(th, segA);
            if (s2 < 0) s2 += segA;
            if (s2 > segA * .5f) s2 = segA - s2;
            auto px = dsp::bsample(s, cx + r * std::cos(s2 + p.phase * kPI), cy + r * std::sin(s2 + p.phase * kPI));
            dr[x * 3 + 0] = px[0];
            dr[x * 3 + 1] = px[1];
            dr[x * 3 + 2] = px[2];
        }
    }
}

static void fx_shatter(const cv::Mat& s, cv::Mat& d, const Params& p) {
    int W = s.cols, H = s.rows;
    d = cv::Mat(H, W, CV_8UC3, cv::Scalar(0, 0, 0));
    int gs = std::max(8, 64 - int(p.amount * 56));
    float sa = p.depth * 30.f;
    std::mt19937 rg(42);
    std::uniform_real_distribution<float> uu(-sa, sa);
    for (int gy = 0; gy < H; gy += gs)
        for (int gx = 0; gx < W; gx += gs) {
            float ox = uu(rg), oy = uu(rg);
            int ew = std::min(gs, W - gx), eh = std::min(gs, H - gy);
            int dx = std::clamp(int(gx + ox), 0, W - ew), dy = std::clamp(int(gy + oy), 0, H - eh);
            cv::Mat(s, {gx, gy, ew, eh}).copyTo(cv::Mat(d, {dx, dy, ew, eh}));
        }
}

void apply(const cv::Mat& s, cv::Mat& d, const Params& p) {
    switch (p.type) {
        case Type::Tumble:       fx_tumble(s, d, p); break;
        case Type::PageTurn:     fx_pageturn(s, d, p); break;
        case Type::Ripple:       fx_ripple(s, d, p); break;
        case Type::Sphere:       fx_sphere(s, d, p); break;
        case Type::Squeeze:      fx_squeeze(s, d, p); break;
        case Type::Mosaic:       fx_mosaic(s, d, p); break;
        case Type::Trails:       fx_trails(s, d, p); break;
        case Type::Mirror:       fx_mirror(s, d, p); break;
        case Type::CubeSpin:     fx_cubespin(s, d, p); break;
        case Type::Kaleidoscope: fx_kaleidoscope(s, d, p); break;
        case Type::Shatter:      fx_shatter(s, d, p); break;
        default:                 fx_none(s, d, p); break;
    }
    if (p.mix < .999f && p.type != Type::None)
        cv::addWeighted(d, p.mix, s, 1.f - p.mix, 0.f, d);
}

float calcSignalDegradation(const Params& p) {
    float baseDegradation = 0.0f;

    switch (p.type) {
        case Type::Tumble:
            baseDegradation = 0.4f * p.depth;
            break;
        case Type::PageTurn:
            baseDegradation = 0.5f * p.depth;
            break;
        case Type::Ripple:
            baseDegradation = 0.25f * p.amount;
            break;
        case Type::Trails:
            baseDegradation = 0.2f * p.amount;
            break;
        case Type::Shatter:
            baseDegradation = 0.6f * p.amount;
            break;
        case Type::Sphere:
            baseDegradation = 0.02f * p.amount;
            break;
        case Type::Mosaic:
            baseDegradation = 0.05f * p.amount;
            break;
        case Type::Mirror:
            baseDegradation = 0.01f * p.amount;
            break;
        case Type::CubeSpin:
            baseDegradation = 0.03f * p.amount;
            break;
        case Type::Kaleidoscope:
            baseDegradation = 0.03f * p.amount;
            break;
        case Type::Squeeze:
            baseDegradation = 0.0f;
            break;
        default:
            baseDegradation = 0.0f;
            break;
    }

    float mixEffect = 1.0f - p.mix * 0.3f;

    return std::clamp(baseDegradation * mixEffect, 0.0f, 0.85f);
}

} // namespace Effects
