#pragma once

#include <cmath>
#include <cstdint>
#include <opencv2/core.hpp>

static constexpr int   FW            = 640;
static constexpr int   FH            = 480;
static constexpr float ASPECT        = float(FW)/float(FH);
static constexpr int   Q_DEPTH       = 4;
static constexpr int   AUDIO_SR      = 44100;
static const     float kPI           = 3.14159265f;
static constexpr float kSC_PX        = 2.f * 3.14159265f * 227.5f / float(FW);
static constexpr float kNTSC_FPS     = 29.97f;
static constexpr float kNTSC_FIELD_HZ = kNTSC_FPS * 2.f;
static constexpr float kNTSC_FRAME_S = 1.f / kNTSC_FPS;
static constexpr float kNTSC_HSYNC   = 15734.26f;
static constexpr int   kNTSC_TOTAL_LINES = 525;
static constexpr int   kNTSC_ACTIVE_LINES = 480;
static constexpr float kHS_VBI_LINES = 15.f;
static constexpr float kSC_FREQ      = 227.5f;

static constexpr int   kFIELDS_PER_FRAME  = 2;
static constexpr int   kLINES_PER_FIELD   = kNTSC_ACTIVE_LINES / kFIELDS_PER_FRAME;
static constexpr float kVHS_DRUM_RPM      = 1800.f;
static constexpr float kVHS_HEAD_ANGLE_DEG = 5.967f;
static constexpr float kVHS_FM_CARRIER_SP = 4.4f;
static constexpr float kVHS_FM_CARRIER_LP = 3.4f;
static constexpr float kVHS_FM_DEV_SP     = 1.0f;
static constexpr float kVHS_CHROMA_SHIFT  = 629.375f;
static constexpr float kVHS_HSW_LINE      = 7.f;
static constexpr float kVHS_TRACK_PITCH_SP = 0.049f;
static constexpr float kVHS_TRACK_PITCH_LP = 0.0245f;

namespace dsp {

inline float clamp(float v, float lo = 0.f, float hi = 1.f) {
    return v < lo ? lo : v > hi ? hi : v;
}

inline uint8_t clamp8(float v) {
    return uint8_t(std::max(0.f, std::min(255.f, v)));
}

inline float lerp(float a, float b, float t) {
    return a + (b - a) * t;
}

inline cv::Vec3b bsample(const cv::Mat& s, float x, float y) {
    x = std::max(0.f, std::min(float(s.cols - 1.001f), x));
    y = std::max(0.f, std::min(float(s.rows - 1.001f), y));
    int ix = int(x), iy = int(y);
    float fx = x - ix, fy = y - iy;
    int ix1 = std::min(ix + 1, s.cols - 1), iy1 = std::min(iy + 1, s.rows - 1);
    const auto& p00 = s.at<cv::Vec3b>(iy, ix);
    const auto& p10 = s.at<cv::Vec3b>(iy, ix1);
    const auto& p01 = s.at<cv::Vec3b>(iy1, ix);
    const auto& p11 = s.at<cv::Vec3b>(iy1, ix1);
    return cv::Vec3b{
        clamp8((p00[0] * (1 - fx) + p10[0] * fx) * (1 - fy) + (p01[0] * (1 - fx) + p11[0] * fx) * fy),
        clamp8((p00[1] * (1 - fx) + p10[1] * fx) * (1 - fy) + (p01[1] * (1 - fx) + p11[1] * fx) * fy),
        clamp8((p00[2] * (1 - fx) + p10[2] * fx) * (1 - fy) + (p01[2] * (1 - fx) + p11[2] * fx) * fy)
    };
}

} // namespace dsp
