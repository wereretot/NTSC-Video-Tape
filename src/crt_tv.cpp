#include "crt_tv.h"

#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace {
constexpr float kCrtPi = 3.14159265f;

inline uint8_t clamp8(float v) {
    return uint8_t(std::max(0.0f, std::min(255.0f, v)));
}

inline bool inBounds(const cv::Mat& src, float x, float y) {
    return x >= 0.0f && y >= 0.0f && x < float(src.cols - 1) && y < float(src.rows - 1);
}

inline cv::Vec3b sampleBlack(const cv::Mat& src, float x, float y) {
    if (!inBounds(src, x, y)) return cv::Vec3b{0, 0, 0};

    x = std::max(0.f, std::min(float(src.cols - 1.001f), x));
    y = std::max(0.f, std::min(float(src.rows - 1.001f), y));
    int ix = int(x), iy = int(y);
    float fx = x - ix, fy = y - iy;
    int ix1 = std::min(ix + 1, src.cols - 1);
    int iy1 = std::min(iy + 1, src.rows - 1);
    const auto& p00 = src.at<cv::Vec3b>(iy, ix);
    const auto& p10 = src.at<cv::Vec3b>(iy, ix1);
    const auto& p01 = src.at<cv::Vec3b>(iy1, ix);
    const auto& p11 = src.at<cv::Vec3b>(iy1, ix1);
    return cv::Vec3b{
        clamp8((p00[0] * (1 - fx) + p10[0] * fx) * (1 - fy) + (p01[0] * (1 - fx) + p11[0] * fx) * fy),
        clamp8((p00[1] * (1 - fx) + p10[1] * fx) * (1 - fy) + (p01[1] * (1 - fx) + p11[1] * fx) * fy),
        clamp8((p00[2] * (1 - fx) + p10[2] * fx) * (1 - fy) + (p01[2] * (1 - fx) + p11[2] * fx) * fy)
    };
}

inline float luma(const cv::Vec3b& c) {
    return (0.114f * c[0] + 0.587f * c[1] + 0.299f * c[2]) / 255.0f;
}
}

const char* const kTVPresetNames[] = {
    "13\" RF PORTABLE",
    "19\" LIVING ROOM",
    "25\" CONSOLE",
    "27\" TRINITRON",
    "20\" PRO MONITOR",
    "REAR PROJECTION"
};

const char* const kTVOutputNames[] = {"RF COAX", "COMPOSITE RCA", "S-VIDEO"};
const char* const kCombFilterNames[] = {"NONE", "1-LINE", "2-LINE"};
const char* const kMaskTypeNames[] = {"NONE", "SHADOW MASK", "SLOT MASK", "APERTURE GRILLE"};

void applyTVPreset(TVParams& tv, TVPreset preset) {
    tv = {};
    tv.preset = preset;

    switch (preset) {
        case TVPreset::RFPortable13:
            tv.output_type = VCROutputType::RF_Coax;
            tv.input_noise = 0.050f;
            tv.cable_quality = 0.50f;
            tv.hum_level = 0.22f;
            tv.rf_interference = 0.26f;
            tv.tv_color = 0.84f;
            tv.tv_brightness = 0.53f;
            tv.tv_contrast = 1.16f;
            tv.tv_sharpness = 0.24f;
            tv.comb_quality = CombFilterType::None;
            tv.scanline_strength = 0.56f;
            tv.phosphor_persistence = 0.20f;
            tv.tv_bloom = 0.48f;
            tv.tv_pincushion = 0.24f;
            tv.convergence_error = 0.55f;
            tv.halation = 0.20f;
            tv.vignette = 0.72f;
            tv.mask_type = MaskType::SlotMask;
            tv.mask_pitch = 0.46f;
            tv.room_brightness = 0.20f;
            break;
        case TVPreset::Console25:
            tv.output_type = VCROutputType::Composite_RCA;
            tv.input_noise = 0.024f;
            tv.cable_quality = 0.76f;
            tv.hum_level = 0.06f;
            tv.rf_interference = 0.05f;
            tv.tv_hue = -2.0f;
            tv.tv_color = 1.10f;
            tv.tv_brightness = 0.49f;
            tv.tv_contrast = 1.05f;
            tv.tv_sharpness = 0.38f;
            tv.comb_quality = CombFilterType::OneLine;
            tv.scanline_strength = 0.42f;
            tv.phosphor_persistence = 0.18f;
            tv.tv_bloom = 0.36f;
            tv.tv_pincushion = 0.18f;
            tv.convergence_error = 0.32f;
            tv.halation = 0.18f;
            tv.vignette = 0.56f;
            tv.mask_type = MaskType::ShadowMask;
            tv.mask_pitch = 0.34f;
            tv.room_brightness = 0.30f;
            break;
        case TVPreset::Trinitron27:
            tv.output_type = VCROutputType::Composite_RCA;
            tv.input_noise = 0.012f;
            tv.cable_quality = 0.88f;
            tv.hum_level = 0.015f;
            tv.rf_interference = 0.025f;
            tv.tv_hue = 1.5f;
            tv.tv_color = 1.08f;
            tv.tv_brightness = 0.50f;
            tv.tv_contrast = 1.13f;
            tv.tv_sharpness = 0.66f;
            tv.comb_quality = CombFilterType::TwoLine;
            tv.scanline_strength = 0.36f;
            tv.phosphor_persistence = 0.10f;
            tv.tv_bloom = 0.22f;
            tv.tv_pincushion = 0.08f;
            tv.convergence_error = 0.13f;
            tv.halation = 0.10f;
            tv.vignette = 0.36f;
            tv.mask_type = MaskType::ApertureGrille;
            tv.mask_pitch = 0.24f;
            tv.room_brightness = 0.24f;
            tv.aperture_wires = true;
            break;
        case TVPreset::ProMonitor20:
            tv.output_type = VCROutputType::SVideo;
            tv.input_noise = 0.006f;
            tv.cable_quality = 0.96f;
            tv.hum_level = 0.0f;
            tv.rf_interference = 0.0f;
            tv.tv_hue = 0.0f;
            tv.tv_color = 0.98f;
            tv.tv_brightness = 0.49f;
            tv.tv_contrast = 1.08f;
            tv.tv_sharpness = 0.78f;
            tv.comb_quality = CombFilterType::TwoLine;
            tv.scanline_strength = 0.48f;
            tv.phosphor_persistence = 0.07f;
            tv.tv_bloom = 0.12f;
            tv.tv_pincushion = 0.03f;
            tv.convergence_error = 0.06f;
            tv.halation = 0.06f;
            tv.vignette = 0.22f;
            tv.mask_type = MaskType::ApertureGrille;
            tv.mask_pitch = 0.20f;
            tv.room_brightness = 0.18f;
            tv.aperture_wires = true;
            break;
        case TVPreset::RearProjection:
            tv.output_type = VCROutputType::Composite_RCA;
            tv.input_noise = 0.020f;
            tv.cable_quality = 0.72f;
            tv.hum_level = 0.04f;
            tv.rf_interference = 0.04f;
            tv.tv_hue = -4.0f;
            tv.tv_color = 0.88f;
            tv.tv_brightness = 0.56f;
            tv.tv_contrast = 0.92f;
            tv.tv_sharpness = 0.18f;
            tv.comb_quality = CombFilterType::OneLine;
            tv.scanline_strength = 0.22f;
            tv.phosphor_persistence = 0.28f;
            tv.tv_bloom = 0.64f;
            tv.tv_pincushion = 0.16f;
            tv.convergence_error = 0.70f;
            tv.halation = 0.32f;
            tv.vignette = 0.82f;
            tv.mask_type = MaskType::None;
            tv.mask_pitch = 0.55f;
            tv.room_brightness = 0.42f;
            break;
        case TVPreset::LivingRoom19:
        case TVPreset::COUNT:
        default:
            tv.output_type = VCROutputType::Composite_RCA;
            tv.input_noise = 0.018f;
            tv.cable_quality = 0.82f;
            tv.hum_level = 0.03f;
            tv.rf_interference = 0.035f;
            tv.tv_hue = -1.0f;
            tv.tv_color = 1.02f;
            tv.tv_brightness = 0.51f;
            tv.tv_contrast = 1.02f;
            tv.tv_sharpness = 0.46f;
            tv.comb_quality = CombFilterType::OneLine;
            tv.scanline_strength = 0.38f;
            tv.phosphor_persistence = 0.14f;
            tv.tv_bloom = 0.30f;
            tv.tv_pincushion = 0.12f;
            tv.convergence_error = 0.24f;
            tv.halation = 0.14f;
            tv.vignette = 0.48f;
            tv.mask_type = MaskType::ShadowMask;
            tv.mask_pitch = 0.30f;
            tv.room_brightness = 0.28f;
            break;
    }
}

void applyCRTOutput(const cv::Mat& in,
                    cv::Mat& out,
                    const TVParams& tv,
                    int frameNum,
                    float wallTime,
                    const std::vector<float>& noiseBuf,
                    int noiseStride,
                    cv::Mat& phosphorBuffer) {
    if (in.empty()) {
        out = in;
        return;
    }

    const int W = in.cols;
    const int H = in.rows;
    if (phosphorBuffer.empty() || phosphorBuffer.cols != W || phosphorBuffer.rows != H) {
        phosphorBuffer.create(H, W, CV_32FC3);
        phosphorBuffer.setTo(cv::Scalar(0, 0, 0));
    }

    cv::Mat glow;
    float glowSigma = 0.75f + tv.tv_bloom * 2.5f + tv.halation * 1.5f;
    cv::GaussianBlur(in, glow, cv::Size(0, 0), glowSigma, glowSigma);

    out.create(H, W, CV_8UC3);
    const float outputNoise =
        tv.output_type == VCROutputType::RF_Coax ? 0.032f :
        tv.output_type == VCROutputType::Composite_RCA ? 0.010f : 0.003f;
    const float outputColor =
        tv.output_type == VCROutputType::RF_Coax ? 0.82f :
        tv.output_type == VCROutputType::Composite_RCA ? 1.00f : 1.07f;
    const float combLeakBase =
        tv.comb_quality == CombFilterType::None ? 0.085f :
        tv.comb_quality == CombFilterType::OneLine ? 0.040f : 0.015f;
    const float chromaLeak =
        tv.output_type == VCROutputType::SVideo ? combLeakBase * 0.25f :
        tv.output_type == VCROutputType::RF_Coax ? combLeakBase + 0.050f : combLeakBase;
    const float noiseAmp = tv.input_noise + (1.0f - tv.cable_quality) * 0.035f + outputNoise;
    const float hue = tv.tv_hue * kCrtPi / 180.0f;
    const float hueC = std::cos(hue);
    const float hueS = std::sin(hue);
    const float colorGain = tv.tv_color * outputColor;
    const float sharp = tv.tv_sharpness - 0.5f;
    const float convPx = tv.convergence_error * 1.65f;
    const float persist = std::clamp(tv.phosphor_persistence, 0.0f, 0.85f);
    const float maskStrength = std::clamp(0.10f + tv.mask_pitch * 0.40f, 0.08f, 0.34f);

#ifdef ADO_OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (int y = 0; y < H; ++y) {
        uchar* dr = out.ptr(y);
        cv::Vec3f* persistRow = phosphorBuffer.ptr<cv::Vec3f>(y);
        const float ny = (float(y) + 0.5f) / float(H) * 2.0f - 1.0f;
        for (int x = 0; x < W; ++x) {
            const float nx = (float(x) + 0.5f) / float(W) * 2.0f - 1.0f;
            float r2 = nx * nx + ny * ny;
            float warp = 1.0f + tv.tv_pincushion * 0.105f * r2;
            float sx = (nx * warp * 0.5f + 0.5f) * float(W - 1);
            float sy = (ny * warp * 0.5f + 0.5f) * float(H - 1);
            bool activeVideo = inBounds(in, sx, sy);

            float conv = convPx * (0.35f + 0.65f * std::sqrt(std::min(r2, 2.0f) * 0.5f));
            cv::Vec3b cg = sampleBlack(in, sx, sy);
            cv::Vec3b cr = sampleBlack(in, sx + conv, sy + conv * ny * 0.18f);
            cv::Vec3b cb = sampleBlack(in, sx - conv, sy - conv * ny * 0.18f);

            float r = cr[2] / 255.0f;
            float g = cg[1] / 255.0f;
            float b = cb[0] / 255.0f;

            float Y = 0.299f * r + 0.587f * g + 0.114f * b;
            float I = 0.596f * r - 0.274f * g - 0.321f * b;
            float Q = 0.211f * r - 0.523f * g + 0.311f * b;

            if (std::abs(sharp) > 0.001f) {
                float blurY = (luma(sampleBlack(in, sx - 1.35f, sy)) + luma(sampleBlack(in, sx + 1.35f, sy)) + Y) / 3.0f;
                if (sharp > 0.0f) {
                    Y += (Y - blurY) * sharp * 1.35f;
                } else {
                    float soft = -sharp * 0.75f;
                    Y = Y * (1.0f - soft) + blurY * soft;
                    I *= 1.0f - soft * 0.22f;
                    Q *= 1.0f - soft * 0.22f;
                }
            }

            float crawl = std::sin((float(x) * 0.92f + float(y) * 1.71f + float(frameNum) * 0.73f) * kCrtPi);
            Y += (I + Q) * chromaLeak * crawl;
            I += Y * chromaLeak * 0.18f * crawl;
            Q -= Y * chromaLeak * 0.14f * crawl;

            float rotI = I * hueC - Q * hueS;
            float rotQ = I * hueS + Q * hueC;
            I = rotI * colorGain;
            Q = rotQ * colorGain;

            if (!noiseBuf.empty() && noiseStride > 0) {
                float n0 = noiseBuf[y * noiseStride + std::min(x, noiseStride - 1)];
                float n1 = noiseBuf[(H - 1 - y) * noiseStride + std::min(W - 1 - x, noiseStride - 1)];
                Y += n0 * noiseAmp;
                I += n1 * noiseAmp * 0.28f;
                Q -= n0 * noiseAmp * 0.24f;
            }

            if (tv.hum_level > 0.001f) {
                Y += std::sin(float(y) / float(H) * kCrtPi * 4.0f + wallTime * kCrtPi * 120.0f) * tv.hum_level * 0.035f;
            }
            if (tv.rf_interference > 0.001f) {
                float rf = std::sin(float(x) * 0.055f + float(y) * 0.17f + float(frameNum) * 0.61f)
                         * std::sin(float(y) * 0.019f - float(frameNum) * 0.13f);
                Y += rf * tv.rf_interference * 0.050f;
                I += rf * tv.rf_interference * 0.018f;
                Q -= rf * tv.rf_interference * 0.014f;
            }

            r = Y + 0.956f * I + 0.621f * Q;
            g = Y - 0.272f * I - 0.647f * Q;
            b = Y - 1.106f * I + 1.703f * Q;

            r = (r - 0.5f) * tv.tv_contrast + 0.5f + (tv.tv_brightness - 0.5f) * 0.55f;
            g = (g - 0.5f) * tv.tv_contrast + 0.5f + (tv.tv_brightness - 0.5f) * 0.55f;
            b = (b - 0.5f) * tv.tv_contrast + 0.5f + (tv.tv_brightness - 0.5f) * 0.55f;

            cv::Vec3b gl = sampleBlack(glow, sx, sy);
            float glowLum = std::max({gl[0], gl[1], gl[2]}) / 255.0f;
            float glowGate = std::pow(std::clamp((glowLum - 0.45f) / 0.55f, 0.0f, 1.0f), 1.35f);
            float glowAmt = glowGate * (tv.halation * 0.22f + tv.tv_bloom * 0.18f);
            r += (gl[2] / 255.0f) * glowAmt;
            g += (gl[1] / 255.0f) * glowAmt;
            b += (gl[0] / 255.0f) * glowAmt;

            float scan = 1.0f;
            if ((y & 1) == 0) {
                scan -= tv.scanline_strength * (0.34f + (1.0f - std::clamp(Y, 0.0f, 1.0f)) * 0.22f);
            } else {
                scan -= tv.scanline_strength * 0.08f;
            }
            r *= scan;
            g *= scan;
            b *= scan;

            if (tv.mask_type != MaskType::None) {
                if (tv.mask_type == MaskType::ApertureGrille) {
                    int cell = x % 3;
                    float dim = 1.0f - maskStrength;
                    r *= cell == 0 ? 1.0f + maskStrength * 0.35f : dim;
                    g *= cell == 1 ? 1.0f + maskStrength * 0.35f : dim;
                    b *= cell == 2 ? 1.0f + maskStrength * 0.35f : dim;
                } else if (tv.mask_type == MaskType::SlotMask) {
                    int cell = (x / 2 + (y / 2)) % 3;
                    float gate = ((y / 2) & 1) ? 0.88f : 1.04f;
                    r *= (cell == 0 ? 1.0f : 1.0f - maskStrength * 0.75f) * gate;
                    g *= (cell == 1 ? 1.0f : 1.0f - maskStrength * 0.75f) * gate;
                    b *= (cell == 2 ? 1.0f : 1.0f - maskStrength * 0.75f) * gate;
                } else {
                    int cell = (x + (y & 1)) % 3;
                    r *= cell == 0 ? 1.0f : 1.0f - maskStrength * 0.70f;
                    g *= cell == 1 ? 1.0f : 1.0f - maskStrength * 0.70f;
                    b *= cell == 2 ? 1.0f : 1.0f - maskStrength * 0.70f;
                }
            }

            if (tv.aperture_wires) {
                float y0 = float(H) * 0.333f;
                float y1 = float(H) * 0.666f;
                if (std::abs(float(y) - y0) < 0.65f || std::abs(float(y) - y1) < 0.65f) {
                    r *= 0.74f;
                    g *= 0.74f;
                    b *= 0.74f;
                }
            }

            float vig = 1.0f - tv.vignette * 0.30f * std::pow(std::clamp(r2 * 0.5f, 0.0f, 1.0f), 0.72f);
            float ambient = activeVideo ? tv.room_brightness * 0.018f : 0.0f;
            r = r * vig + ambient;
            g = g * vig + ambient;
            b = b * vig + ambient;

            cv::Vec3f cur(b, g, r);
            float localPersist = activeVideo ? persist : persist * 0.20f;
            cv::Vec3f held = cur * (1.0f - localPersist) + persistRow[x] * localPersist;
            persistRow[x] = held;

            dr[x * 3 + 0] = (uchar)std::clamp(held[0] * 255.0f, 0.0f, 255.0f);
            dr[x * 3 + 1] = (uchar)std::clamp(held[1] * 255.0f, 0.0f, 255.0f);
            dr[x * 3 + 2] = (uchar)std::clamp(held[2] * 255.0f, 0.0f, 255.0f);
        }
    }
}
