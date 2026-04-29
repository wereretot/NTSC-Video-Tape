#pragma once

#include <opencv2/core.hpp>
#include <vector>

enum class VCROutputType { RF_Coax = 0, Composite_RCA, SVideo };
enum class TVPreset { RFPortable13 = 0, LivingRoom19, Console25, Trinitron27, ProMonitor20, RearProjection, COUNT };
enum class CombFilterType { None = 0, OneLine, TwoLine };
enum class MaskType { None = 0, ShadowMask, SlotMask, ApertureGrille };

extern const char* const kTVPresetNames[];
extern const char* const kTVOutputNames[];
extern const char* const kCombFilterNames[];
extern const char* const kMaskTypeNames[];

struct TVParams {
    VCROutputType  output_type = VCROutputType::Composite_RCA;
    TVPreset       preset = TVPreset::LivingRoom19;

    float          input_noise = 0.015f;
    float          cable_quality = 0.85f;
    float          hum_level = 0.0f;
    float          rf_interference = 0.0f;

    float          tv_hue = 0.0f;
    float          tv_color = 1.0f;
    float          tv_brightness = 0.5f;
    float          tv_contrast = 1.0f;
    float          tv_sharpness = 0.5f;
    CombFilterType comb_quality = CombFilterType::TwoLine;

    float          scanline_strength = 0.35f;
    float          phosphor_persistence = 0.12f;
    float          tv_bloom = 0.25f;
    float          tv_pincushion = 0.08f;
    float          convergence_error = 0.18f;
    float          halation = 0.12f;
    float          vignette = 0.45f;
    MaskType       mask_type = MaskType::ShadowMask;
    float          mask_pitch = 0.30f;
    float          room_brightness = 0.25f;
    bool           aperture_wires = false;
};

void applyTVPreset(TVParams& tv, TVPreset preset);

void applyCRTOutput(const cv::Mat& in,
                    cv::Mat& out,
                    const TVParams& tv,
                    int frameNum,
                    float wallTime,
                    const std::vector<float>& noiseBuf,
                    int noiseStride,
                    cv::Mat& phosphorBuffer);
