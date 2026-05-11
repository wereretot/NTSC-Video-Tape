#pragma once

#include "constants.h"
#include "vcr_brand.h"
#include "fm_luma.h"
#include "color_under.h"
#include "vcr_audio.h"
#include "tape_degradation.h"
// Assuming dsp_types.hpp contains structural dependencies like EngineParams
#include "dsp_types.hpp"
#include <atomic>
#include <mutex>
#include <vector>
#include <random>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

struct VideoParams {
    float tracking_error = 0.0f;
    float tape_crease = 0.0f;
    float sync_hold_failure = 0.0f;
    float signal_strength = 1.0f;

    float dropout_rate = 0.0f;
    float motor_health = 0.0f;
    float oxide_shedding = 0.0f;
    float demagnetization = 0.0f;
    float sticky_shed = 0.0f;
    float tape_age = 0.0f;

    float base_rf_level = 1.0f;
    float chroma_level = 1.0f;
    float luma_noise = 0.0f;
    float chroma_noise = 0.0f;
    float dropout_intensity = 0.0f;

    float helical_sweep = 0.5f;
    float head_switch_jitter = 0.3f;
    float fm_carrier_noise = 0.15f;
    float chroma_crosstalk = 0.2f;
    float inter_field_phase_error = 0.1f;
    float head_pre_echo = 0.0f;
    float drum_eccentricity = 0.0f;

    // VCR alignment parameters (0 = perfectly aligned, 1 = severely misaligned)
    float head_azimuth_error = 0.0f;    // Video head azimuth angle misalignment
    float tracking_alignment = 0.0f;    // Control track head position error
    float drum_height_error = 0.0f;     // Drum height / tape path alignment
    float audio_head_alignment = 0.0f;  // Audio head position/azimuth error
};

// VCR Output Type and TV params
enum class VCROutputType { RF_Coax, Composite_RCA, SVideo };
enum class TVPreset { Cheap13, MidRange19, Better25, Trinitron27, Projection };
enum class CombFilterType { None, OneLine, TwoLine };
enum class MaskType { None, ShadowMask, ApertureGrille };

struct TVParams {
    VCROutputType   output_type     = VCROutputType::Composite_RCA;
    TVPreset        preset          = TVPreset::MidRange19;

    // Front-end
    float           input_noise     = 0.015f;
    float           cable_quality   = 0.85f;
    float           hum_level       = 0.0f;
    float           rf_interference = 0.0f;

    // Color decoder
    float           tv_hue          = 0.0f;    // degrees
    float           tv_color        = 1.0f;
    float           tv_brightness   = 0.5f;
    float           tv_contrast     = 1.0f;
    float           tv_sharpness    = 0.5f;
    CombFilterType  comb_quality    = CombFilterType::TwoLine;

    // CRT display
    float           scanline_strength = 0.4f;
    float           phosphor_persistence = 0.12f;
    float           tv_bloom        = 0.3f;
    float           tv_pincushion   = 0.1f;
    float           convergence_error = 0.3f;
    float           halation        = 0.15f;
    float           vignette        = 0.6f;
    MaskType        mask_type       = MaskType::ShadowMask;
    float           mask_pitch      = 0.28f;   // mm
    float           warmup_time     = 999.f;   // seconds since power-on
    float           room_brightness = 0.3f;
    bool            aperture_wires  = false;   // Trinitron damper wires
};

struct HeadState {
    float rf_level       = 1.0f;    // slight variation head to head
    float azimuth_error  = 0.0f;    // degrees — causes HF rolloff when misaligned
    float chroma_phase   = 0.0f;    // each head has independent chroma AFC error
};

class NTSCSimulator {
public:
    float v_roll_accum_ = 0.0f;
    float noise_bar_phase_ = 0.0f;
    float h_roll_phase_ = 0.0f;
    float tracking_error_lpf_ = 0.0f;
    float tracking_lock_ = 1.0f;
    float motor_top_bend_phase_ = 0.0f;
    float motor_top_bend_lpf_ = 0.0f;
    float crinkle_center_lpf_ = 0.0f;
    float crinkle_strength_lpf_ = 0.0f;
    float crinkle_phase_ = 0.0f;
    float mechanical_wow_ = 0.0f;
    float mechanical_flutter_ = 0.0f;
    float afc_error_ = 0.0f;
    
    // VCR alignment state (tracks misalignment effects)
    float head_azimuth_shift_ = 0.0f;     // Head azimuth angle offset (degrees)
    float tracking_phase_error_ = 0.0f;   // Control track phase error (lines)
    float drum_tilt_error_ = 0.0f;        // Drum tilt causing top/bottom skew diff
    float audio_head_shift_ = 0.0f;       // Audio head position error
    
    // Motor degradation state
    float motor_speed_var_ = 0.0f;        // Instantaneous speed deviation
    float motor_belt_slip_phase_ = 0.0f;  // Belt slip position in frame
    float capstan_wow_phase_ = 0.0f;      // Capstan motor wow cycle
    float drum_wobble_accum_ = 0.0f;      // Drum bearing wobble accumulation
    
    float tapeTime_ = 0.0f;
    float wallTime_ = 0.0f;

    float wow_phase_[3] = {0.0f, 0.0f, 0.0f};
    float snow_phase_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float flutter_phase_[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    int recording_id_ = 0;
    int last_recording_id_ = 0;
    float recording_blend_ = 0.0f;
    float recording_transition_time_ = 0.0f;

    int prev_field_ = -1;
    float field_chroma_phase_accum_ = 0.0f;
    float head_switch_offset_ = 0.0f;
    float drum_ecc_phase_ = 0.0f;
    float adjacent_track_phase_ = 0.0f;
    float prev_line_luma_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    
    TVParams tvParams_;

    // VCR configuration
    VCRBrand         brand_             = VCRBrand::JVC;
    VCRBrandProfile  brand_profile_;
    float            brand_transition_time_ = 0.0f;  // Smooth brand transition timer
    HeadConfig       head_config_       = {};
    TapeFormat       tape_format_       = TapeFormat::VHS;

    void initBuffers(int w, int h);

    void process(const cv::Mat& in, cv::Mat& out, int frameNum, const EngineParams& ep,
                 const VideoParams& vp, float tapeSpd, float instantSpd, float wallDt,
                 int current_recording_id);

    void setRecordingId(int id) { recording_id_ = id; }

    // VCR brand / head / tape format configuration
    void setBrand(VCRBrand brand) {
        VCRBrand prev = brand_;
        brand_ = brand;
        brand_profile_ = ::getBrandProfile(brand);
        brand_transition_time_ = (prev != brand) ? 0.5f : 0.0f;
    }
    void setHeadCount(HeadCount count) {
        head_config_ = ::getHeadConfig(count);
    }
    void setTapeFormat(TapeFormat fmt) { tape_format_ = fmt; }

    VCRBrand    getBrand()      const { return brand_; }
    HeadCount   getHeadCount()  const { return head_config_.count; }
    TapeFormat  getTapeFormat() const { return tape_format_; }

    const VCRBrandProfile& getBrandProfile() const { return brand_profile_; }
    const HeadConfig&      getHeadConfig()  const { return head_config_; }

private:
    int W_ = 0, H_ = 0, H_BLANK = 0, W_TOTAL = 0, W_TOTAL_ = 0;
    float kSC_PX = 0.f;

    std::vector<float> noise_buf_;
    std::vector<float> sc_cos_, sc_sin_;
    std::vector<float> comp_frame_;
    cv::Mat field_buf_[2];
    cv::Mat field_out_[2];
    cv::Mat phosphor_buffer_;
    HeadState heads_[2];

    // FM Luma Processing (Phase 2)
    FMLumaProcessor fm_luma_;
    std::vector<float> fm_encode_buf_;
    std::vector<float> fm_decode_buf_;

    // Color-Under Processing (Phase 3)
    ColorUnderProcessor color_under_;
    std::vector<float> cu_chroma_i_buf_;
    std::vector<float> cu_chroma_q_buf_;
    std::vector<float> cu_chroma_i_down_;
    std::vector<float> cu_chroma_q_down_;

    // Tape Degradation (Phase 5)
    TapeDegradationProcessor tape_degradation_;
    TapeState tape_state_;

    float generateSnow(float time, int x, int y, std::mt19937& rng); // Kept if needed by other components, though noise_buf_ is replacement
};

extern std::atomic<double> g_wallTimeSec;
