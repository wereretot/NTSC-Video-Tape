#pragma once
#include "constants.h"
#include <atomic>
#include <mutex>

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
};

class NTSCSimulator {
public:
    float v_roll_accum_ = 0.0f;
    float noise_bar_phase_ = 0.0f;
    float tapeTime_ = 0.0f;
    float wallTime_ = 0.0f;
    float h_roll_phase_ = 0.0f;
    float tracking_error_lpf_ = 0.0f;
    float mechanical_wow_ = 0.0f;
    float mechanical_flutter_ = 0.0f;
    float afc_error_ = 0.0f;

    float wow_phase_[3] = {0.0f, 0.0f, 0.0f};
    float flutter_phase_[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    float snow_phase_[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

    void initBuffers(int w, int h);

    void process(const cv::Mat& in, cv::Mat& out, int frameNum, const EngineParams& ep,
                 const VideoParams& vp, float tapeSpd, float instantSpd, float wallDt,
                 int current_recording_id);

    void setRecordingId(int id) { recording_id_ = id; }

private:
    int W_ = 0, H_BLANK = 0, W_TOTAL = 0;
    float kSC_PX = 0.f;

    float generateSnow(float time, int x, int y, std::mt19937& rng);
};

extern std::atomic<double> g_wallTimeSec;
