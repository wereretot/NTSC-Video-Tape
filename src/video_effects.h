#pragma once
#include <string>
#include "constants.h"

namespace Effects {

enum class Type : int {
    None = 0, Tumble, PageTurn, Ripple, Sphere,
    Squeeze, Mosaic, Trails, Mirror, CubeSpin, Kaleidoscope, Shatter, COUNT
};

static const char* kNames[] = {
    "BYPASS", "TUMBLE", "PAGE TURN", "RIPPLE", "SPHERE",
    "SQUEEZE", "MOSAIC", "TRAILS", "MIRROR", "CUBE SPIN", "KALEID.", "SHATTER"
};

struct Params {
    Type type = Type::None;
    float amount = .5f, speed = .3f, depth = .6f, phase = 0.f, mix = 1.f;
};

void apply(const cv::Mat& s, cv::Mat& d, const Params& p);

float calcSignalDegradation(const Params& p);

void resetTrail();

} // namespace Effects
