#ifndef COMMON_H
#define COMMON_H

#include <plog/Log.h>
#include "plog/Initializers/RollingFileInitializer.h"
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include <tuple>

#pragma once

constexpr float wrapRad(float& angle){
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

constexpr float deg2rad(float degrees){
    return degrees * (M_PI / 180.0f);
}

constexpr float rad2deg(float radians){
    return radians * (180.0f / M_PI);
}

constexpr float dT = 0.01f;

#endif