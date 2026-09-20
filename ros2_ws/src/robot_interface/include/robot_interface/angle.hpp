#pragma once

#include <cmath>

namespace robot_interface
{

inline double wrap_to_pi(double angle) {
    constexpr double pi = 3.14159265358979323846;
    double wrapped = std::fmod(angle + pi, 2.0 * pi);
    if (wrapped < 0.0) {
        wrapped += 2.0 * pi;
    }
    return wrapped - pi;
}

}
