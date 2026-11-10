// thrust_duty_profile.hpp
// Provides a simple mapping from desired thrust (N) to PWM on-time (µs)
// Generated from t200.csv measurements (linear interpolation between measured points).

#pragma once

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <algorithm>

namespace thrust_profile {

// Convert desired thrust (in Newtons) to PWM on-time in microseconds.
// - Input: force_N (positive = forward, negative = reverse).
// - Output: PWM pulse width in microseconds.
//   Positive force: 1500–1900 µs  /  Negative force: 1500–1100 µs (symmetric around center)
inline uint16_t forceN_to_on_time(double force_N) {
    // Measured mapping (force in N -> pwm in µs). Force values are absolute magnitudes.
    static constexpr double forces[] = {
        0.392266,
        0.784532,
        2.157463,
        2.451662,
        3.530394,
        3.922660,
        4.609125,
        5.491724,
        5.883990,
        6.276256,
        8.924051,
        9.316317,
        9.708583,
        10.100849,
        11.179581,
        11.571847,
        12.160246,
        12.552512,
        13.042844,
        13.631243,
        14.513842,
        15.102241,
        15.592574,
        16.180972,
        16.573238,
        17.848103,
        18.436502,
        18.926834,
        19.515233,
        20.103632,
        21.378497,
        21.770763,
        23.830160,
        24.712758,
        25.987622,
        26.772154,
        27.066354,
        27.850886,
        28.341219,
        29.223817,
        29.910282,
        31.087080,
        31.577413,
        32.361945,
        33.048410,
        33.538743,
        34.421341,
        35.402006,
        36.088472,
        36.676871,
        38.147869,
        39.814999,
        40.697598,
        41.678262,
        42.168595,
        42.953127,
        44.227991,
        44.424124,
        45.600923,
        46.189321,
        46.973853,
        47.464186,
        48.346784,
        49.131316,
        49.817782,
        50.406181,
        50.798447,
        51.190713,
        51.484913,
    };

    static constexpr uint16_t pwms[] = {
        1500,1540,1560,1564,1576,1580,1588,1596,1600,1604,1628,1632,1636,1640,1648,1652,1656,1660,1664,1668,1676,1680,1684,1688,1692,1700,1704,1708,1712,1716,1724,1728,1740,1744,1752,1756,1760,1764,1768,1772,1776,1784,1788,1792,1796,1800,1804,1808,1812,1816,1824,1832,1836,1840,1844,1848,1852,1856,1860,1864,1868,1872,1876,1880,1884,1888,1892,1896,1900
    };

    constexpr size_t N = sizeof(forces)/sizeof(forces[0]);
    double f = std::fabs(force_N);

    if (N == 0) return 1500;

    // 正方向の PWM 値を求める（1500–1900 µs）
    double pwm;
    if (f <= forces[0]) {
        pwm = static_cast<double>(pwms[0]);
    } else if (f >= forces[N-1]) {
        pwm = static_cast<double>(pwms[N-1]);
    } else {
        const double* begin = forces;
        const double* end   = forces + N;
        const double* it    = std::lower_bound(begin, end, f);
        size_t idx = static_cast<size_t>(it - begin);
        if (idx == 0) {
            pwm = static_cast<double>(pwms[0]);
        } else {
            double f0 = forces[idx-1], f1 = forces[idx];
            double p0 = static_cast<double>(pwms[idx-1]);
            double p1 = static_cast<double>(pwms[idx]);
            double t  = (f - f0) / (f1 - f0);
            pwm = p0 + t * (p1 - p0);
        }
    }

    // 負方向: センター(1500)を軸に対称反転 → 1500–1100 µs
    if (force_N < 0.0) {
        pwm = 3000.0 - pwm;
    }

    if (pwm < 0.0)      pwm = 0.0;
    if (pwm > 65535.0)  pwm = 65535.0;
    return static_cast<uint16_t>(std::round(pwm));
}

} // namespace thrust_profile
