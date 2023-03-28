#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>

#include <gcem.hpp>
static inline double EnergyToLoudness(double energy) { return 10 * std::log10(energy) - 0.691; }

consteval double loudnessToEnergy(double lufs) { return gcem::pow(10.0, (lufs + 0.691) / 10.0); }

template <typename T>
inline T sinc(T x)
    requires std::floating_point<T>
{
    return std::sin(x) / x;
}

template <typename T>
static constexpr bool safe_size_mul(T nmemb, T size, T* result) requires std::unsigned_integral<T>
{
    /* Adapted from OpenBSD reallocarray. */
    constexpr T mul_no_overflow = static_cast<T>(1) << sizeof(T) * 4;
    if ((nmemb >= mul_no_overflow || size >= mul_no_overflow) && /**/
        nmemb > 0 && std::numeric_limits<T>::max() / nmemb < size) {
        return false;
    }
    *result = nmemb * size;
    return true;
}

template <typename T>
consteval double getScalingFactor()
    requires std::integral<T>
{
    return std::max<double>(-static_cast<double>(std::numeric_limits<T>::min()),
                            static_cast<double>(std::numeric_limits<T>::max()));
}

template <typename T>
consteval double getScalingFactor()
    requires std::floating_point<T>
{
    return 1.0;
}

#endif  // UTILS_HPP
