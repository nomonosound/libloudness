#ifndef UTILS_HPP
#define UTILS_HPP

#include <cmath>
#include <ranges>
#include <algorithm>
#include <vector>

#include <gcem.hpp>
static inline double energyToLoudness(double energy) { return 10 * std::log10(energy) - 0.691; }

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

/* This changes the order of the range */
inline auto medianInPlace(std::ranges::random_access_range auto&& range)
{
    const size_t n = range.size() / 2;
    if (n == 0) [[unlikely]]{
        return -HUGE_VAL;
    }
    std::ranges::nth_element(range, range.begin() + n);
    return (range.size() % 2) ? range[n] : (range[n] + *std::max_element(range.begin(), range.begin() + n)) / 2;
}

inline auto median(std::ranges::range auto&& range)
{
    using T = std::ranges::range_value_t<decltype(range)>;
    std::vector<T> copied;
    std::ranges::copy(range, std::back_inserter(copied));
    return medianInPlace(copied);
}

#if defined(__SSE2_MATH__) || defined(_M_X64) || _M_IX86_FP >= 2
// TODO: Add support for arm
#include <immintrin.h>

class ScopedFTZ {
public:
    ScopedFTZ() : mxcsr_(_mm_getcsr())
    {
        _mm_setcsr(mxcsr_ | _MM_FLUSH_ZERO_ON | _MM_DENORMALS_ZERO_ON);
    }
    ~ScopedFTZ(){
        _mm_setcsr(mxcsr_);
    }
private:
    const unsigned int mxcsr_;
};

#else
class ScopedFTZ {};
#warning "manual FTZ is being used, please enable SSE2 (-msse2 -mfpmath=sse)"
#define MANUALLY_FTZ
#endif

#endif  // UTILS_HPP
