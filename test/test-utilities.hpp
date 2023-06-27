#ifndef TEST_UTILITIES
#define TEST_UTILITIES
#include <algorithm>
#include <cmath>
#include <vector>
#include <assert.h>

inline double dbFSToLinear(double dbFS) {
    return std::pow(10, dbFS / 20.0);
}

template<typename T>
auto sineWave(double frequency, double samplerate, long num_samples, unsigned int channels, const std::vector<double>& dbFS) -> std::vector<T> {
    assert (dbFS.size() == channels);
    std::vector<double> scale(channels);
    std::transform(dbFS.begin(), dbFS.end(), scale.begin(), [](auto val){return dbFSToLinear(val);});
    const double factor = 2 * std::numbers::pi * frequency / samplerate;
    std::vector<T> output;
    output.reserve(num_samples*channels);
    for (long i = 0; i < num_samples; ++i){
        for (unsigned int c = 0; c < channels; ++c){
            output.push_back(scale[c]*std::sin(static_cast<T>(i*factor)));
        }
    }
    return output;
}

template<typename T>
auto sineWave(double frequency, double samplerate, long num_samples, unsigned int channels, double dbFS) -> std::vector<T> {
    std::vector dbFSs(channels, dbFS);
    return sineWave<T>(frequency, samplerate, num_samples, channels, dbFSs);
}
#endif // TEST_UTILITIES
