#ifndef TEST_UTILITIES
#define TEST_UTILITIES
#include <algorithm>
#include <cmath>
#include <vector>

inline double dbFSToLinear(double dbFS) {
    return std::pow(10, dbFS / 20.0);
}

template<typename T>
auto sineWave(double frequency, double samplerate, long num_samples, unsigned int channels, double dbFS) -> std::vector<T> {
    const double scale = dbFSToLinear(dbFS);
    const double factor = 2 * std::numbers::pi * frequency / samplerate;
    std::vector<T> output;
    output.reserve(num_samples);
    for (long i = 0; i < num_samples; ++i){
        for (int c = 0; c < channels; ++c){
            output.push_back(scale*std::sin(static_cast<T>(i*factor)));
        }
    }
    return output;
}
#endif // TEST_UTILITIES
