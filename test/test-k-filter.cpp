#include "k-filter.hpp"
#include "test-utilities.hpp"
#include <algorithm>
#include <numeric>
#include <doctest/doctest.h>

TEST_CASE("K-filter behaves as expected") {
    KFilter filter(48000, 1);

    auto test_sine = sineWave<double>(1000.0, 48000.0, 10*48000, 2, -23.0);
    std::vector<double> filtered;
    filtered.reserve(test_sine.size());
    for (auto val : test_sine){
        filtered.push_back(filter.apply(val, 0));
    }
    double z_before = std::accumulate(test_sine.cbegin(), test_sine.cend(), 0.0, [](auto sum, auto val){ return sum + val*val;}) / test_sine.size();
    double z_after = std::accumulate(filtered.cbegin(), filtered.cend(), 0.0, [](auto sum, auto val){ return sum + val*val;}) / filtered.size();

    double LUFS = -0.691 + 10*std::log10(2*z_after);
    double LUFS_unfiltered = -0.691 + 10*std::log10(2*z_before);
//    std::transform(test_sine.cbegin(), test_sine.cend(), std::back_inserter(filtered), [&filter](auto val){return filter.apply(val, 1);});
//    double val = *std::max_element(test_sine.cbegin(), test_sine.cend());
//    CHECK(*std::max_element(test_sine.cbegin(), test_sine.cend()) == doctest::Approx(*std::max_element(filtered.cbegin(), filtered.cend())));
//    CHECK(*std::min_element(test_sine.cbegin(), test_sine.cend()) == doctest::Approx(*std::min_element(filtered.cbegin(), filtered.cend())));
}
