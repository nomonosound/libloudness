#include "meter.hpp"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <numeric>

TEST_CASE("Test configuration", "[meter][parameters][config]") {
    constexpr auto min_samplerate = loudness::Samplerate(loudness::min_samplerate);
    constexpr auto max_samplerate = loudness::Samplerate(loudness::max_samplerate);
    constexpr auto min_channels = loudness::NumChannels(1);
    constexpr auto max_channels = loudness::NumChannels(loudness::max_channels);

    SECTION ("Illegal window sizes"){
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK_THROWS_AS(meter.setMaxWindow(std::numeric_limits<unsigned long>::max()), std::domain_error);
        CHECK_FALSE(meter.setMaxWindow(0));
    }
    SECTION ("History sizes"){
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK(meter.setMaxHistory(0));
        CHECK(meter.setMaxHistory(std::numeric_limits<unsigned long>::max()));
    }
    SECTION ("Strong parameter types"){
        CHECK_THROWS_MATCHES(loudness::NumChannels(0), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::NumChannels(loudness::max_channels + 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::Samplerate(loudness::min_samplerate - 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::Samplerate(loudness::max_samplerate + 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
    }
    SECTION ("Constructor parameters"){
        CHECK_NOTHROW(loudness::Meter<loudness::Mode::EBU_M>(min_channels, min_samplerate));
        CHECK_NOTHROW(loudness::Meter<loudness::Mode::EBU_M>(max_channels, max_samplerate));
    }
    SECTION ("changeParameters allowed arguments"){
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK_NOTHROW(meter.changeParameters(min_channels, min_samplerate));
        CHECK_NOTHROW(meter.changeParameters(max_channels, max_samplerate));
    }

}
