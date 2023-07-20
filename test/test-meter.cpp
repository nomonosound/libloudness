#include <algorithm>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <numeric>

#include "meter.hpp"
#include "test-utilities.hpp"

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
    SECTION ("Accessing per channel data"){
        constexpr int num_channels = 8;
        loudness::Meter<loudness::Mode::TruePeak> meter(loudness::NumChannels(num_channels), min_samplerate);
        CHECK_THROWS_AS(meter.truePeak(num_channels), std::invalid_argument);
        CHECK_THROWS_AS(meter.lastTruePeak(num_channels), std::invalid_argument);
        CHECK_THROWS_AS(meter.samplePeak(num_channels), std::invalid_argument);
        CHECK_THROWS_AS(meter.lastSamplePeak(num_channels), std::invalid_argument);
        CHECK_THROWS_AS(meter.truePeak(-1), std::invalid_argument);
        CHECK_THROWS_AS(meter.lastTruePeak(-1), std::invalid_argument);
        CHECK_THROWS_AS(meter.samplePeak(-1), std::invalid_argument);
        CHECK_THROWS_AS(meter.lastSamplePeak(-1), std::invalid_argument);
        CHECK(meter.truePeak(num_channels - 1) == 0.0);
        CHECK(meter.lastTruePeak(num_channels - 1) == 0.0);
        CHECK(meter.samplePeak(num_channels - 1) == 0.0);
        CHECK(meter.lastSamplePeak(num_channels - 1) == 0.0);
        CHECK(meter.truePeak(0) == 0.0);
        CHECK(meter.lastTruePeak(0) == 0.0);
        CHECK(meter.samplePeak(0) == 0.0);
        CHECK(meter.lastSamplePeak(0) == 0.0);
    }
}

TEMPLATE_TEST_CASE("Test all input data configurations", "[meter][input]", float, double, int16_t, int32_t){
    constexpr auto samplerate = 48000;
    constexpr auto channels = 2;

    loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S> meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};
    SECTION("Test interleaved"){
        constexpr double target = -23.0;
        constexpr double margin = 0.1;
        const auto interleaved = sineWave<TestType>(1000.0, samplerate, 20*samplerate, channels, -23.0);

        meter.addFrames(interleaved.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFrames(interleaved, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFramesMT(interleaved.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFramesMT(interleaved, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));
    }
    SECTION("Test channel based"){
        constexpr double target = -23.0;
        constexpr double margin = 0.1;
        const auto channel_audio = sineWaveChannels<TestType>(1000.0, samplerate, 20*samplerate, channels, -23.0);
        std::vector<const TestType*> channel_ptrs = {channel_audio[0].data(), channel_audio[1].data()};

        meter.addFrames(channel_audio, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFrames(channel_ptrs, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFrames(channel_ptrs.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFramesMT(channel_audio, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFramesMT(channel_ptrs, 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));

        meter.addFramesMT(channel_ptrs.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));
    }
}

TEMPLATE_LIST_TEST_CASE("Test multi-global-loudness", "[integrated][multi]", DataTypes){
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = -23.0;
    auto sine_wave1 = sineWave<TestType>(1000, samplerate, 20*samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<TestType>(1000, samplerate, 60*samplerate, channels, -23.0);

    std::vector<loudness::Meter<loudness::Mode::EBU_I>> meters;
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));

    meters[0].addFrames(sine_wave1.data(), 20*samplerate);
    meters[1].addFrames(sine_wave2.data(), 60*samplerate);

    CHECK_THAT(loudnessGlobalMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}

TEMPLATE_LIST_TEST_CASE("Test multi-loudness-range", "[loudness-range][multi]", DataTypes){
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = 13.0;
    auto sine_wave1 = sineWave<TestType>(1000, samplerate, 20*samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<TestType>(1000, samplerate, 60*samplerate, channels, -23.0);

    std::vector<loudness::Meter<loudness::Mode::EBU_LRA>> meters;
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));

    meters[0].addFrames(sine_wave1.data(), 20*samplerate);
    meters[1].addFrames(sine_wave2.data(), 60*samplerate);

    CHECK_THAT(loudnessRangeMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}
