#include "meter.hpp"
#include "test-utilities.hpp"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <numeric>

#include <sndfile.h>

constexpr unsigned long prime_samplerate = 69313;

TEMPLATE_TEST_CASE("EBU Tech3341 I-Mode test cases", "[Tech3341][EBU][integrated][median]",
                   (loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S>),
                   (loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S | loudness::Mode::Histogram>)) {
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    auto sine_1000hz_60s_23LUFS = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -23.0);
    auto sine_1000hz_20s_33LUFS = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -33.0);
    auto sine_1000hz_10s_36LUFS = sineWave<double>(1000.0, samplerate, 10*samplerate, channels, -36.0);
    auto sine_1000hz_10s_72LUFS = sineWave<double>(1000.0, samplerate, 10*samplerate, channels, -72.0);

    TestType meter(channels, samplerate);
    SECTION("Test case 1"){
        constexpr double target = -23.0;

        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));

        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, 0.1));
    }

    SECTION("Test case 2"){
        constexpr double target = -33.0;

        meter.addFrames(sine_1000hz_20s_33LUFS.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));

        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, 0.1));
    }

    SECTION("Test case 3"){
        constexpr double target = -23.0;

        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));

        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, 0.1));
    }
    SECTION("Test case 4"){
        constexpr double target = -23.0;
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, 0.1));
    }
    SECTION("Test case 5"){
        constexpr double target = -23.0;
        constexpr double median_target = -26.0;
        auto sine_wave_26 = sineWave<double>(1000.0, samplerate, 20*samplerate, channels, -26.0);
        auto sine_wave_20 = sineWave<double>(1000.0, samplerate, std::lround(20.1*samplerate), channels, -20.0);
        meter.addFrames(sine_wave_26.data(), 20*samplerate);
        meter.addFrames(sine_wave_20.data(), std::lround(20.1*samplerate));
        meter.addFrames(sine_wave_26.data(), 20*samplerate);

        CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(median_target, 0.1));
    }
    SECTION("Test case 6"){
        TestType meter_5chn(5, samplerate);
        constexpr double target = -23.0;
        auto sine_wave = sineWave<double>(1000.0, samplerate, 20*samplerate, 5, {-28.0,-28.0,-24.0, -30.0, -30.0});
        meter_5chn.addFrames(sine_wave.data(), 20*samplerate);

        CHECK_THAT(meter_5chn.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THAT(meter_5chn.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, 0.1));
    }
}

TEST_CASE("EBU Tech3341 S-Mode and M-Mode test cases", "[Tech3341][EBU][shortterm][momentary]") {
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    loudness::Meter<loudness::Mode::EBU_S> meter(channels, samplerate);
    SECTION("Test case 9"){
        const double target = -23.0;
        const long frames_30 = std::lround(1.66*samplerate);
        const long frames_20 = std::lround(1.34*samplerate);
        auto sine_wave_30 = sineWave<double>(1000.0, samplerate, frames_30, channels, -30.0);
        auto sine_wave_20 = sineWave<double>(1000.0, samplerate, frames_20, channels, -20.0);
        for (int i = 0; i < 5; ++i){
            meter.addFrames(sine_wave_20.data(), frames_20);
            if (i > 0){
                CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
            }
            meter.addFrames(sine_wave_30.data(), frames_30);
            CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        }
    }
    SECTION("Test case 10"){
        const double target = -23.0;
        auto sine_wave_23 = sineWave<double>(1000.0, samplerate, 3*samplerate, channels, -23.0);
        auto silence = std::vector<double>(3*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            meter.addFrames(silence.data(), std::lround(0.15*i));
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).margin(0.1));
            meter.addFrames(sine_wave_23.data(), 3*samplerate);
            CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
            meter.addFrames(silence.data(), 1*samplerate);
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).margin(0.1));
        }
    }
    SECTION("Test case 11"){
        auto silence = std::vector<double>(3*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            const double target = -38.0 + i;
            auto sine_wave = sineWave<double>(1000.0, samplerate, 3*samplerate, channels, target);
            meter.addFrames(silence.data(), std::lround(0.15*i));
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).margin(0.1));
            meter.addFrames(sine_wave.data(), 3*samplerate);
            CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
            meter.addFrames(silence.data(), std::lround(3 - i*0.15)*samplerate);
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).margin(0.1));
        }
    }
    SECTION("Test case 12"){
        const double target = -23.0;
        const long frames_30 = std::lround(0.22*samplerate);
        const long frames_20 = std::lround(0.18*samplerate);
        auto sine_wave_30 = sineWave<double>(1000.0, samplerate, frames_30, channels, -30.0);
        auto sine_wave_20 = sineWave<double>(1000.0, samplerate, frames_20, channels, -20.0);
        for (int i = 0; i < 25; ++i){
            meter.addFrames(sine_wave_20.data(), frames_20);
            if (i > 1){
                CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));
            }
            meter.addFrames(sine_wave_30.data(), frames_30);
            if (i > 1){
                CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));
            }
        }
    }
    SECTION("Test case 13"){
        const double target = -23.0;
        auto sine_wave_23 = sineWave<double>(1000.0, samplerate, std::lround(0.4*samplerate), channels, -23.0);
        auto silence = std::vector<double>(1*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            meter.addFrames(silence.data(), std::lround(0.02*i));
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).margin(0.1));
            meter.addFrames(sine_wave_23.data(), std::lround(0.4*samplerate));
            CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));
            meter.addFrames(silence.data(), 1*samplerate);
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).margin(0.1));
        }
    }
    SECTION("Test case 14"){
        auto silence = std::vector<double>(1*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            const double target = -38.0 + i;
            auto sine_wave = sineWave<double>(1000.0, samplerate, std::lround(0.4*samplerate), channels, target);
            meter.addFrames(silence.data(), std::lround(0.02*i));
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).margin(0.1));
            meter.addFrames(sine_wave.data(), std::lround(0.4*samplerate));
            CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));
            meter.addFrames(silence.data(), std::lround(0.4 - i*0.02)*samplerate);
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).margin(0.1));
        }
    }
}

TEST_CASE("Test non-interleaved data", "[non-interleaved]"){
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    loudness::Meter<loudness::Mode::EBU_S> meter(channels, samplerate);
    SECTION("Test case 9"){
        const double target = -23.0;
        const long frames_30 = std::lround(1.66*samplerate);
        const long frames_20 = std::lround(1.34*samplerate);
        auto sine_wave_30 = sineWaveChannels<double>(1000.0, samplerate, frames_30, channels, -30.0);
        auto sine_wave_20 = sineWaveChannels<double>(1000.0, samplerate, frames_20, channels, -20.0);
        for (int i = 0; i < 5; ++i){
            meter.addFrames(sine_wave_20, frames_20);
            if (i > 0){
                CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
            }
            meter.addFrames(sine_wave_30, frames_30);
            CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        }
        std::vector<const double*> sine_wave2 = {sine_wave_20[0].data(), sine_wave_20[1].data()};
        std::vector<const double*> sine_wave3 = {sine_wave_30[0].data(), sine_wave_30[1].data()};
        meter.addFrames(sine_wave2, frames_20);
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        meter.addFrames(sine_wave3.data(), frames_30);
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
        std::vector<double*> sine_wave4 = {sine_wave_20[0].data(), sine_wave_20[1].data()};
        meter.addFrames(sine_wave4, frames_20);
        CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, 0.1));
    }
}

TEST_CASE("EBU Tech3341 true peak test cases", "[Tech3341][EBU][peak][true-peak]") {
    const unsigned long samplerate = GENERATE(48000, prime_samplerate, 96000, 190000);
    constexpr unsigned int channels = 2;
    loudness::Meter<loudness::Mode::TruePeak> meter(channels, samplerate);
    UNSCOPED_INFO(samplerate);
    SECTION("Test case 15"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(std::lround(samplerate/4.0), samplerate, channels, 0.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK_THAT(meter.samplePeak(0), Catch::Matchers::WithinAbs(0.5, 1e-9));
        CHECK_THAT(meter.samplePeak(1), Catch::Matchers::WithinAbs(0.5, 1e-9));
        CHECK_THAT(20*std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), AsymetricMarginMatcher(target, 0.3, 0.2));
    }
    SECTION("Test case 16"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(std::lround(samplerate/4.0), samplerate, channels, 45.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK_THAT(20*std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), AsymetricMarginMatcher(target, 0.3, 0.2));
    }
    SECTION("Test case 17"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(std::lround(samplerate/6.0), samplerate, channels, 60.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK_THAT(20*std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), AsymetricMarginMatcher(target, 0.3, 0.2));
    }
    SECTION("Test case 18"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(std::lround(samplerate/8.0), samplerate, channels, 67.5, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK_THAT(20*std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), AsymetricMarginMatcher(target, 0.3, 0.2));
    }
    SECTION("Test case 19"){
        const double target = 3.0;
        auto sine_wave = sineWaveTP<double>(std::lround(samplerate/4.0), samplerate, channels, 45.0, 1.41);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK_THAT(20*std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), AsymetricMarginMatcher(target, 0.3, 0.2));
    }
}

TEST_CASE("EBU Tech 3342 test cases", "[Tech3342][EBU][LRA][loudness-range]")
{
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    loudness::Meter<loudness::Mode::EBU_LRA> meter(channels, samplerate);
    SECTION("Test case 1"){
        constexpr double target = 10;
        auto sine_wave_20 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -20.0);
        auto sine_wave_30 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -30.0);
        meter.addFrames(sine_wave_20.data(), sine_wave_20.size() / channels);
        meter.addFrames(sine_wave_30.data(), sine_wave_30.size() / channels);
        CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(target, 0.1));
    }
    SECTION("Test case 2"){
        constexpr double target = 5;
        auto sine_wave_20 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -20.0);
        auto sine_wave_15 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -15.0);
        meter.addFrames(sine_wave_20.data(), sine_wave_20.size() / channels);
        meter.addFrames(sine_wave_15.data(), sine_wave_15.size() / channels);
        CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(target, 0.1));
    }
    SECTION("Test case 3"){
        constexpr double target = 20;
        auto sine_wave_40 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -40.0);
        auto sine_wave_20 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -20.0);
        meter.addFrames(sine_wave_40.data(), sine_wave_40.size() / channels);
        meter.addFrames(sine_wave_20.data(), sine_wave_20.size() / channels);
        CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(target, 0.1));
    }
    SECTION("Test case 4"){
        constexpr double target = 15;
        auto sine_wave_50 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -50.0);
        auto sine_wave_35 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -35.0);
        auto sine_wave_20 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -20.0);
        meter.addFrames(sine_wave_50.data(), sine_wave_50.size() / channels);
        meter.addFrames(sine_wave_35.data(), sine_wave_35.size() / channels);
        meter.addFrames(sine_wave_20.data(), sine_wave_20.size() / channels);
        meter.addFrames(sine_wave_35.data(), sine_wave_35.size() / channels);
        meter.addFrames(sine_wave_50.data(), sine_wave_50.size() / channels);
        CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(target, 0.1));
    }
}

// This test is disabled by default since you need to download the test files located at
// https://tech.ebu.ch/files/live/sites/tech/files/shared/testmaterial/ebu-loudness-test-setv05.zip
// And place them in test/reference_files/
TEST_CASE("File dependent tests from Tech 3341 and Tech 3342", "[.][EBU][Tech3341][Tech3342]")
{
    const std::string reference_folder = std::string(TEST_DIR) + "reference_files/";
    constexpr double lufs_target = -23.0;
    SECTION ("Authentic program 1,  narrow Loudness Range"){
            constexpr double lra_target = 5.0;
            const std::string filename = reference_folder + "seq-3341-7_seq-3342-5-24bit.wav";
            SF_INFO format{};
            auto* file = sf_open(filename.c_str(), SFM_READ, &format);
            REQUIRE(file != nullptr);
            std::vector<float> interleaved(format.frames * format.channels);
            const auto num_frames = sf_readf_float(file, interleaved.data(), format.frames);
            sf_close(file);
            loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_LRA> meter(format.channels, format.samplerate);

            meter.addFrames(interleaved.data(), num_frames);

            // Tech 3341 case 7
            CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(lufs_target, 0.1));
            // Tech 3342 case 5
            CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(lra_target, 0.1));
    }
    SECTION ("Authentic program 2,  wide Loudness Range"){
            constexpr double lra_target = 15.0;
            const std::string filename = reference_folder + "seq-3341-2011-8_seq-3342-6-24bit-v02.wav";
            SF_INFO format{};
            auto* file = sf_open(filename.c_str(), SFM_READ, &format);
            REQUIRE(file != nullptr);
            std::vector<float> interleaved(format.frames * format.channels);
            const auto num_frames = sf_readf_float(file, interleaved.data(), format.frames);
            sf_close(file);
            loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_LRA> meter(format.channels, format.samplerate);

            meter.addFrames(interleaved.data(), num_frames);

            // Tech 3341 case 8
            CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(lufs_target, 0.1));
            // Tech 3342 case 6
            CHECK_THAT(meter.loudnessRange(), Catch::Matchers::WithinAbs(lra_target, 0.1));
    }
    SECTION ("Tech 3341 case 20-23"){
        constexpr double target = 0.0;
        int number = GENERATE(20,21,22,23);
        const std::string filename = reference_folder + "seq-3341-" + std::to_string(number) + "-24bit.wav.wav";
        SF_INFO format{};
        auto file = sf_open(filename.c_str(), SFM_READ, &format);
        REQUIRE(file != nullptr);
        std::vector<float> interleaved(format.frames * format.channels);
        const auto num_frames = sf_readf_float(file, interleaved.data(), format.frames);
        sf_close(file);

        loudness::Meter<loudness::Mode::TruePeak> meter(format.channels, format.samplerate);
        meter.addFrames(interleaved.data(), num_frames);
        CHECK_THAT(20*std::log10(meter.truePeak(0)), Catch::Matchers::WithinAbs(target, 0.2));
        CHECK_THAT(20*std::log10(meter.truePeak(1)), Catch::Matchers::WithinAbs(target, 0.2));
    }
}

TEST_CASE("Test multi-global-loudness", "[integrated][multi]"){
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = -23.0;
    auto sine_wave1 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<double>(1000, samplerate, 60*samplerate, channels, -23.0);

    std::vector<loudness::Meter<loudness::Mode::EBU_I>> meters;
    meters.emplace_back(channels, samplerate);
    meters.emplace_back(channels, samplerate);

    meters[0].addFrames(sine_wave1.data(), 20*samplerate);
    meters[1].addFrames(sine_wave2.data(), 60*samplerate);

    CHECK_THAT(loudnessGlobalMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}

TEST_CASE("Test multi-loudness-range", "[loudness-range][multi]"){
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = 13.0;
    auto sine_wave1 = sineWave<double>(1000, samplerate, 20*samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<double>(1000, samplerate, 60*samplerate, channels, -23.0);

    std::vector<loudness::Meter<loudness::Mode::EBU_LRA>> meters;
    meters.emplace_back(channels, samplerate);
    meters.emplace_back(channels, samplerate);

    meters[0].addFrames(sine_wave1.data(), 20*samplerate);
    meters[1].addFrames(sine_wave2.data(), 60*samplerate);

    CHECK_THAT(loudnessRangeMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}

TEST_CASE("Benchmark Integrated Loudness", "[.benchmark][integrated]")
{
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    auto sine_wave = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -23.0);
    BENCHMARK_ADVANCED("Benchmark sine wave")(Catch::Benchmark::Chronometer chronometer){
        loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S> meter(channels, samplerate);
        chronometer.measure([&sine_wave, &meter, frames = 60*samplerate / channels]{
            meter.addFrames(sine_wave.data(), frames);
            return meter.loudnessGlobal();
        });
    };
    BENCHMARK_ADVANCED("Benchmark sine wave hist")(Catch::Benchmark::Chronometer chronometer){
        loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S | loudness::Mode::Histogram> meter(channels, samplerate);
        chronometer.measure([&sine_wave, &meter, frames = 60*samplerate / channels]{
            meter.addFrames(sine_wave.data(), frames);
            return meter.loudnessGlobal();
        });
    };
}
TEST_CASE("Benchmark True Peak", "[.benchmark][integrated][true-peak]")
{
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    auto sine_wave = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -23.0);
    BENCHMARK_ADVANCED("Benchmark sine wave")(Catch::Benchmark::Chronometer chronometer){
        loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::TruePeak> meter(channels, samplerate);
        chronometer.measure([&sine_wave, &meter, frames = 60*samplerate / channels]{
            meter.addFrames(sine_wave.data(), frames);
            return meter.loudnessGlobal();
        });
    };
}
