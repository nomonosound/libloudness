#include "ebur128.hpp"
#include "test-utilities.hpp"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <numeric>


TEMPLATE_TEST_CASE("EBU Tech3341 I-Mode test cases", "[Tech3341][EBU][integrated][median]",
                   (Ebur128<EBUR128_MODE_I | EBUR128_MODE_S>),
                   (Ebur128<EBUR128_MODE_I | EBUR128_MODE_S | EBUR128_MODE_HISTOGRAM>)) {
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

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter.loudnessShortterm();
        CHECK(loudness_s == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter.loudnessMomentary();
        CHECK(loudness_m == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));

        CHECK(meter.loudnessGlobalMedian() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SECTION("Test case 2"){
        constexpr double target = -33.0;

        meter.addFrames(sine_1000hz_20s_33LUFS.data(), 20*samplerate);

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter.loudnessShortterm();
        CHECK(loudness_s == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter.loudnessMomentary();
        CHECK(loudness_m == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));

        CHECK(meter.loudnessGlobalMedian() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SECTION("Test case 3"){
        constexpr double target = -23.0;

        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);

        double loudness_i = meter.loudnessGlobal();

        CHECK(loudness_i == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));

        CHECK(meter.loudnessGlobalMedian() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
    SECTION("Test case 4"){
        constexpr double target = -23.0;
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate);
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate);

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));

        CHECK(meter.loudnessGlobalMedian() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
    SECTION("Test case 5"){
        constexpr double target = -23.0;
        constexpr double median_target = -26.0;
        auto sine_wave_26 = sineWave<double>(1000.0, samplerate, 20*samplerate, channels, -26.0);
        auto sine_wave_20 = sineWave<double>(1000.0, samplerate, std::round(20.1*samplerate), channels, -20.0);
        meter.addFrames(sine_wave_26.data(), 20*samplerate);
        meter.addFrames(sine_wave_20.data(), std::round(20.1*samplerate));
        meter.addFrames(sine_wave_26.data(), 20*samplerate);

        CHECK(meter.loudnessGlobal() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        CHECK(meter.loudnessGlobalMedian() == Catch::Approx(median_target).scale(1.0/-median_target).epsilon(0.1));
    }
    SECTION("Test case 6"){
        TestType meter_5chn(5, samplerate);
        constexpr double target = -23.0;
        auto sine_wave = sineWave<double>(1000.0, samplerate, 20*samplerate, 5, {-28.0,-28.0,-24.0, -30.0, -30.0});
        meter_5chn.addFrames(sine_wave.data(), 20*samplerate);

        CHECK(meter_5chn.loudnessGlobal() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        CHECK(meter_5chn.loudnessGlobalMedian() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
}

TEST_CASE("EBU Tech3341 S-Mode and M-Mode test cases", "[Tech3341][EBU][shortterm][momentary]") {
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    Ebur128<EBUR128_MODE_S> meter(channels, samplerate);
    SECTION("Test case 9"){
        const double target = -23.0;
        const long frames_30 = std::lround(1.66*samplerate);
        const long frames_20 = std::lround(1.34*samplerate);
        auto sine_wave_30 = sineWave<double>(1000.0, samplerate, frames_30, channels, -30.0);
        auto sine_wave_20 = sineWave<double>(1000.0, samplerate, frames_20, channels, -20.0);
        for (int i = 0; i < 5; ++i){
            meter.addFrames(sine_wave_20.data(), frames_20);
            if (i > 0){
                CHECK(meter.loudnessShortterm() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            }
            meter.addFrames(sine_wave_30.data(), frames_30);
            CHECK(meter.loudnessShortterm() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        }
    }
    SECTION("Test case 10"){
        const double target = -23.0;
        auto sine_wave_23 = sineWave<double>(1000.0, samplerate, 3*samplerate, channels, -23.0);
        auto silence = std::vector<double>(3*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            meter.addFrames(silence.data(), std::lround(0.15*i));
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(sine_wave_23.data(), 3*samplerate);
            CHECK(meter.loudnessShortterm() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(silence.data(), 1*samplerate);
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        }
    }
    SECTION("Test case 11"){
        auto silence = std::vector<double>(3*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            const double target = -38.0 + i;
            auto sine_wave = sineWave<double>(1000.0, samplerate, 3*samplerate, channels, target);
            meter.addFrames(silence.data(), std::lround(0.15*i));
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(sine_wave.data(), 3*samplerate);
            CHECK(meter.loudnessShortterm() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(silence.data(), std::lround(3 - i*0.15)*samplerate);
            CHECK(meter.loudnessShortterm() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
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
                CHECK(meter.loudnessMomentary() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            }
            meter.addFrames(sine_wave_30.data(), frames_30);
            if (i > 1){
                CHECK(meter.loudnessMomentary() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            }
        }
    }
    SECTION("Test case 13"){
        const double target = -23.0;
        auto sine_wave_23 = sineWave<double>(1000.0, samplerate, std::lround(0.4*samplerate), channels, -23.0);
        auto silence = std::vector<double>(1*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            meter.addFrames(silence.data(), std::lround(0.02*i));
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(sine_wave_23.data(), std::lround(0.4*samplerate));
            CHECK(meter.loudnessMomentary() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(silence.data(), 1*samplerate);
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        }
    }
    SECTION("Test case 14"){
        auto silence = std::vector<double>(1*channels*samplerate);
        for (int i = 0; i < 20; ++i){
            const double target = -38.0 + i;
            auto sine_wave = sineWave<double>(1000.0, samplerate, std::lround(0.4*samplerate), channels, target);
            meter.addFrames(silence.data(), std::lround(0.02*i));
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(sine_wave.data(), std::lround(0.4*samplerate));
            CHECK(meter.loudnessMomentary() == Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
            meter.addFrames(silence.data(), std::lround(0.4 - i*0.02)*samplerate);
            CHECK(meter.loudnessMomentary() <= Catch::Approx(target).scale(1.0/-target).epsilon(0.1));
        }
    }
}

TEST_CASE("EBU Tech3341 true peak test cases", "[Tech3341][EBU][peak][true-peak]") {
    const unsigned long samplerate = GENERATE(48000, 96000, 191999);
    constexpr unsigned int channels = 2;
    Ebur128<EBUR128_MODE_TRUE_PEAK> meter(channels, samplerate);
    UNSCOPED_INFO(samplerate);
    SECTION("Test case 15"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(samplerate/4, samplerate, channels, 0.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK(meter.samplePeak(0) == Catch::Approx(0.5));
        CHECK(meter.samplePeak(1) == Catch::Approx(0.5));
        CHECK(20*std::log10(meter.truePeak(0)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
        CHECK(20*std::log10(meter.truePeak(1)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
    }
    SECTION("Test case 16"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(samplerate/4, samplerate, channels, 45.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK(20*std::log10(meter.truePeak(0)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
        CHECK(20*std::log10(meter.truePeak(1)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
    }
    SECTION("Test case 17"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(samplerate/6, samplerate, channels, 60.0, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK(20*std::log10(meter.truePeak(0)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
        CHECK(20*std::log10(meter.truePeak(1)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
    }
    SECTION("Test case 18"){
        const double target = -6.0;
        auto sine_wave = sineWaveTP<double>(samplerate/8, samplerate, channels, 67.5, 0.5);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK(20*std::log10(meter.truePeak(0)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
        CHECK(20*std::log10(meter.truePeak(1)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
    }
    SECTION("Test case 19"){
        const double target = 3.0;
        auto sine_wave = sineWaveTP<double>(samplerate/4, samplerate, channels, 45.0, 1.41);
        meter.addFrames(sine_wave.data(), sine_wave.size() / channels);
        CHECK(20*std::log10(meter.truePeak(0)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
        CHECK(20*std::log10(meter.truePeak(1)) == Catch::Approx(target).scale(1.0/-target).epsilon(0.2));
    }
}

TEST_CASE("Benchmark Integrated Loudness", "[.benchmark][integrated]")
{
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    auto sine_wave = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -23.0);
    BENCHMARK_ADVANCED("Benchmark sine wave")(Catch::Benchmark::Chronometer chronometer){
        Ebur128<EBUR128_MODE_I | EBUR128_MODE_S> meter(channels, samplerate);
        chronometer.measure([&sine_wave, &meter, frames = 60*samplerate / channels]{
            meter.addFrames(sine_wave.data(), frames);
            return meter.loudnessGlobal();
        });
    };
    BENCHMARK_ADVANCED("Benchmark sine wave hist")(Catch::Benchmark::Chronometer chronometer){
        Ebur128<EBUR128_MODE_I | EBUR128_MODE_S | EBUR128_MODE_HISTOGRAM> meter(channels, samplerate);
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
        Ebur128<EBUR128_MODE_I | EBUR128_MODE_TRUE_PEAK> meter(channels, samplerate);
        chronometer.measure([&sine_wave, &meter, frames = 60*samplerate / channels]{
            meter.addFrames(sine_wave.data(), frames);
            return meter.loudnessGlobal();
        });
    };
}
