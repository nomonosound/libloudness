#include "ebur128.hpp"
#include "test-utilities.hpp"
#include <algorithm>
#include <doctest/doctest.h>
#include <numeric>

TEST_CASE("EBU Tech3341 test cases") {
    constexpr unsigned long samplerate = 48000;
    constexpr unsigned int channels = 2;
    auto sine_1000hz_60s_23LUFS = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -23.0);
    auto sine_1000hz_20s_33LUFS = sineWave<double>(1000.0, samplerate, 60*samplerate, channels, -33.0);
    auto sine_1000hz_10s_36LUFS = sineWave<double>(1000.0, samplerate, 10*samplerate, channels, -36.0);
    auto sine_1000hz_10s_72LUFS = sineWave<double>(1000.0, samplerate, 10*samplerate, channels, -72.0);

    Ebur128<EBUR128_MODE_I | EBUR128_MODE_S> meter(channels, samplerate);
    Ebur128<EBUR128_MODE_I | EBUR128_MODE_S | EBUR128_MODE_HISTOGRAM> meter_hist(channels, samplerate);
    SUBCASE("Test case 1"){
        constexpr double target = -23.0;

        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 20*samplerate / channels);

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter.loudnessShortterm();
        CHECK(loudness_s == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter.loudnessMomentary();
        CHECK(loudness_m == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
    SUBCASE("Test case 1 Histogram"){
        constexpr double target = -23.0;

        meter_hist.addFrames(sine_1000hz_60s_23LUFS.data(), 20*samplerate / channels);

        double loudness_i = meter_hist.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter_hist.loudnessShortterm();
        CHECK(loudness_s == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter_hist.loudnessMomentary();
        CHECK(loudness_m == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SUBCASE("Test case 2"){
        constexpr double target = -33.0;

        meter.addFrames(sine_1000hz_20s_33LUFS.data(), 20*samplerate / channels);

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter.loudnessShortterm();
        CHECK(loudness_s == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter.loudnessMomentary();
        CHECK(loudness_m == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
    SUBCASE("Test case 2 Histogram"){
        constexpr double target = -33.0;

        meter_hist.addFrames(sine_1000hz_20s_33LUFS.data(), 20*samplerate / channels);

        double loudness_i = meter_hist.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_s = meter_hist.loudnessShortterm();
        CHECK(loudness_s == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
        double loudness_m = meter_hist.loudnessMomentary();
        CHECK(loudness_m == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SUBCASE("Test case 3"){
        constexpr double target = -23.0;

        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate / channels);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);

        double loudness_i = meter.loudnessGlobal();

        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SUBCASE("Test case 3 Histogram"){
        constexpr double target = -23.0;

        meter_hist.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);

        double loudness_i = meter_hist.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SUBCASE("Test case 4"){
        constexpr double target = -23.0;
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate / channels);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate / channels);
        meter.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate / channels);

        double loudness_i = meter.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }

    SUBCASE("Test case 4 Histogram"){
        constexpr double target = -23.0;
        meter_hist.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_60s_23LUFS.data(), 60*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_10s_36LUFS.data(), 10*samplerate / channels);
        meter_hist.addFrames(sine_1000hz_10s_72LUFS.data(), 10*samplerate / channels);

        double loudness_i = meter_hist.loudnessGlobal();
        CHECK(loudness_i == doctest::Approx(target).scale(1.0/-target).epsilon(0.1));
    }
}
