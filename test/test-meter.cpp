/* See LICENSE file for copyright and license details. */

#include <algorithm>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <list>
#include <numeric>

#include "loudness/meter.hpp"
#include "test-utilities.hpp"

TEST_CASE("Test configuration", "[meter][parameters][setChannel]")
{
    constexpr auto min_samplerate = loudness::Samplerate(loudness::min_samplerate);
    constexpr auto max_samplerate = loudness::Samplerate(loudness::max_samplerate);
    constexpr auto min_channels = loudness::NumChannels(1);
    constexpr auto max_channels = loudness::NumChannels(loudness::max_channels);

    SECTION("Illegal window sizes")
    {
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK_THROWS_AS(meter.setMaxWindow(std::numeric_limits<unsigned long>::max()), std::domain_error);
        CHECK_FALSE(meter.setMaxWindow(0));
    }
    SECTION("History sizes")
    {
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK(meter.setMaxHistory(0));
        CHECK(meter.setMaxHistory(std::numeric_limits<unsigned long>::max()));
    }
    SECTION("Strong parameter types")
    {
        CHECK_THROWS_MATCHES(loudness::NumChannels(0), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::NumChannels(loudness::max_channels + 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::Samplerate(loudness::min_samplerate - 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
        CHECK_THROWS_MATCHES(loudness::Samplerate(loudness::max_samplerate + 1), std::domain_error,
                             Catch::Matchers::Message("Requested value not within allowed range"));
    }
    SECTION("Constructor arguments")
    {
        CHECK_NOTHROW(loudness::Meter<loudness::Mode::EBU_M>(min_channels, min_samplerate));
        CHECK_NOTHROW(loudness::Meter<loudness::Mode::EBU_M>(max_channels, max_samplerate));
    }
    SECTION("changeParameters allowed arguments")
    {
        loudness::Meter<loudness::Mode::EBU_M> meter(min_channels, loudness::Samplerate(12000));
        CHECK_NOTHROW(meter.changeParameters(min_channels, min_samplerate));
        CHECK_NOTHROW(meter.changeParameters(max_channels, max_samplerate));
    }
    SECTION("Per channel data")
    {
        constexpr int num_channels = 8;
        loudness::Meter<loudness::Mode::EBU_M | loudness::Mode::TruePeak> meter(loudness::NumChannels(num_channels),
                                                                                min_samplerate);
        CHECK_THROWS_AS(meter.setChannel(num_channels, loudness::Channel::Center), std::out_of_range);
        CHECK_THROWS_AS(meter.truePeak(num_channels), std::out_of_range);
        CHECK_THROWS_AS(meter.lastTruePeak(num_channels), std::out_of_range);
        CHECK_THROWS_AS(meter.samplePeak(num_channels), std::out_of_range);
        CHECK_THROWS_AS(meter.lastSamplePeak(num_channels), std::out_of_range);
        CHECK_THROWS_AS(meter.setChannel(-1, loudness::Channel::Center), std::out_of_range);
        CHECK_THROWS_AS(meter.truePeak(-1), std::out_of_range);
        CHECK_THROWS_AS(meter.lastTruePeak(-1), std::out_of_range);
        CHECK_THROWS_AS(meter.samplePeak(-1), std::out_of_range);
        CHECK_THROWS_AS(meter.lastSamplePeak(-1), std::out_of_range);
        CHECK_NOTHROW(meter.setChannel(num_channels - 1, loudness::Channel::Center));
        CHECK(meter.truePeak(num_channels - 1) == 0.0);
        CHECK(meter.lastTruePeak(num_channels - 1) == 0.0);
        CHECK(meter.samplePeak(num_channels - 1) == 0.0);
        CHECK(meter.lastSamplePeak(num_channels - 1) == 0.0);
        CHECK_NOTHROW(meter.setChannel(0, loudness::Channel::Center));
        CHECK(meter.truePeak(0) == 0.0);
        CHECK(meter.lastTruePeak(0) == 0.0);
        CHECK(meter.samplePeak(0) == 0.0);
        CHECK(meter.lastSamplePeak(0) == 0.0);
    }
}

TEST_CASE("Test window functions", "[meter][window]")
{
    constexpr long samplerate = 32000;
    constexpr int channels = 2;
    SECTION("M Mode has 400 ms window by default")
    {
        constexpr long num_samples = samplerate;
        constexpr double target = -23.0;
        loudness::Meter<loudness::Mode::EBU_M> meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};
        auto sine_wave = sineWave<float>(1000.0, samplerate, num_samples, channels, target);
        meter.addFrames(sine_wave, num_samples);
        CHECK_THAT(meter.loudnessWindow(400), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THROWS_AS(meter.loudnessWindow(401), std::domain_error);
    }
    SECTION("S Mode has 3000 ms window by default")
    {
        constexpr long num_samples = 4 * samplerate;
        constexpr double target = -23.0;
        loudness::Meter<loudness::Mode::EBU_S> meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};
        auto sine_wave = sineWave<float>(1000.0, samplerate, num_samples, channels, target);
        meter.addFrames(sine_wave, num_samples);
        CHECK_THAT(meter.loudnessWindow(3000), Catch::Matchers::WithinAbs(target, 0.1));
        CHECK_THROWS_AS(meter.loudnessWindow(3001), std::domain_error);
    }
    SECTION("Reading windows works as expected")
    {
        constexpr long num_samples = 3 * samplerate;
        constexpr double target1 = -23.0;
        constexpr double target2 = -36.0;
        const auto sine_wave1 = sineWave<float>(1000.0, samplerate, num_samples, channels, target1);
        const auto sine_wave2 = sineWave<float>(1000.0, samplerate, num_samples, channels, target2);
        loudness::Meter<loudness::Mode::EBU_M> meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};
        CHECK(meter.setMaxWindow(2500));
        CHECK_FALSE(meter.setMaxWindow(2500));
        meter.addFrames(sine_wave1, num_samples);
        CHECK_THAT(meter.loudnessWindow(2500), Catch::Matchers::WithinAbs(target1, 0.1));
        CHECK_THROWS_AS(meter.loudnessWindow(2501), std::domain_error);
        meter.addFrames(sine_wave2, samplerate / 2);
        CHECK_THAT(meter.loudnessWindow(400), Catch::Matchers::WithinAbs(target2, 0.1));
        CHECK(meter.setMaxWindow(700));
        CHECK_FALSE(meter.setMaxWindow(700));
        meter.addFrames(sine_wave1, num_samples);
        CHECK_THAT(meter.loudnessWindow(700), Catch::Matchers::WithinAbs(target1, 0.1));
        CHECK_THROWS_AS(meter.loudnessWindow(701), std::domain_error);
    }
}

TEST_CASE("Change parameters works as expected", "[meter][parameters]")
{
    constexpr long samplerate1 = 32000;
    constexpr int channels1 = 2;
    constexpr long samplerate2 = 48000;
    constexpr int channels2 = 1;
    constexpr double target1 = -23.0;
    constexpr double target2 = -20.0;
    const auto sine_wave1 = sineWave<float>(1000.0, samplerate1, 3 * samplerate1, channels1, target1);
    const auto sine_wave2 = sineWave<float>(1000.0, samplerate2, 3 * samplerate2, channels2, target2);
    loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::TruePeak> meter{loudness::NumChannels(channels1),
                                                                            loudness::Samplerate(samplerate1)};
    meter.addFrames(sine_wave1, 3 * samplerate1);
    CHECK_THAT(20 * std::log10(meter.truePeak(0)), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK_THAT(20 * std::log10(meter.truePeak(1)), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK_THAT(20 * std::log10(meter.truePeak()), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK(meter.changeParameters(loudness::NumChannels(channels2), loudness::Samplerate(samplerate2)));
    meter.addFrames(sine_wave2, 3 * samplerate2);
    CHECK_THAT(20 * std::log10(meter.truePeak(0)), Catch::Matchers::WithinAbs(-20.0, 0.1));
    CHECK_THAT(20 * std::log10(meter.truePeak()), Catch::Matchers::WithinAbs(-20.0, 0.1));
    CHECK_THROWS_AS(meter.truePeak(1), std::out_of_range);
    CHECK(meter.changeParameters(loudness::NumChannels(channels1), loudness::Samplerate(samplerate1)));
    meter.addFrames(sine_wave1, 3 * samplerate1);
    CHECK_THAT(20 * std::log10(meter.truePeak(0)), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK_THAT(20 * std::log10(meter.truePeak(1)), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK_THAT(20 * std::log10(meter.truePeak()), Catch::Matchers::WithinAbs(-23.0, 0.1));
    CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(-23.0, 0.1));
}

TEST_CASE("Test setChannel", "[meter][setChannel][dual-mono]")
{
    constexpr long samplerate = 32000;
    SECTION("Dual mono")
    {
        SECTION("Allowed for non-mono meters")
        {
            loudness::Meter<loudness::Mode::EBU_I> meter{loudness::NumChannels(2), loudness::Samplerate(samplerate)};
            CHECK_NOTHROW(meter.setChannel(0, loudness::Channel::DualMono));
        }
        SECTION("Produces expected output")
        {
            constexpr double target = -23.0;
            constexpr long num_samples = samplerate;
            constexpr int channels = 1;
            loudness::Meter<loudness::Mode::EBU_I> meter{loudness::NumChannels(channels),
                                                         loudness::Samplerate(samplerate)};
            auto sine_wave = sineWave<float>(1000.0, samplerate, num_samples, channels, target);
            CHECK_NOTHROW(meter.setChannel(0, loudness::Channel::DualMono));
            meter.addFrames(sine_wave, num_samples);
            /* This would have been 3 LU lower for normal mono */
            CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, 0.1));
            CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, 0.1));
        }
    }
    SECTION("Unused channel")
    {
        constexpr double target = -23.0;
        constexpr long num_samples = samplerate;
        SECTION("Only unused channels adds no data")
        {
            constexpr int channels = 1;
            loudness::Meter<loudness::Mode::EBU_I> meter{loudness::NumChannels(channels),
                                                         loudness::Samplerate(samplerate)};
            auto sine_wave = sineWave<float>(1000.0, samplerate, num_samples, channels, target);
            // Should this be an error?
            CHECK_NOTHROW(meter.setChannel(0, loudness::Channel::Unused));
            meter.addFrames(sine_wave, num_samples);
            /* No data added, so -HUGE_VAL */
            CHECK(meter.loudnessGlobal() == -HUGE_VAL);
            CHECK(meter.loudnessMomentary() == -HUGE_VAL);
        }
        SECTION("Produces expected output")
        {
            constexpr int channels = 2;
            loudness::Meter<loudness::Mode::EBU_I> meter{loudness::NumChannels(channels),
                                                         loudness::Samplerate(samplerate)};
            auto sine_wave = sineWave<float>(1000.0, samplerate, num_samples, channels, target);
            CHECK_NOTHROW(meter.setChannel(0, loudness::Channel::Unused));
            meter.addFrames(sine_wave, num_samples);
            /* 3 LU lower since it is treated as mono */
            CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target - 3.0, 0.1));
            CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target - 3.0, 0.1));
        }
    }
}

TEST_CASE("Resetting the meter", "[meter][reset]")
{
    constexpr long samplerate = 48000;
    constexpr int channels = 2;
    constexpr long num_samples = 3 * samplerate;
    loudness::Meter<loudness::Mode::TruePeak | loudness::Mode::EBU_I | loudness::Mode::EBU_S | loudness::Mode::EBU_LRA>
        meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};
    const auto audio1 = sineWave<float>(1000.0, samplerate, num_samples, channels, -26.0);
    const auto audio2 = sineWave<float>(1000.0, samplerate, num_samples, channels, -22.0);
    meter.addFrames(audio1, samplerate);
    meter.addFrames(audio2, num_samples);
    CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(-24.0, 2.0));
    CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(-22.0, 0.1));
    CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(-22.0, 0.1));
    CHECK(meter.loudnessRange() > 0.0);
    CHECK(meter.truePeak() > 0.0);
    CHECK(meter.samplePeak() > 0.0);
    meter.reset();
    CHECK(meter.loudnessGlobal() == -HUGE_VAL);
    CHECK(meter.loudnessMomentary() == -HUGE_VAL);
    CHECK(meter.loudnessShortterm() == -HUGE_VAL);
    CHECK(meter.loudnessRange() == 0.0);
    CHECK(meter.truePeak() == 0.0);
    CHECK(meter.samplePeak() == 0.0);
}

TEMPLATE_LIST_TEST_CASE("At sample rate >= 192000 true peak == sample peak", "[meter][true-peak][sample-peak]",
                        DataTypes)
{
    constexpr long samplerate = 192000;
    constexpr int channels = 2;
    loudness::Meter<loudness::Mode::TruePeak> meter{loudness::NumChannels(channels), loudness::Samplerate(samplerate)};

    constexpr double target = 0.5;
    auto sine_wave = sineWaveTP<TestType>(samplerate / 4.0, samplerate, channels, 0.0, target);
    meter.addFrames(sine_wave);

    CHECK_THAT(meter.samplePeak(0), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK_THAT(meter.samplePeak(1), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK_THAT(meter.samplePeak(), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK(meter.lastSamplePeak(0) == meter.samplePeak(0));
    CHECK(meter.lastSamplePeak(1) == meter.samplePeak(1));
    CHECK(meter.lastSamplePeak() == meter.samplePeak());
    CHECK(meter.truePeak(0) == meter.samplePeak(0));
    CHECK(meter.truePeak(1) == meter.samplePeak(1));
    CHECK(meter.truePeak() == meter.samplePeak());
    CHECK(meter.lastTruePeak(0) == meter.samplePeak(0));
    CHECK(meter.lastTruePeak(1) == meter.samplePeak(1));
    CHECK(meter.lastTruePeak() == meter.samplePeak());

    constexpr double new_target = 0.25;
    sine_wave = sineWaveTP<TestType>(samplerate / 4.0, samplerate, channels, 0.0, new_target);
    meter.addFrames(sine_wave);

    CHECK_THAT(meter.samplePeak(0), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK_THAT(meter.samplePeak(1), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK_THAT(meter.samplePeak(), Catch::Matchers::WithinAbs(target, 1e-9));
    CHECK_THAT(meter.lastSamplePeak(0), Catch::Matchers::WithinAbs(new_target, 1e-9));
    CHECK_THAT(meter.lastSamplePeak(1), Catch::Matchers::WithinAbs(new_target, 1e-9));
    CHECK_THAT(meter.lastSamplePeak(), Catch::Matchers::WithinAbs(new_target, 1e-9));
    CHECK(meter.truePeak(0) == meter.samplePeak(0));
    CHECK(meter.truePeak(1) == meter.samplePeak(1));
    CHECK(meter.truePeak() == meter.samplePeak());
    CHECK(meter.lastTruePeak(0) == meter.lastSamplePeak(0));
    CHECK(meter.lastTruePeak(1) == meter.lastSamplePeak(1));
    CHECK(meter.lastTruePeak() == meter.lastSamplePeak());
}

template <class Meter>
void checkLoudness(const Meter& meter, double target, double margin)
{
    CHECK_THAT(meter.relativeThreshold(), Catch::Matchers::WithinAbs(target - 10.0, margin));
    CHECK_THAT(meter.loudnessGlobal(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessGlobalUngated(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessShortterm(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessMomentary(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessGlobalMedian(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessGlobalMedianAfterGate(), Catch::Matchers::WithinAbs(target, margin));
    CHECK_THAT(meter.loudnessGlobalMedianUngated(), Catch::Matchers::WithinAbs(target, margin));
}

template <class Meter>
void checkTruePeak(const Meter& meter, double target)
{
    CHECK_THAT(20 * std::log10(meter.truePeak()), AsymetricMarginMatcher(target, 0.3, 0.2));
    CHECK_THAT(20 * std::log10(meter.samplePeak() * std::numbers::sqrt2), AsymetricMarginMatcher(target, 0.3, 0.2));
}

TEMPLATE_LIST_TEST_CASE("Test all input data configurations", "[meter][input]", DataTypes)
{
    constexpr auto samplerate = loudness::Samplerate(32000);
    constexpr auto channels = loudness::NumChannels(2);

    SECTION("Loudness measurments")
    {
        constexpr auto num_samples = 3 * samplerate.get();
        constexpr double target = -23.0;
        constexpr double margin = 0.1;
        loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S> meter{channels, samplerate};
        SECTION("Test interleaved")
        {
            const auto interleaved = sineWave<TestType>(1000.0, samplerate.get(), num_samples, channels.get(), target);

            meter.addFrames(interleaved.data(), num_samples);
            checkLoudness(meter, target, margin);

            meter.addFrames(interleaved, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFrames(interleaved);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(interleaved.data(), num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(interleaved, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(interleaved);
            checkLoudness(meter, target, margin);
        }
        SECTION("Test channel based")
        {
            const auto channel_audio =
                sineWaveChannels<TestType>(1000.0, samplerate.get(), num_samples, channels.get(), target);
            std::vector<const TestType*> ptr_vector;
            std::list<const TestType*> ptr_list;
            for (const auto& chan : channel_audio) {
                ptr_vector.push_back(chan.data());
                ptr_list.push_back(chan.data());
            }

            meter.addFrames(channel_audio, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFrames(channel_audio);
            checkLoudness(meter, target, margin);

            meter.addFrames(ptr_vector, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFrames(ptr_list, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFrames(ptr_vector.data(), num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(channel_audio, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(channel_audio);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(ptr_vector, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(ptr_list, num_samples);
            checkLoudness(meter, target, margin);

            meter.addFramesMT(ptr_vector.data(), num_samples);
            checkLoudness(meter, target, margin);
        }
        SECTION("Test single channel")
        {
            constexpr auto mono_target = target - 3.0;
            const auto audio = sineWaveChannels<TestType>(1000.0, samplerate.get(), num_samples, 1, target);
            loudness::Meter<loudness::Mode::EBU_I | loudness::Mode::EBU_S> meter{loudness::NumChannels(1), samplerate};

            meter.addFrames(audio, num_samples);
            checkLoudness(meter, mono_target, margin);

            meter.addFrames(audio);
            checkLoudness(meter, mono_target, margin);

            meter.addFrames(audio[0], num_samples);
            checkLoudness(meter, mono_target, margin);

            meter.addFrames(audio[0]);
            checkLoudness(meter, mono_target, margin);

            meter.addFrames(audio[0].data(), num_samples);
            checkLoudness(meter, mono_target, margin);

            meter.addFramesMT(audio, num_samples);
            checkLoudness(meter, mono_target, margin);

            meter.addFramesMT(audio);
            checkLoudness(meter, mono_target, margin);

            meter.addFramesMT(audio[0], num_samples);
            checkLoudness(meter, mono_target, margin);

            meter.addFramesMT(audio[0]);
            checkLoudness(meter, mono_target, margin);

            meter.addFramesMT(audio[0].data(), num_samples);
            checkLoudness(meter, mono_target, margin);
        }
    }
    SECTION("Peak measurements")
    {
        constexpr double target = -6.0;
        SECTION("Test interleaved")
        {
            loudness::Meter<loudness::Mode::TruePeak> meter{channels, samplerate};
            const auto sine_wave =
                sineWaveTP<TestType>(samplerate.get() / 4.0, samplerate.get(), channels.get(), 45.0, 0.5);
            const auto num_samples = sine_wave.size() / channels.get();

            meter.addFrames(sine_wave, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFrames(sine_wave.data(), num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(sine_wave, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(sine_wave.data(), num_samples);
            checkTruePeak(meter, target);
        }
        SECTION("Test deinterleaved")
        {
            loudness::Meter<loudness::Mode::TruePeak> meter{channels, samplerate};
            const auto interleaved =
                sineWaveTP<TestType>(samplerate.get() / 4.0, samplerate.get(), channels.get(), 45.0, 0.5);
            const auto num_samples = interleaved.size() / channels.get();
            const auto audio = deinterleave(interleaved, channels.get());
            std::vector<const TestType*> ptr_vector;
            std::list<const TestType*> ptr_list;
            for (const auto& chan : audio) {
                ptr_vector.push_back(chan.data());
                ptr_list.push_back(chan.data());
            }

            meter.addFrames(audio, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFrames(ptr_vector, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFrames(ptr_list, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFrames(ptr_vector.data(), num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(audio, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(ptr_vector, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(ptr_list, num_samples);
            checkTruePeak(meter, target);

            meter.reset();
            meter.addFramesMT(ptr_vector.data(), num_samples);
            checkTruePeak(meter, target);
        }
        SECTION("Test single channel")
        {
            loudness::Meter<loudness::Mode::TruePeak> meter{loudness::NumChannels(1), samplerate};
            const auto sine_wave = sineWaveTP<TestType>(samplerate.get() / 4.0, samplerate.get(), 1, 45.0, 0.5);
            const auto [min, max] = std::minmax_element(sine_wave.begin(), sine_wave.end());
            const auto sample_peak = std::max<TestType>(-*min, *max) / loudness::getScalingFactor<TestType>();
            const auto num_samples = sine_wave.size();

            meter.addFrames(sine_wave, num_samples);
            CHECK_THAT(20 * std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
            CHECK(meter.samplePeak(0) == sample_peak);

            meter.reset();
            meter.addFrames(sine_wave.data(), num_samples);
            CHECK_THAT(20 * std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
            CHECK(meter.samplePeak(0) == sample_peak);

            meter.reset();
            meter.addFramesMT(sine_wave, num_samples);
            CHECK_THAT(20 * std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
            CHECK(meter.samplePeak(0) == sample_peak);

            meter.reset();
            meter.addFramesMT(sine_wave.data(), num_samples);
            CHECK_THAT(20 * std::log10(meter.truePeak(0)), AsymetricMarginMatcher(target, 0.3, 0.2));
            CHECK(meter.samplePeak(0) == sample_peak);
        }
    }
}

TEMPLATE_TEST_CASE_SIG("Test multi-global-loudness", "[integrated][multi]",
                       ((typename T, loudness::Mode mode), T, mode), (float, loudness::Mode::EBU_I),
                       (double, loudness::Mode::EBU_I), (std::int16_t, loudness::Mode::EBU_I),
                       (std::int32_t, loudness::Mode::EBU_I), (float, loudness::Mode::EBU_I | loudness::Mode::Histogram),
                       (double, loudness::Mode::EBU_I | loudness::Mode::Histogram),
                       (std::int16_t, loudness::Mode::EBU_I | loudness::Mode::Histogram),
                       (std::int32_t, loudness::Mode::EBU_I | loudness::Mode::Histogram))
{
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = -23.0;
    auto sine_wave1 = sineWave<T>(1000, samplerate, 20 * samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<T>(1000, samplerate, 60 * samplerate, channels, -23.0);

    std::vector<loudness::Meter<mode>> meters;
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));

    meters[0].addFrames(sine_wave1.data(), 20 * samplerate);
    meters[1].addFrames(sine_wave2.data(), 60 * samplerate);

    CHECK_THAT(loudnessGlobalMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}

TEMPLATE_TEST_CASE_SIG("Test multi-loudness-range", "[loudness-range][multi]",
                       ((typename T, loudness::Mode mode), T, mode), (float, loudness::Mode::EBU_LRA),
                       (double, loudness::Mode::EBU_LRA), (std::int16_t, loudness::Mode::EBU_LRA),
                       (std::int32_t, loudness::Mode::EBU_LRA), (float, loudness::Mode::EBU_LRA | loudness::Mode::Histogram),
                       (double, loudness::Mode::EBU_LRA | loudness::Mode::Histogram),
                       (std::int16_t, loudness::Mode::EBU_LRA | loudness::Mode::Histogram),
                       (std::int32_t, loudness::Mode::EBU_LRA | loudness::Mode::Histogram))
{
    const unsigned long samplerate = GENERATE(44100, 48000, prime_samplerate);
    constexpr unsigned int channels = 2;
    constexpr double target = 13.0;
    auto sine_wave1 = sineWave<T>(1000, samplerate, 20 * samplerate, channels, -36.0);
    auto sine_wave2 = sineWave<T>(1000, samplerate, 60 * samplerate, channels, -23.0);

    std::vector<loudness::Meter<mode>> meters;
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));
    meters.emplace_back(loudness::NumChannels(channels), loudness::Samplerate(samplerate));

    meters[0].addFrames(sine_wave1.data(), 20 * samplerate);
    meters[1].addFrames(sine_wave2.data(), 60 * samplerate);

    CHECK_THAT(loudnessRangeMultiple(meters), Catch::Matchers::WithinAbs(target, 0.1));
}
