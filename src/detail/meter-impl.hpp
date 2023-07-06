/* See COPYING file for copyright and license details. */

#ifndef METER_IMPL_HPP
#define METER_IMPL_HPP


#include "defines.hpp"
#include <cstddef>
#include <memory>
#include <vector>

namespace loudness::detail {

    class Meter {
    public:
        static constexpr unsigned int MAX_CHANNELS = 64;
        static constexpr unsigned long MIN_SAMPLERATE = 16;
        static constexpr unsigned long MAX_SAMPLERATE = 2822400;

        Meter(unsigned int channels, unsigned long samplerate, unsigned int mode);
        ~Meter();
        Meter(const Meter&) = delete;
        Meter& operator=(const Meter&) = delete;
        Meter(Meter&& other) noexcept;
        Meter& operator=(Meter&& other) = delete;

        void setChannel(unsigned int channel_index, Channel value);
        bool changeParameters(unsigned int channels, unsigned long samplerate);
        bool setMaxWindow(unsigned long window_ms);
        bool setMaxHistory(unsigned long history_ms);

        void addFrames(DataType src, size_t frames);

        void addFramesSeq(DataType src, size_t frames);

        // EBU-R128
        [[nodiscard]] double loudnessGlobal() const;
        [[nodiscard]] double loudnessMomentary() const;
        [[nodiscard]] double loudnessShortterm() const;
        [[nodiscard]] double loudnessWindow(unsigned long window_ms) const;

        [[nodiscard]] double loudnessRange() const;

        [[nodiscard]] double loudnessGlobalMedian() const;

        [[nodiscard]] double samplePeak(unsigned int channel_index) const;
        [[nodiscard]] double lastSamplePeak(unsigned int channel_index) const;
        [[nodiscard]] double truePeak(unsigned int channel_index) const;
        [[nodiscard]] double lastTruePeak(unsigned int channel_index) const;

        [[nodiscard]] double relativeThreshold() const;

        [[nodiscard]] static double loudnessGlobalMultiple(const std::vector<const Meter*>& meters);
        [[nodiscard]] static double loudnessRangeMultipleHist(const std::vector<const Meter*>& meters);
        [[nodiscard]] static double loudnessRangeMultipleBlocks(const std::vector<const Meter*>& meters);

    private:
        std::unique_ptr<struct Impl> pimpl; /**< Internal state. */
        const unsigned int mode;    /**< The current mode. */
        unsigned int channels;      /**< The number of channels. */
        unsigned long samplerate;   /**< The sample rate. */
    };
} // namespace loudness::detail
#endif /* METER_IMPL_HPP */
