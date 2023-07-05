/* See COPYING file for copyright and license details. */

#ifndef EBUR128_IMPL_HPP_
#define EBUR128_IMPL_HPP_


#include <cstddef> /* for size_t */
#include <memory>
#include <vector>
#include "defines.hpp"

namespace detail {

    class Ebur128 {
    public:
        static constexpr unsigned int MAX_CHANNELS = 64;
        static constexpr unsigned long MIN_SAMPLERATE = 16;
        static constexpr unsigned long MAX_SAMPLERATE = 2822400;

        Ebur128(unsigned int channels, unsigned long samplerate, unsigned int mode);
        ~Ebur128();
        Ebur128(const Ebur128&) = delete;
        Ebur128& operator=(const Ebur128&) = delete;
        Ebur128(Ebur128&& other);
        Ebur128& operator=(Ebur128&& other) = delete;

        void setChannel(unsigned int channel_number, Channel value);
        bool changeParameters(unsigned int channels, unsigned long samplerate);
        bool setMaxWindow(unsigned long window);
        bool setMaxHistory(unsigned long history);

        template <typename T>
        void addFrames(const T* src, size_t frames);

        template <typename T>
        void addFrames(const T** src, size_t frames);

        // EBU-R128
        double loudnessGlobal();
        double loudnessMomentary();
        double loudnessShortterm();
        double loudnessWindow(unsigned long window);

        double loudnessRange();

        double loudnessGlobalMedian();

        double samplePeak(unsigned int channel_number) const;
        double lastSamplePeak(unsigned int channel_number) const;
        double truePeak(unsigned int channel_number) const;
        double lastTruePeak(unsigned int channel_number) const;

        double relativeThreshold();

        static double loudnessGlobalMultiple(const std::vector<const Ebur128*>& meters);
        static double loudnessRangeMultipleHist(const std::vector<const Ebur128*>& meters);
        static double loudnessRangeMultipleBlocks(const std::vector<const Ebur128*>& meters);

    private:
        template <ConstData T>
        void addFramesLoudness(T* src, size_t frames);

        std::unique_ptr<struct Ebur128Impl> pimpl; /**< Internal state. */
        const unsigned int mode;    /**< The current mode. */
        unsigned int channels;      /**< The number of channels. */
        unsigned long samplerate;   /**< The sample rate. */
    };
} // namespace detail
#endif /* EBUR128_IMPL_HPP_ */
