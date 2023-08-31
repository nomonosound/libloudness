/* See LICENSE file for copyright and license details. */

#ifndef LIBLOUDNESS_DETAIL_METER_IMPL_HPP
#define LIBLOUDNESS_DETAIL_METER_IMPL_HPP

#include <cstddef>
#include <memory>
#include <vector>

#include "loudness/defines.hpp"

namespace loudness::detail {
    class Meter {
    public:
        Meter(NumChannels channels, Samplerate samplerate, Mode mode);
        ~Meter() noexcept;
        Meter(Meter&& other) noexcept;
        Meter& operator=(Meter&& other) noexcept;
        Meter(const Meter&) = delete;
        Meter& operator=(const Meter&) = delete;

        void reset();

        std::size_t channels() const { return channels_; }
        void setChannel(unsigned int channel_index, Channel value);
        bool changeParameters(NumChannels channels, Samplerate samplerate);
        bool setMaxWindow(unsigned long window_ms);
        bool setMaxHistory(unsigned long history_ms);

        void addFrames(DataType src, std::size_t frames);
        void addFramesMT(DataType src, std::size_t frames);

        template <std::ranges::range Range>
        void addFrames(const Range& src, std::size_t frames);
        template <std::ranges::range Range>
        void addFramesMT(const Range& src, std::size_t frames);

        template <std::ranges::range Range>
        void addFrames(const Range& src);
        template <std::ranges::range Range>
        void addFramesMT(const Range& src);

        // Tech 3341
        [[nodiscard]] double loudnessGlobal() const;
        [[nodiscard]] double loudnessMomentary() const;
        [[nodiscard]] double loudnessShortterm() const;

        // Tech 3342
        [[nodiscard]] double loudnessRange() const;

        // Other BS.1770 based
        [[nodiscard]] double loudnessWindow(unsigned long window_ms) const;
        [[nodiscard]] double loudnessGlobalUngated() const;
        [[nodiscard]] double loudnessGlobalMedian() const;
        [[nodiscard]] double loudnessGlobalMedianAfterGate() const;
        [[nodiscard]] double loudnessGlobalMedianUngated() const;

        [[nodiscard]] double samplePeak(unsigned int channel_index) const;
        [[nodiscard]] double lastSamplePeak(unsigned int channel_index) const;
        [[nodiscard]] double truePeak(unsigned int channel_index) const;
        [[nodiscard]] double lastTruePeak(unsigned int channel_index) const;

        [[nodiscard]] double samplePeak() const;
        [[nodiscard]] double lastSamplePeak() const;
        [[nodiscard]] double truePeak() const;
        [[nodiscard]] double lastTruePeak() const;

        [[nodiscard]] double relativeThreshold() const;

        [[nodiscard]] static double loudnessGlobalMultiple(const std::vector<const Meter*>& meters);
        [[nodiscard]] static double loudnessRangeMultipleHist(const std::vector<const Meter*>& meters);
        [[nodiscard]] static double loudnessRangeMultipleBlocks(const std::vector<const Meter*>& meters);

    private:
        // TODO: If header only library, mode can be const
        Mode mode_;                          /**< The mode of the meter */
        unsigned int channels_;              /**< The number of channels. */
        unsigned long samplerate_;           /**< The sample rate. */
        std::unique_ptr<struct Impl> pimpl_; /**< Internal state. */
    };

    /* Ugly template code to convert ranges into pointers */
    template <typename T>
    concept HasDataPointer = requires(T t) { *t.data(); };

    template <void (Meter::*Func)(DataType, std::size_t), std::ranges::range Range>
    void addRange(Meter* obj, const Range& src, std::size_t frames)
    {
        using T = std::ranges::range_value_t<Range>;
        if constexpr (std::is_pointer_v<T>) {
            using U = std::add_pointer_t<std::add_const_t<std::remove_pointer_t<T>>>;
            if constexpr (HasDataPointer<Range> && std::is_const_v<std::remove_pointer_t<T>>) {
                (obj->*Func)(src.data(), frames);
            }
            else {
                std::vector<U> data;
                for (auto* ptr : src) {
                    data.push_back(ptr);
                }
                (obj->*Func)(data.data(), frames);
            }
        }
        else if constexpr (HasDataPointer<T>) {
            using U = std::add_pointer_t<std::add_const_t<std::remove_pointer_t<decltype(src.begin()->data())>>>;
            std::vector<U> data;
            for (auto& container : src) {
                data.push_back(container.data());
            }
            (obj->*Func)(data.data(), frames);
        }
        else {
            // Add it as interleaved/single channel if possible
            (obj->*Func)(src.data(), frames);
        }
    }

    template <void (Meter::*Func)(DataType, std::size_t), std::ranges::range Range>
    void addRange(Meter* obj, const Range& src)
    {
        using T = std::ranges::range_value_t<Range>;
        if constexpr (std::ranges::sized_range<T> && std::ranges::contiguous_range<T>){
            using U = std::add_pointer_t<std::add_const_t<std::remove_pointer_t<decltype(src.begin()->data())>>>;
            std::vector<U> data;
            const std::size_t frames = src.begin()->size();
            for (auto& container : src) {
                data.push_back(container.data());
            }
            (obj->*Func)(data.data(), frames);
        } else {
            (obj->*Func)(src.data(), src.size() / obj->channels());
        }
    }

    template <std::ranges::range Range>
    void Meter::addFrames(const Range& src, std::size_t frames)
    {
        addRange<&Meter::addFrames, Range>(this, src, frames);
    }

    template <std::ranges::range Range>
    void Meter::addFramesMT(const Range& src, std::size_t frames)
    {
        addRange<&Meter::addFramesMT, Range>(this, src, frames);
    }

    template <std::ranges::range Range>
    void Meter::addFrames(const Range& src)
    {
        addRange<&Meter::addFrames, Range>(this, src);
    }

    template <std::ranges::range Range>
    void Meter::addFramesMT(const Range& src)
    {
        addRange<&Meter::addFramesMT, Range>(this, src);
    }

}  // namespace loudness::detail
#endif  // LIBLOUDNESS_DETAIL_METER_IMPL_HPP
