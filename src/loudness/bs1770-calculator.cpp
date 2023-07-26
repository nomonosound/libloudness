/* See LICENSE file for copyright and license details. */

#include "bs1770-calculator.hpp"

#include <algorithm>
#include <cmath>
#include <gcem.hpp>
#include <numeric>

#include "constants.hpp"
#include "utils.hpp"

namespace loudness {

    constexpr double low_percent = 0.10;
    constexpr double high_percent = 0.95;

    /* HistogramCalculator */

    constexpr double bin_size_LU = 0.1;

    consteval std::array<double, HistogramCalculator::HISTOGRAM_SIZE> getHistogramEnergies()
    {
        std::array<double, HistogramCalculator::HISTOGRAM_SIZE> histogram_energies;
        for (unsigned int i = 0; i < histogram_energies.size(); ++i) {
            histogram_energies[i] =
                loudnessToEnergy(static_cast<double>(i) * bin_size_LU + absolute_gate_LUFS + bin_size_LU / 2);
        }
        return histogram_energies;
    }

    constexpr double minimum_energy = absolute_gate;

    constexpr std::size_t findHistogramIndex(double energy)
    {
        constexpr double max =
            loudnessToEnergy((HistogramCalculator::HISTOGRAM_SIZE - 1) * bin_size_LU + absolute_gate_LUFS);
        if (energy >= max) [[unlikely]] {
            return HistogramCalculator::HISTOGRAM_SIZE - 1;
        }
        else if (energy <= minimum_energy) [[unlikely]] {
            return 0;
        }
        else [[likely]] {
            return static_cast<std::size_t>((energyToLoudness(energy) - absolute_gate_LUFS) / bin_size_LU);
        }
    }

    static constexpr auto histogram_energies = getHistogramEnergies();

    void HistogramCalculator::addBlock(double energy) { ++block_energy_histogram[findHistogramIndex(energy)]; }

    void HistogramCalculator::addShortTermBlock(double energy)
    {
        ++short_term_block_energy_histogram[findHistogramIndex(energy)];
    }

    ValueCounter HistogramCalculator::blockCountAndSum() const
    {
        return {.count = std::reduce(block_energy_histogram.cbegin(), block_energy_histogram.cend(), 0UL),
                .sum = std::transform_reduce(block_energy_histogram.cbegin(), block_energy_histogram.cend(),
                                             histogram_energies.cbegin(), 0.0)};
    }

    ValueCounter HistogramCalculator::loudness(double relative_threshold) const
    {
        std::size_t above_thresh_counter = 0;
        double gated_loudness = 0.0;
        for (std::size_t j = relative_threshold_index(relative_threshold); j < HISTOGRAM_SIZE; ++j) {
            gated_loudness += block_energy_histogram[j] * histogram_energies[j];
            above_thresh_counter += block_energy_histogram[j];
        }

        return {.count = above_thresh_counter, .sum = gated_loudness};
    }

    double HistogramCalculator::medianLoudness(double relative_threshold) const
    {
        std::size_t i = relative_threshold_index(relative_threshold);
        const auto total = std::reduce(block_energy_histogram.cbegin() + i, block_energy_histogram.cend(), 0UL);
        if (total == 0) return -HUGE_VAL;

        const auto n = total / 2;
        std::size_t count = 0;
        for (; count < n; ++i) {
            count += block_energy_histogram[i];
        }
        if (count == n && total % 2 == 0) {
            // Median is between two bars, take average of the two
            std::size_t j = i;
            for (; block_energy_histogram[j] == 0; ++j)
                ;
            return (histogram_energies[i - 1] + histogram_energies[j]) / 2;
        }
        return histogram_energies[i - 1];
    }

    double HistogramCalculator::ungatedMedianLoudness() const { return medianLoudness(minimum_energy); }

    double HistogramCalculator::loudnessRangeMultiple(const std::vector<const HistogramCalculator*>& histograms)
    {
        std::array<unsigned long, HISTOGRAM_SIZE> hist = {0};
        std::size_t num_blocks = 0;
        double power = 0.0;

        for (const auto& hist_calc : histograms) {
            for (std::size_t j = 0; j < HISTOGRAM_SIZE; ++j) {
                hist[j] += hist_calc->short_term_block_energy_histogram[j];
                num_blocks += hist_calc->short_term_block_energy_histogram[j];
                power += hist_calc->short_term_block_energy_histogram[j] * histogram_energies[j];
            }
        }
        if (num_blocks == 0) {
            return 0.0;
        }

        power /= num_blocks;
        const double limit_energy = minus_twenty_decibels * power;

        std::size_t index;
        if (limit_energy < minimum_energy) {
            index = 0;
        }
        else {
            index = findHistogramIndex(limit_energy);
            if (limit_energy > histogram_energies[index]) {
                ++index;
            }
        }
        num_blocks = std::reduce(hist.cbegin() + index, hist.cend(), 0UL);
        if (num_blocks == 0) {
            return 0.0;
        }

        const std::size_t percentile_low = std::lround((num_blocks - 1) * low_percent);
        const std::size_t percentile_high = std::lround((num_blocks - 1) * high_percent);

        num_blocks = 0;
        std::size_t j = index;
        while (num_blocks <= percentile_low) {
            num_blocks += hist[j++];
        }
        const double low_en = histogram_energies[j - 1];
        while (num_blocks <= percentile_high) {
            num_blocks += hist[j++];
        }
        const double high_en = histogram_energies[j - 1];

        return energyToLoudness(high_en) - energyToLoudness(low_en);
    }

    std::size_t HistogramCalculator::relative_threshold_index(double relative_threshold)
    {
        if (relative_threshold < minimum_energy) {
            return 0;
        }

        std::size_t start_index = findHistogramIndex(relative_threshold);
        if (relative_threshold > histogram_energies[start_index]) {
            ++start_index;
        }
        return start_index;
    }

    /* BlockListCalculator */

    BlockListCalculator::BlockListCalculator()
        : history_ms_(std::numeric_limits<unsigned long>::max()),
          block_list_max_(history_ms_ / block_overlap_ms),
          st_block_list_max_(history_ms_ / block_overlap_ms)
    {
    }

    void BlockListCalculator::addBlock(double energy)
    {
        if (block_list_.size() == block_list_max_) {
            block_list_.pop_front();
        }
        block_list_.push_back(energy);
    }

    void BlockListCalculator::addShortTermBlock(double energy)
    {
        if (short_term_block_list_.size() == st_block_list_max_) {
            short_term_block_list_.pop_front();
        }
        short_term_block_list_.push_back(energy);
    }

    bool BlockListCalculator::setMaxHistory(unsigned long history_ms) noexcept
    {
        if (history_ms == history_ms_) return false;
        history_ms_ = history_ms;
        block_list_max_ = history_ms_ / block_overlap_ms;
        if (block_list_max_ >= m_subblocks) {
            block_list_max_ -= m_subblocks - 1;
        }
        if (block_list_.size() > block_list_max_) {
            block_list_.resize(block_list_max_);
        }
        st_block_list_max_ = history_ms_ / block_overlap_ms;
        if (st_block_list_max_ >= st_subblocks) {
            st_block_list_max_ -= st_subblocks - 1;
        }
        if (short_term_block_list_.size() > st_block_list_max_) {
            short_term_block_list_.resize(st_block_list_max_);
        }
        return true;
    }
    ValueCounter BlockListCalculator::blockCountAndSum() const
    {
        return {.count = block_list_.size(), .sum = std::reduce(block_list_.cbegin(), block_list_.cend(), 0.0)};
    }

    ValueCounter BlockListCalculator::loudness(double relative_threshold) const
    {
        double gated_loudness = 0.0;
        std::size_t above_thresh_counter = 0;
        for (auto block : block_list_) {
            if (block >= relative_threshold) {
                ++above_thresh_counter;
                gated_loudness += block;
            }
        }
        return {.count = above_thresh_counter, .sum = gated_loudness};
    }

    double BlockListCalculator::medianLoudness(double relative_threshold) const
    {
        return median(
            std::views::filter(block_list_, [relative_threshold](auto val) { return val >= relative_threshold; }));
    }

    double BlockListCalculator::ungatedMedianLoudness() const
    {
        // List doesn't need to be chronological, so might as well take the median in place.
        return median(block_list_);
    }

    double BlockListCalculator::loudnessRangeMultiple(const std::vector<const BlockListCalculator*>& block_lists)
    {
        std::size_t num_blocks = 0;
        for (const auto& block_list : block_lists) {
            num_blocks += block_list->short_term_block_list_.size();
        }
        if (num_blocks == 0) {
            return 0.0;
        }

        std::vector<double> blocks;
        blocks.reserve(num_blocks);

        for (const auto& block_list : block_lists) {
            blocks.insert(blocks.end(), block_list->short_term_block_list_.begin(),
                          block_list->short_term_block_list_.end());
        }

        std::sort(blocks.begin(), blocks.end());

        const double power = std::reduce(blocks.begin(), blocks.end(), 0.0) / static_cast<double>(num_blocks);
        const double limit_energy = minus_twenty_decibels * power;

        auto range_begin = std::lower_bound(blocks.begin(), blocks.end(), limit_energy);

        if (range_begin == blocks.end()) {
            return 0.0;
        }
        const auto range_size = std::distance(range_begin, blocks.end() - 1);
        const double high_en = range_begin[std::lround(range_size * high_percent)];
        const double low_en = range_begin[std::lround(range_size * low_percent)];

        return energyToLoudness(high_en) - energyToLoudness(low_en);
    }
}  // namespace loudness
