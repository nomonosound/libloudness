#include "bs1770-calculator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <gcem.hpp>

#include "constants.hpp"
#include "utils.hpp"

namespace loudness {

    /* HistogramCalculator */

    consteval std::array<double, HistogramCalculator::HISTOGRAM_SIZE> getHistogramEnergies()
    {
        std::array<double, HistogramCalculator::HISTOGRAM_SIZE> histogram_energies;
        for (unsigned int i = 0; i < histogram_energies.size(); ++i) {
            histogram_energies[i] = loudnessToEnergy(static_cast<double>(i) / 10.0 + abs_threshold + 0.05);
        }
        return histogram_energies;
    }

    constexpr double minimum_energy = absolute_gate;

    constexpr size_t findHistogramIndex(double energy)
    {
        constexpr double max = loudnessToEnergy((HistogramCalculator::HISTOGRAM_SIZE - 1) / 10.0 + abs_threshold);
        if (energy >= max) [[unlikely]]{
            return HistogramCalculator::HISTOGRAM_SIZE - 1;
        } else if (energy <= minimum_energy) [[unlikely]]{
            return 0;
        } else [[likely]]{
            return static_cast<size_t>(10.0 * (energyToLoudness(energy) - abs_threshold));
        }
    }

    static constexpr auto histogram_energies = getHistogramEnergies();

    void HistogramCalculator::addBlock(double energy) { ++block_energy_histogram[findHistogramIndex(energy)]; }

    void HistogramCalculator::addShortTermBlock(double energy)
    {
        ++short_term_block_energy_histogram[findHistogramIndex(energy)];
    }

    ValueCounter HistogramCalculator::relativeThreshold() const
    {
        return {
            .counter = std::reduce(block_energy_histogram.cbegin(), block_energy_histogram.cend(), 0UL),
            .sum = std::transform_reduce(block_energy_histogram.cbegin(), block_energy_histogram.cend(), histogram_energies.cbegin(), 0.0)
        };
    }

    ValueCounter HistogramCalculator::gatedLoudness(double relative_threshold) const
    {
        size_t above_thresh_counter = 0;
        double gated_loudness = 0.0;
        for (size_t j = relative_threshold_index(relative_threshold); j < HISTOGRAM_SIZE; ++j) {
            gated_loudness += block_energy_histogram[j] * histogram_energies[j];
            above_thresh_counter += block_energy_histogram[j];
        }

        return {.counter=above_thresh_counter, .sum=gated_loudness};
    }

    double HistogramCalculator::gatedMedianLoudness(double relative_threshold) const
    {
        size_t i = relative_threshold_index(relative_threshold);
        const auto total = std::reduce(block_energy_histogram.cbegin() + i, block_energy_histogram.cend(), 0UL);
        if (total == 0) return -HUGE_VAL;

        const auto n = total / 2;
        size_t count = 0;
        for (; count < n; ++i){
            count += block_energy_histogram[i];
        }
        if (count == n && total % 2 == 0) {
            // Median is between two bars, take average of the two
            size_t j = i;
            for (;block_energy_histogram[j] == 0; ++j);
            return (histogram_energies[i - 1] + histogram_energies[j]) / 2;
        }
        return histogram_energies[i - 1];
    }


    double HistogramCalculator::loudnessRangeMultiple(const std::vector<const HistogramCalculator*>& histograms)
    {
        std::array<unsigned long, HISTOGRAM_SIZE> hist = {0};
        size_t stl_size = 0;
        double stl_power = 0.0;

        for (const auto& hist_calc : histograms) {
            for (size_t j = 0; j < HISTOGRAM_SIZE; ++j) {
                hist[j] += hist_calc->short_term_block_energy_histogram[j];
                stl_size += hist_calc->short_term_block_energy_histogram[j];
                stl_power += hist_calc->short_term_block_energy_histogram[j] * histogram_energies[j];
            }
        }
        if (stl_size == 0) {
            return 0.0;
        }

        stl_power /= stl_size;
        const double stl_integrated = minus_twenty_decibels * stl_power;

        size_t index;
        if (stl_integrated < minimum_energy) {
            index = 0;
        }
        else {
            index = findHistogramIndex(stl_integrated);
            if (stl_integrated > histogram_energies[index]) {
                ++index;
            }
        }
        stl_size = 0;
        for (size_t j = index; j < HISTOGRAM_SIZE; ++j) {
            stl_size += hist[j];
        }
        if (stl_size == 0) {
            return 0.0;
        }

        const auto percentile_low = std::lround((stl_size - 1) * 0.10);
        const auto percentile_high = std::lround((stl_size - 1) * 0.95);

        stl_size = 0;
        size_t j = index;
        while (stl_size <= percentile_low) {
            stl_size += hist[j++];
        }
        const double l_en = histogram_energies[j - 1];
        while (stl_size <= percentile_high) {
            stl_size += hist[j++];
        }
        const double h_en = histogram_energies[j - 1];

        return energyToLoudness(h_en) - energyToLoudness(l_en);
    }

    size_t HistogramCalculator::relative_threshold_index(double relative_threshold)
    {
        if (relative_threshold < minimum_energy) {
            return 0;
        }

        size_t start_index = findHistogramIndex(relative_threshold);
        if (relative_threshold > histogram_energies[start_index]) {
            ++start_index;
        }
        return start_index;
    }

    /* BlockListCalculator */

    BlockListCalculator::BlockListCalculator()
        : history_ms_(std::numeric_limits<unsigned long>::max()),
          block_list_max_(history_ms_ / m_block_overlap_ms),
          st_block_list_max_(history_ms_ / st_block_overlap_ms)
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
        block_list_max_ = history_ms / m_block_overlap_ms;
        st_block_list_max_ = history_ms / st_block_overlap_ms;
        if (block_list_.size() > block_list_max_) {
            block_list_.resize(block_list_max_);
        }
        if (short_term_block_list_.size() > st_block_list_max_) {
            short_term_block_list_.resize(st_block_list_max_);
        }
        return true;
    }
    ValueCounter BlockListCalculator::relativeThreshold() const
    {
        return {.counter = block_list_.size(), .sum = std::reduce(block_list_.cbegin(), block_list_.cend(), 0.0)};
    }

    ValueCounter BlockListCalculator::gatedLoudness(double relative_threshold) const
    {
        double gated_loudness = 0.0;
        size_t above_thresh_counter = 0;
        for (auto block : block_list_) {
            if (block >= relative_threshold) {
                ++above_thresh_counter;
                gated_loudness += block;
            }
        }
        return {.counter=above_thresh_counter, .sum=gated_loudness};
    }

    double BlockListCalculator::gatedMedianLoudness(double relative_threshold) const
    {
        return median(std::views::filter(block_list_, [relative_threshold](auto val){return val >= relative_threshold;}));
    }

    double BlockListCalculator::loudnessRangeMultiple(const std::vector<const BlockListCalculator*>& block_lists)
    {
        size_t stl_size = 0;
        for (const auto& block_list : block_lists) {
            stl_size += block_list->short_term_block_list_.size();
        }
        if (stl_size == 0) {
            return 0.0;
        }

        std::vector<double> stl_vector;
        stl_vector.reserve(stl_size);

        for (const auto& block_list : block_lists) {
            stl_vector.insert(stl_vector.end(), block_list->short_term_block_list_.begin(),
                              block_list->short_term_block_list_.end());
        }

        std::sort(stl_vector.begin(), stl_vector.end());

        const double stl_power = std::reduce(stl_vector.begin(), stl_vector.end(), 0.0) / static_cast<double>(stl_size);
        const double stl_integrated = minus_twenty_decibels * stl_power;

        double* stl_relgated = stl_vector.data();
        size_t stl_relgated_size = stl_size;
        while (stl_relgated_size > 0 && *stl_relgated < stl_integrated) {
            ++stl_relgated;
            --stl_relgated_size;
        }

        if (stl_relgated_size != 0) {
            const double h_en = stl_relgated[std::lround((stl_relgated_size - 1) * 0.95)];
            const double l_en = stl_relgated[std::lround((stl_relgated_size - 1) * 0.10)];
            return energyToLoudness(h_en) - energyToLoudness(l_en);
        }

        return 0.0;
    }
} // namespace loudness
