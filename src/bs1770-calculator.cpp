#include "bs1770-calculator.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <gcem.hpp>

#include "constants.hpp"
#include "utils.hpp"

consteval std::array<double, HistogramCalculator::HISTOGRAM_SIZE> getHistogramEnergies()
{
    std::array<double, HistogramCalculator::HISTOGRAM_SIZE> histogram_energies;
    for (unsigned int i = 0; i < histogram_energies.size(); ++i) {
        histogram_energies[i] = loudnessToEnergy(static_cast<double>(i) / 10.0 - 69.95);
    }
    return histogram_energies;
}

constexpr double minimum_energy = absolute_gate;

constexpr size_t findHistogramIndex(double energy)
{
    constexpr double max = loudnessToEnergy((HistogramCalculator::HISTOGRAM_SIZE - 1) / 10.0 - 70.0);
    if (energy >= max) [[unlikely]]{
        return HistogramCalculator::HISTOGRAM_SIZE - 1;
    } else if (energy <= minimum_energy) [[unlikely]]{
        return 0;
    } else [[likely]]{
        return static_cast<size_t>(10.0 * (10.0 * std::log10(energy) + 70.0 - 0.691));
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
        .counter = std::accumulate(block_energy_histogram.cbegin(), block_energy_histogram.cend(), 0UL),
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

    size_t index = relative_threshold_index(stl_integrated);
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

    const auto percentile_low = static_cast<size_t>((stl_size - 1) * 0.1 + 0.5);
    const auto percentile_high = static_cast<size_t>((stl_size - 1) * 0.95 + 0.5);

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

    return EnergyToLoudness(h_en) - EnergyToLoudness(l_en);
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

bool BlockListCalculator::setMaxHistory(unsigned long history)
{
    if (history == history_) return false;
    history_ = history;
    block_list_max_ = history / 100;
    st_block_list_max_ = history / 3000;
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
    return {.counter = block_list_.size(), .sum = std::accumulate(block_list_.cbegin(), block_list_.cend(), 0.0)};
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

    const double stl_power = std::accumulate(stl_vector.begin(), stl_vector.end(), 0.0) / static_cast<double>(stl_size);
    const double stl_integrated = minus_twenty_decibels * stl_power;

    double* stl_relgated = stl_vector.data();
    size_t stl_relgated_size = stl_size;
    while (stl_relgated_size > 0 && *stl_relgated < stl_integrated) {
        ++stl_relgated;
        --stl_relgated_size;
    }

    if (stl_relgated_size) {
        const double h_en = stl_relgated[static_cast<size_t>((stl_relgated_size - 1) * 0.95 + 0.5)];
        const double l_en = stl_relgated[static_cast<size_t>((stl_relgated_size - 1) * 0.1 + 0.5)];
        return EnergyToLoudness(h_en) - EnergyToLoudness(l_en);
    }

    return 0.0;
}
