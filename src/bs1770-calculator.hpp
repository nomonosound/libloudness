#ifndef BS1770CALCULATOR_HPP
#define BS1770CALCULATOR_HPP

#include <array>
#include <cstddef>
#include <deque>
#include <limits>
#include <vector>

struct ValueCounter {
    size_t counter;
    double sum;
};

class BS1770Calculator {
public:
    virtual ~BS1770Calculator() = default;
    virtual void addBlock(double energy) = 0;
    virtual void addShortTermBlock(double energy) = 0;
    virtual bool setMaxHistory(unsigned long /*history_ms*/) { return false; };

    virtual ValueCounter relativeThreshold() const = 0;
    virtual ValueCounter gatedLoudness(double relative_threshold) const = 0;
    virtual double gatedMedianLoudness(double relative_threshold) const = 0;
};

class BlockListCalculator : public BS1770Calculator {
public:
    void addBlock(double energy) override;
    void addShortTermBlock(double energy) override;
    bool setMaxHistory(unsigned long history_ms) override;
    ValueCounter relativeThreshold() const override;
    ValueCounter gatedLoudness(double relative_threshold) const override;
    double gatedMedianLoudness(double relative_threshold) const override;

    static double loudnessRangeMultiple(const std::vector<const BlockListCalculator*>& block_lists);

private:
    unsigned long history_ = std::numeric_limits<unsigned long>::max();
    /** Linked list of block energies. */
    std::deque<double> block_list_;
    unsigned long block_list_max_ = history_ / 100;
    /** Linked list of 3s-block energies, used to calculate LRA. */
    std::deque<double> short_term_block_list_;
    unsigned long st_block_list_max_ = history_ / 3000;
};

class HistogramCalculator : public BS1770Calculator {
public:
    static constexpr size_t HISTOGRAM_SIZE = 1000;

    void addBlock(double energy) override;
    void addShortTermBlock(double energy) override;
    ValueCounter relativeThreshold() const override;
    ValueCounter gatedLoudness(double relative_threshold) const override;
    double gatedMedianLoudness(double relative_threshold) const override;

    static double loudnessRangeMultiple(const std::vector<const HistogramCalculator*>& histograms);

private:
    static size_t relative_threshold_index(double relative_threshold);

    std::array<unsigned long, HISTOGRAM_SIZE> block_energy_histogram{};
    std::array<unsigned long, HISTOGRAM_SIZE> short_term_block_energy_histogram{};
};


class LoudnessRangeCalculator {
public:
    void addHistCalculator(const HistogramCalculator& calc);
    double getLoudnessRange() const;
private:
    size_t stl_size = 0;
    double stl_power = 0.0;
};

#endif  // BS1770CALCULATOR_HPP
