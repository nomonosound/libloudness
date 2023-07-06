#ifndef BS1770CALCULATOR_HPP
#define BS1770CALCULATOR_HPP

#include <array>
#include <cstddef>
#include <deque>
#include <vector>

namespace loudness {
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

        [[nodiscard]] virtual ValueCounter relativeThreshold() const = 0;
        [[nodiscard]] virtual ValueCounter gatedLoudness(double relative_threshold) const = 0;
        [[nodiscard]] virtual double gatedMedianLoudness(double relative_threshold) const = 0;
    };

    class BlockListCalculator : public BS1770Calculator {
    public:
        BlockListCalculator();

        void addBlock(double energy) override;
        void addShortTermBlock(double energy) override;
        bool setMaxHistory(unsigned long history_ms) override;

        [[nodiscard]] ValueCounter relativeThreshold() const override;
        [[nodiscard]] ValueCounter gatedLoudness(double relative_threshold) const override;
        [[nodiscard]] double gatedMedianLoudness(double relative_threshold) const override;

        static double loudnessRangeMultiple(const std::vector<const BlockListCalculator*>& block_lists);

    private:
        unsigned long history_ms_;
        /** Container of block energies. */
        std::deque<double> block_list_;
        unsigned long block_list_max_;
        /** Container of 3s-block energies, used to calculate LRA. */
        std::deque<double> short_term_block_list_;
        unsigned long st_block_list_max_;
    };

    class HistogramCalculator : public BS1770Calculator {
    public:
        static constexpr size_t HISTOGRAM_SIZE = 1000;

        void addBlock(double energy) override;
        void addShortTermBlock(double energy) override;

        [[nodiscard]] ValueCounter relativeThreshold() const override;
        [[nodiscard]] ValueCounter gatedLoudness(double relative_threshold) const override;
        [[nodiscard]] double gatedMedianLoudness(double relative_threshold) const override;

        static double loudnessRangeMultiple(const std::vector<const HistogramCalculator*>& histograms);

    private:
        static size_t relative_threshold_index(double relative_threshold);

        std::array<unsigned long, HISTOGRAM_SIZE> block_energy_histogram{};
        std::array<unsigned long, HISTOGRAM_SIZE> short_term_block_energy_histogram{};
    };
} // namespace loudness
#endif  // BS1770CALCULATOR_HPP
