/* See COPYING file for copyright and license details. */

#include "ebur128.hpp"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath> /* You may have to define _USE_MATH_DEFINES if you use MSVC */
#include <cstdio>
#include <deque>
#include <numeric>
#include <type_traits>
#include <vector>

#include <gcem.hpp>

#include "bs1770-calculator.hpp"
#include "k-filter.hpp"
#include "true-peak-calculator.hpp"
#include "utils.hpp"
#include "constants.hpp"

struct Ebur128Impl {
    Ebur128Impl(unsigned int channels, unsigned long samplerate, unsigned int mode);

    void initChannelMap(size_t num_channels);
    void initFilter(double samplerate, unsigned int channels);
    double energyShortterm();
    double calcGatingBlock(size_t frames_per_block, bool add_block);
    int energyInInterval(size_t interval_frames, double* out);
    /** Filtered audio data (used as ring buffer). */
    std::vector<double> audio_data;
    /** Size of audio_data array. */
    size_t audio_data_frames;
    /** Current index for audio_data. */
    size_t audio_data_index = 0;
    /** The channel map. Has as many elements as there are channels. */
    std::vector<Channel> channel_map;
    /** How many samples fit in 100ms (rounded). */
    unsigned long samples_in_100ms;
    /** How many frames are needed for a gating block. Will correspond to 400ms
     *  of audio at initialization, and 100ms after the first block (75% overlap
     *  as specified in the 2011 revision of BS1770). */
    unsigned long needed_frames;
    KFilter filter;

    /** Keeps track of when a new short term block is needed. */
    size_t short_term_frame_counter = 0;
    /** Maximum sample peak, one per channel */
    std::vector<double> sample_peak;
    std::vector<double> prev_sample_peak;
    /** Maximum true peak, one per channel */
    std::vector<double> true_peak;
    /** The maximum window duration in ms. */
    unsigned long window;

    TruePeakCalculator resampler;
    std::unique_ptr<BS1770Calculator> bs1770_calculator;
};

void Ebur128Impl::initChannelMap(size_t num_channels)
{
    channel_map.resize(num_channels);
    if (num_channels == 4) {
        channel_map[0] = Channel::LEFT;
        channel_map[1] = Channel::RIGHT;
        channel_map[2] = Channel::LEFT_SURROUND;
        channel_map[3] = Channel::RIGHT_SURROUND;
    }
    else if (num_channels == 5) {
        channel_map[0] = Channel::LEFT;
        channel_map[1] = Channel::RIGHT;
        channel_map[2] = Channel::CENTER;
        channel_map[3] = Channel::LEFT_SURROUND;
        channel_map[4] = Channel::RIGHT_SURROUND;
    }
    else {
        for (size_t i = 0; i < num_channels; ++i) {
            switch (i) {
            case 0:
                channel_map[i] = Channel::LEFT;
                break;
            case 1:
                channel_map[i] = Channel::RIGHT;
                break;
            case 2:
                channel_map[i] = Channel::CENTER;
                break;
            case 3:
                channel_map[i] = Channel::UNUSED;
                break;
            case 4:
                channel_map[i] = Channel::LEFT_SURROUND;
                break;
            case 5:
                channel_map[i] = Channel::RIGHT_SURROUND;
                break;
            default:
                channel_map[i] = Channel::UNUSED;
                break;
            }
        }
    }
}

void ebur128_get_version(int* major, int* minor, int* patch)
{
    *major = EBUR128_VERSION_MAJOR;
    *minor = EBUR128_VERSION_MINOR;
    *patch = EBUR128_VERSION_PATCH;
}

Ebur128::Ebur128(unsigned int channels, unsigned long samplerate, unsigned int mode)
    : mode(mode), channels(channels), samplerate(samplerate)
{
    assert(channels > 0 && channels <= MAX_CHANNELS);
    assert(samplerate >= MIN_SAMPLERATE && samplerate <= MAX_SAMPLERATE);

    pimpl = std::make_unique<Ebur128Impl>(channels, samplerate, mode);
}

Ebur128::~Ebur128() = default;

Ebur128Impl::Ebur128Impl(unsigned int channels, unsigned long samplerate, unsigned int mode)
    : resampler(samplerate, channels),
      sample_peak(channels),
      prev_sample_peak(channels),
      true_peak(channels),
      samples_in_100ms((samplerate + 5) / 10),
      /* the first block needs 400ms of audio data */
      needed_frames(samples_in_100ms * 4),
      filter(static_cast<double>(samplerate), channels)
{
    initChannelMap(channels);

    if ((mode & EBUR128_MODE_S) == EBUR128_MODE_S) {
        window = 3000;
    }
    else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M) {
        window = 400;
    }
    else {
        // No valid mode?
        return;
    }
    audio_data_frames = samplerate * window / 1000;
    if (audio_data_frames % samples_in_100ms) {
        /* round up to multiple of samples_in_100ms */
        audio_data_frames = (audio_data_frames + samples_in_100ms) - (audio_data_frames % samples_in_100ms);
    }
    audio_data.resize(audio_data_frames * channels);

    if (mode & EBUR128_MODE_HISTOGRAM) {
        bs1770_calculator = std::make_unique<HistogramCalculator>();
    }
    else {
        bs1770_calculator = std::make_unique<BlockListCalculator>();
    }
}

#if defined(__SSE2_MATH__) || defined(_M_X64) || _M_IX86_FP >= 2
#include <xmmintrin.h>
#define TURN_ON_FTZ                          \
    const unsigned int mxcsr = _mm_getcsr(); \
    _mm_setcsr(mxcsr | _MM_FLUSH_ZERO_ON);
#define TURN_OFF_FTZ _mm_setcsr(mxcsr);
#define FLUSH_MANUALLY
#else
#warning "manual FTZ is being used, please enable SSE2 (-msse2 -mfpmath=sse)"
#define TURN_ON_FTZ
#define TURN_OFF_FTZ
#define FLUSH_MANUALLY                                                          \
    pimpl->v[c][4] = std::abs(pimpl->v[c][4]) < DBL_MIN ? 0.0 : pimpl->v[c][4]; \
    pimpl->v[c][3] = std::abs(pimpl->v[c][3]) < DBL_MIN ? 0.0 : pimpl->v[c][3]; \
    pimpl->v[c][2] = std::abs(pimpl->v[c][2]) < DBL_MIN ? 0.0 : pimpl->v[c][2]; \
    pimpl->v[c][1] = std::abs(pimpl->v[c][1]) < DBL_MIN ? 0.0 : pimpl->v[c][1];
#endif

template <typename T>
void Ebur128::filter(const T* src, size_t frames)
{
    static constexpr double scaling_factor = getScalingFactor<T>();

    double* audio_data = pimpl->audio_data.data() + pimpl->audio_data_index;
    size_t i, c;

    TURN_ON_FTZ

    // Find new sample peak
    if ((mode & EBUR128_MODE_SAMPLE_PEAK) == EBUR128_MODE_SAMPLE_PEAK) {
        for (c = 0; c < channels; ++c) {
            double max = 0.0;
            for (i = 0; i < frames; ++i) {
                const double cur = std::abs(static_cast<double>(src[i * channels + c]));
                if (cur > max) {
                    max = cur;
                }
            }
            max /= scaling_factor;
            if (max > pimpl->prev_sample_peak[c]) {
                pimpl->prev_sample_peak[c] = max;
            }
        }
    }

    // Find new true peak
    if ((mode & EBUR128_MODE_TRUE_PEAK) == EBUR128_MODE_TRUE_PEAK) {
        pimpl->resampler.process(src, frames);
    }

    // Apply filter
    for (c = 0; c < channels; ++c) {
        if (pimpl->channel_map[c] == Channel::UNUSED) {
            continue;
        }
        for (i = 0; i < frames; ++i) {
            audio_data[i * channels + c] = pimpl->filter.apply(
                static_cast<double>(static_cast<double>(src[i * channels + c]) / scaling_factor), c);
        }
        FLUSH_MANUALLY
    }
    TURN_OFF_FTZ
}

double Ebur128Impl::calcGatingBlock(size_t frames_per_block, bool add_block)
{
    size_t i;
    double sum = 0.0;
    const size_t channels = channel_map.size();
    for (size_t c = 0; c < channels; ++c) {
        if (channel_map[c] == Channel::UNUSED) {
            continue;
        }
        double channel_sum = 0.0;
        if (audio_data_index < frames_per_block * channels) {
            for (i = 0; i < audio_data_index / channels; ++i) {
                channel_sum += audio_data[i * channels + c] * audio_data[i * channels + c];
            }
            for (i = audio_data_frames - (frames_per_block - audio_data_index / channels); i < audio_data_frames; ++i) {
                channel_sum += audio_data[i * channels + c] * audio_data[i * channels + c];
            }
        }
        else {
            for (i = audio_data_index / channels - frames_per_block; i < audio_data_index / channels; ++i) {
                channel_sum += audio_data[i * channels + c] * audio_data[i * channels + c];
            }
        }
        switch (channel_map[c]) {
            case Channel::Mp110: case Channel::Mm110: case Channel::Mp060: case Channel::Mm060: case Channel::Mp090: case  Channel::Mm090:
            channel_sum *= 1.41;
            break;
        case Channel::DUAL_MONO:
            channel_sum *= 2.0;
            break;
        default:
            break;
        }

        sum += channel_sum;
    }

    sum /= static_cast<double>(frames_per_block);

    if (add_block && sum >= absolute_gate) {
        bs1770_calculator->addBlock(sum);
    }

    return sum;
}

int Ebur128::setChannel(unsigned int channel_number, Channel value)
{
    if (channel_number >= channels) {
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }
    if (value == Channel::DUAL_MONO && (channels != 1 || channel_number != 0)) {
        fprintf(stderr, "EBUR128_DUAL_MONO only works with mono files!\n");
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }
    pimpl->channel_map[channel_number] = value;
    return EBUR128_SUCCESS;
}

int Ebur128::changeParameters(unsigned int channels, unsigned long samplerate)
{
    if (channels == 0 || channels > MAX_CHANNELS) {
        return EBUR128_ERROR_NOMEM;
    }

    if (samplerate < MIN_SAMPLERATE || samplerate > MAX_SAMPLERATE) {
        return EBUR128_ERROR_NOMEM;
    }

    if (this->channels == channels && this->samplerate == samplerate) {
        return EBUR128_ERROR_NO_CHANGE;
    }

    if (channels != this->channels) {
        pimpl->sample_peak.assign(channels, 0.0);
        pimpl->prev_sample_peak.assign(channels, 0.0);
        pimpl->true_peak.assign(channels, 0.0);

        pimpl->initChannelMap(channels);
    }
    if (samplerate != this->samplerate) {
        this->samplerate = samplerate;
        pimpl->samples_in_100ms = (samplerate + 5) / 10;
    }

    /* If we're here, either samplerate or channels
     * have changed. Re-init filter. */
    pimpl->filter = KFilter(static_cast<double>(samplerate), channels);

    pimpl->audio_data_frames = samplerate * pimpl->window / 1000;
    if (pimpl->audio_data_frames % pimpl->samples_in_100ms) {
        /* round up to multiple of samples_in_100ms */
        pimpl->audio_data_frames =
            (pimpl->audio_data_frames + pimpl->samples_in_100ms) - (pimpl->audio_data_frames % pimpl->samples_in_100ms);
    }
    pimpl->audio_data.assign(pimpl->audio_data_frames * channels, 0.0);

    pimpl->resampler.configure(samplerate, channels);
    /* the first block needs 400ms of audio data */
    pimpl->needed_frames = pimpl->samples_in_100ms * 4;
    /* start at the beginning of the buffer */
    pimpl->audio_data_index = 0;
    /* reset short term frame counter */
    pimpl->short_term_frame_counter = 0;

    return EBUR128_SUCCESS;
}

int Ebur128::setMaxWindow(unsigned long window)
{
    if ((mode & EBUR128_MODE_S) == EBUR128_MODE_S && window < 3000) {
        window = 3000;
    }
    else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M && window < 400) {
        window = 400;
    }

    if (window == pimpl->window) {
        return EBUR128_ERROR_NO_CHANGE;
    }

    size_t new_audio_data_frames;
    if (not safe_size_mul(samplerate, window, &new_audio_data_frames) ||
        new_audio_data_frames > (static_cast<size_t>(-1)) - pimpl->samples_in_100ms) {
        return EBUR128_ERROR_NOMEM;
    }
    if (new_audio_data_frames % pimpl->samples_in_100ms) {
        /* round up to multiple of samples_in_100ms */
        new_audio_data_frames =
            (new_audio_data_frames + pimpl->samples_in_100ms) - (new_audio_data_frames % pimpl->samples_in_100ms);
    }

    size_t new_audio_data_size;
    if (not safe_size_mul(new_audio_data_frames, channels * sizeof(double), &new_audio_data_size)) {
        return EBUR128_ERROR_NOMEM;
    }

    pimpl->window = window;
    pimpl->audio_data.assign(pimpl->audio_data_frames * channels, 0.0);

    /* the first block needs 400ms of audio data */
    pimpl->needed_frames = pimpl->samples_in_100ms * 4;
    /* start at the beginning of the buffer */
    pimpl->audio_data_index = 0;
    /* reset short term frame counter */
    pimpl->short_term_frame_counter = 0;

    return EBUR128_SUCCESS;
}

int Ebur128::setMaxHistory(unsigned long history)
{
    if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA && history < 3000) {
        history = 3000;
    }
    else if ((mode & EBUR128_MODE_M) == EBUR128_MODE_M && history < 400) {
        history = 400;
    }

    if (pimpl->bs1770_calculator->setMaxHistory(history)) {
        return EBUR128_SUCCESS;
    }
    else {
        return EBUR128_ERROR_NO_CHANGE;
    }
}

template <typename T>
int Ebur128::addFrames(const T* src, size_t frames)
{
    size_t src_index = 0;
    pimpl->prev_sample_peak.assign(channels, 0.0);
    pimpl->resampler.clearTruePeaks();
    while (frames > 0) {
        if (frames >= pimpl->needed_frames) {
            filter(src + src_index, pimpl->needed_frames);
            src_index += pimpl->needed_frames * channels;
            frames -= pimpl->needed_frames;
            pimpl->audio_data_index += pimpl->needed_frames * channels;
            /* calculate the new gating block */
            if ((mode & EBUR128_MODE_I) == EBUR128_MODE_I) {
                pimpl->calcGatingBlock(pimpl->samples_in_100ms * 4, true);
            }
            if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA) {
                pimpl->short_term_frame_counter += pimpl->needed_frames;
                if (pimpl->short_term_frame_counter == pimpl->samples_in_100ms * 30) {
                    const double st_energy = pimpl->energyShortterm();
                    if (st_energy >= absolute_gate) {
                        pimpl->bs1770_calculator->addShortTermBlock(st_energy);
                    }
                    pimpl->short_term_frame_counter = pimpl->samples_in_100ms * 20;
                }
            }
            /* 100ms are needed for all blocks besides the first one */
            pimpl->needed_frames = pimpl->samples_in_100ms;
            /* reset audio_data_index when buffer full */
            if (pimpl->audio_data_index == pimpl->audio_data_frames * channels) {
                pimpl->audio_data_index = 0;
            }
        }
        else {
            filter(src + src_index, frames);
            pimpl->audio_data_index += frames * channels;
            if ((mode & EBUR128_MODE_LRA) == EBUR128_MODE_LRA) {
                pimpl->short_term_frame_counter += frames;
            }
            pimpl->needed_frames -= static_cast<unsigned long>(frames);
            frames = 0;
        }
    }
    for (unsigned int c = 0; c < channels; c++) {
        if (pimpl->prev_sample_peak[c] > pimpl->sample_peak[c]) {
            pimpl->sample_peak[c] = pimpl->prev_sample_peak[c];
        }
        if (pimpl->resampler.prevTruePeak(c) > pimpl->true_peak[c]) {
            pimpl->true_peak[c] = pimpl->resampler.prevTruePeak(c);
        }
    }
    return EBUR128_SUCCESS;
}

template int Ebur128::addFrames<short>(const short* src, size_t frames);
template int Ebur128::addFrames<int>(const int* src, size_t frames);
template int Ebur128::addFrames<float>(const float* src, size_t frames);
template int Ebur128::addFrames<double>(const double* src, size_t frames);

template <typename Iterator>
static int ebur128_gated_loudness(Iterator begin, Iterator end, double* out) requires std::same_as<typename std::iterator_traits<Iterator>::value_type, Ebur128>
{
    double gated_loudness = 0.0;
    double relative_threshold = 0.0;
    size_t above_thresh_counter = 0;

    for (auto st = begin; st != end; ++st) {
        if (st && (st->mode & EBUR128_MODE_I) != EBUR128_MODE_I) {
            return EBUR128_ERROR_INVALID_MODE;
        }
    }

    for (auto st = begin; st != end; ++st) {
        auto value = st->pimpl->bs1770_calculator->relativeThreshold();
        relative_threshold += value.sum;
        above_thresh_counter += value.counter;
    }
    if (!above_thresh_counter) {
        *out = -HUGE_VAL;
        return EBUR128_SUCCESS;
    }

    relative_threshold /= static_cast<double>(above_thresh_counter);
    relative_threshold *= relative_gate_factor;

    above_thresh_counter = 0;
    for (auto st = begin; st != end; ++st) {
        auto value = st->pimpl->bs1770_calculator->gatedLoudness(relative_threshold);
        above_thresh_counter += value.counter;
        gated_loudness += value.sum;
    }
    if (above_thresh_counter == 0) {
        *out = -HUGE_VAL;
        return EBUR128_SUCCESS;
    }
    gated_loudness /= static_cast<double>(above_thresh_counter);
    *out = EnergyToLoudness(gated_loudness);
    return EBUR128_SUCCESS;
}

int Ebur128::relativeThreshold(double* out)
{
    if ((mode & EBUR128_MODE_I) != EBUR128_MODE_I) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    auto [above_thresh_counter, relative_threshold] = pimpl->bs1770_calculator->relativeThreshold();

    if (above_thresh_counter == 0) {
        *out = -70.0;
        return EBUR128_SUCCESS;
    }

    relative_threshold /= static_cast<double>(above_thresh_counter);
    relative_threshold *= relative_gate_factor;

    *out = EnergyToLoudness(relative_threshold);
    return EBUR128_SUCCESS;
}

int Ebur128::loudnessGlobal(double* out) { return ebur128_gated_loudness(this, this + 1, out); }

int ebur128_loudness_global_multiple(Ebur128* begin, Ebur128* end, double* out)
{
    return ebur128_gated_loudness(begin, end, out);
}

int Ebur128Impl::energyInInterval(size_t interval_frames, double* out)
{
    if (interval_frames > audio_data_frames) {
        return EBUR128_ERROR_INVALID_MODE;
    }
    *out = calcGatingBlock(interval_frames, false);

    return EBUR128_SUCCESS;
}

double Ebur128Impl::energyShortterm()
{
    double out{};
    energyInInterval(samples_in_100ms * 30, &out);
    return out;
}

int Ebur128::loudnessMomentary(double* out)
{
    double energy{};
    int error;

    error = pimpl->energyInInterval(pimpl->samples_in_100ms * 4, &energy);
    if (error) {
        return error;
    }

    if (energy <= 0.0) {
        *out = -HUGE_VAL;
        return EBUR128_SUCCESS;
    }

    *out = EnergyToLoudness(energy);
    return EBUR128_SUCCESS;
}

int Ebur128::loudnessShortterm(double* out)
{
    const double energy = pimpl->energyShortterm();

    if (energy <= 0.0) {
        *out = -HUGE_VAL;
        return EBUR128_SUCCESS;
    }

    *out = EnergyToLoudness(energy);
    return EBUR128_SUCCESS;
}

int Ebur128::loudnessWindow(unsigned long window, double* out)
{
    size_t interval_frames;
    int error;

    if (window > pimpl->window) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    interval_frames = samplerate * window / 1000;
    if (interval_frames > pimpl->audio_data_frames) {
        return EBUR128_ERROR_INVALID_MODE;
    }
    const double energy = pimpl->calcGatingBlock(interval_frames, false);

    if (energy <= 0.0) {
        *out = -HUGE_VAL;
        return EBUR128_SUCCESS;
    }

    *out = EnergyToLoudness(energy);
    return EBUR128_SUCCESS;
}

/* EBU - TECH 3342 */
template <typename Iterator>
int ebur128_loudness_range_multiple(Iterator begin, Iterator end, double* out) requires std::same_as<typename std::iterator_traits<Iterator>::value_type, Ebur128>
{
    bool use_histogram = false;

    for (auto st = begin; st != end; ++st) {
        if (st) {
            if ((st->mode & EBUR128_MODE_LRA) != EBUR128_MODE_LRA) {
                return EBUR128_ERROR_INVALID_MODE;
            }
            if (st == begin && st->mode & EBUR128_MODE_HISTOGRAM) {
                use_histogram = true;
            }
            else if (use_histogram != !!(st->mode & EBUR128_MODE_HISTOGRAM)) {
                return EBUR128_ERROR_INVALID_MODE;
            }
        }
    }

    // TODO: Find better solution here, avoiding dynamic cast
    if (use_histogram) {
        std::vector<const HistogramCalculator*> hists;
        for (auto st = begin; st != end; ++st){
            hists.push_back(dynamic_cast<HistogramCalculator*>(st->pimpl->bs1770_calculator.get()));
        }
        *out = HistogramCalculator::loudnessRangeMultiple(hists);
        return EBUR128_SUCCESS;
    }

    std::vector<const BlockListCalculator*> lists;
    for (auto st = begin; st != end; ++st){
        lists.push_back(dynamic_cast<BlockListCalculator*>(st->pimpl->bs1770_calculator.get()));
    }
    *out = BlockListCalculator::loudnessRangeMultiple(lists);
    return EBUR128_SUCCESS;
}

int Ebur128::loudnessRange(double* out) { return ebur128_loudness_range_multiple(this, this + 1, out); }

int Ebur128::samplePeak(unsigned int channel_number, double* out) const
{
    if ((mode & EBUR128_MODE_SAMPLE_PEAK) != EBUR128_MODE_SAMPLE_PEAK) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    if (channel_number >= channels) {
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }

    *out = pimpl->sample_peak[channel_number];
    return EBUR128_SUCCESS;
}

int Ebur128::prevSamplePeak(unsigned int channel_number, double* out) const
{
    if ((mode & EBUR128_MODE_SAMPLE_PEAK) != EBUR128_MODE_SAMPLE_PEAK) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    if (channel_number >= channels) {
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }

    *out = pimpl->prev_sample_peak[channel_number];
    return EBUR128_SUCCESS;
}

int Ebur128::truePeak(unsigned int channel_number, double* out) const
{
    if ((mode & EBUR128_MODE_TRUE_PEAK) != EBUR128_MODE_TRUE_PEAK) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    if (channel_number >= channels) {
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }

    *out = std::max(pimpl->true_peak[channel_number], pimpl->sample_peak[channel_number]);
    return EBUR128_SUCCESS;
}

int Ebur128::prevTruePeak(unsigned int channel_number, double* out) const
{
    if ((mode & EBUR128_MODE_TRUE_PEAK) != EBUR128_MODE_TRUE_PEAK) {
        return EBUR128_ERROR_INVALID_MODE;
    }

    if (channel_number >= channels) {
        return EBUR128_ERROR_INVALID_CHANNEL_INDEX;
    }

    *out = std::max(static_cast<double>(pimpl->resampler.prevTruePeak(channel_number)),
                    pimpl->prev_sample_peak[channel_number]);
    return EBUR128_SUCCESS;
}
