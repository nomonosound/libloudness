#ifndef INTERPOLATOR_HPP
#define INTERPOLATOR_HPP

#include <vector>

struct InterpFilter {
    unsigned int count = 0; /* Number of coefficients in this subfilter */
    std::vector<unsigned int> index;
    std::vector<double> coeff;
};

class Interpolator {
    /* Polyphase FIR interpolator */
public:
    Interpolator(unsigned int taps, unsigned int factor, unsigned int channels);
    std::size_t process(std::size_t frames, const float* in, float* out);
    unsigned int factor() const { return factor_; };

private:
    const unsigned int factor_;   /* Interpolation factor of the interpolator */
                                  //    unsigned int taps_;     /* Taps (prefer odd to increase zero coeffs) */
    const unsigned int channels_; /* Number of channels */
    const unsigned int delay_;    /* Size of delay buffer */
    const unsigned int out_stride_;
    std::vector<InterpFilter> filter_;
    std::vector<std::vector<float>> z_; /* List of delay buffers (one for each channel) */
    unsigned int zi_;                   /* Current delay buffer index */
};

#endif  // INTERPOLATOR_HPP
