
#ifndef WAVESHAPE_H
#define WAVESHAPE_H

#include "Utility/dsp.h"
#include <cmath>
#include <algorithm>
using namespace daisysp;

class Waveshape
{
    public:
    Waveshape() {}
    ~Waveshape() {}

    void Init()
    {
        Reset();
    }

    inline void Reset() { drive_ = 0.f; }

    inline float Process(float in)
    {
        return atan(2.25 * in) * sqrt(abs(tanh(drive_ * in)));
    }

    inline void SetDrive(float drive)
    {
        drive = fclamp(drive, 0.f, 1.f);
        drive_ = 2 * drive;
    }
    private:
    float drive_;

};
#endif //WAVESHAPE_H