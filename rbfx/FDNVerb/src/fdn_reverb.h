#pragma once
/** FdnReverb - stereo room reverb: pre-delay, 16-tap early reflections with
 *  optional chorus, and a 16-branch ring of Schroeder allpass + delay + HF
 *  shelf. Port of FDN.m (bakerbass/REVERB_HACKATHON), restructured for Daisy.
 *
 *  Hardware-free so it can be unit-tested on a PC. Delay tables assume a
 *  48 kHz sample rate.
 *
 *  Optimizations vs. the MATLAB version:
 *  - Early reflections: the 32 taps all read ONE mono buffer (the original
 *    kept 16 identical 4096-sample copies per channel).
 *  - One shared write counter; power-of-two buffers wrapped with a mask.
 *  - Delay lengths are a per-sample ramp of a single size scalar, so sweeping
 *    Size glides (tape-like) instead of clicking, with no buffer clearing.
 *  - Filter coefficients (pow/tan/exp) are computed once per block.
 */
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rbfx
{
// ---- tables (samples @ 48 kHz at size = 1) ----
constexpr float kM[16]
    = {223, 281, 331, 379, 419, 457, 479, 499,
       509, 523, 541, 547, 557, 563, 569, 571};
// 3 * {53, 59, 67, 73, 79, 89, 97, 101, 107, 113, 127, 137, 149, 157, 167, 179}
constexpr float kD[16]
    = {159, 177, 201, 219, 237, 267, 291, 303,
       321, 339, 381, 411, 447, 471, 501, 537};
constexpr float kErTapL[16]
    = {70,  98,  153, 204, 269, 367, 385, 418,
       419, 425, 435, 467, 472, 498, 501, 513};
constexpr float kErTapR[16]
    = {70,  82,  139, 204, 258, 367, 385, 409,
       419, 425, 426, 458, 464, 498, 513, 525};
constexpr float kGErL[16]
    = {-0.5208f, -0.4857f, 0.2715f,  -0.3877f, 0.2183f,  0.1873f,
       -0.2888f, -0.1104f, 0.1742f,  0.1729f,  0.1707f,  -0.1036f,
       -0.1029f, 0.1574f,  -0.2480f, -0.2444f};
constexpr float kGErR[16]
    = {-0.5208f, -0.5049f, 0.2798f,  -0.3877f, 0.2226f,  0.1873f,
       -0.2888f, -0.1117f, 0.1742f,  0.1729f,  0.1727f,  -0.1047f,
       -0.1040f, 0.1574f,  -0.2444f, -0.2411f};
constexpr float kALateL[16]
    = {-0.2846f, 0.2684f,  -0.2346f, 0.2084f,  -0.1946f, 0.1784f,
       -0.1446f, 0.1384f,  -0.1246f, 0.1384f,  -0.1546f, 0.1484f,
       -0.1646f, 0.1584f,  -0.1146f, 0.0884f};
constexpr float kALateR[16]
    = {0.2856f,  -0.2674f, 0.2356f,  -0.2074f, 0.1956f,  -0.1774f,
       0.1456f,  -0.1374f, 0.1256f,  0.1374f,  0.1556f,  -0.1474f,
       -0.1656f, -0.1574f, 0.1156f,  -0.0874f};

class FdnReverb
{
  public:
    static constexpr int   kBranches = 16;
    static constexpr float kMaxSize  = 7.f;
    static constexpr float kMaxWidth = 200.f;
    static constexpr float kMaxPre   = 2000.f;
    static constexpr float kFs       = 48000.f;

    /** Feed the last branch back into the first (the intent of the original).
     *  The MATLAB code actually evaluated G^(1/0) = 0 there, i.e. open loop. */
    static constexpr bool kCloseLoop = true;

    static constexpr uint32_t kLineSize   = 4096; // >= kMaxSize*571 + 2
    static constexpr uint32_t kErSize     = 8192; // >= kMaxSize*(525+200) + 2
    static constexpr uint32_t kPreSize    = 2048;
    static constexpr uint32_t kChorusSize = 2048;

    /** Delay memory. Put one of these in SDRAM (~0.55 MB). */
    struct Memory
    {
        float ap[kBranches][kLineSize];
        float dl[kBranches][kLineSize];
        float er[kErSize];
        float pre[kPreSize];
        float chorus[2][kChorusSize];
    };

    struct Params
    {
        float size        = 3.f;     // 1..7, scales every delay
        float predelay    = 1200.f;  // samples, 0..2000
        float feedback    = 0.2f;    // total loop gain, 0..0.95
        float allpassGain = 0.7f;    // 0..0.95
        float mix         = 0.5f;    // 0..1
        float shelfHz     = 20000.f; // HF shelf corner
        float shelfDb     = -10.f;   // HF shelf gain per pass (<= 0)
        float erGainDb    = 0.f;     // early reflection level, dB
        float diffGainDb  = 0.f;     // late tail level, dB
        float width       = 0.f;     // extra right-channel ER offset, samples
        float chorusRate  = 1.f;     // Hz
        float chorusDepth = 0.2f;    // ms of delay modulation, 0..1
        float chorusLevel = 0.5f;    // 0..1
        bool  chorusOn    = false;
        bool  active      = true; // false: stop feeding the reverb, tail rings out
    };

    void Init(Memory* mem, float sampleRate)
    {
        mem_ = mem;
        fs_  = sampleRate;
        std::memset(mem_, 0, sizeof(Memory));
        std::memset(yd_, 0, sizeof(yd_));
        std::memset(hs_, 0, sizeof(hs_));
        w_         = 0;
        lfo_       = 0.f;
        coefN_     = 0;
        size_      = t_.size;
        pre_       = t_.predelay;
        width_     = t_.width;
        fb_        = t_.feedback;
        mix_       = t_.mix;
        chorusLvl_ = 0.f;
        act_       = t_.active ? 1.f : 0.f;
        erG_       = DbToLin(t_.erGainDb);
        diffG_     = DbToLin(t_.diffGainDb);
        ap_        = t_.allpassGain;
        shelfHz_   = t_.shelfHz;
        shelfDb_   = t_.shelfDb;
        depth_     = t_.chorusDepth;
        rate_      = t_.chorusRate;
    }

    /** Set targets; values are clamped and smoothed internally. */
    void SetParams(const Params& p)
    {
        t_             = p;
        t_.size        = Clamp(p.size, 1.f, kMaxSize);
        t_.predelay    = Clamp(p.predelay, 0.f, kMaxPre);
        t_.feedback    = Clamp(p.feedback, 0.f, 0.95f);
        t_.allpassGain = Clamp(p.allpassGain, 0.f, 0.95f);
        t_.mix         = Clamp(p.mix, 0.f, 1.f);
        t_.shelfHz     = Clamp(p.shelfHz, 500.f, 0.45f * fs_);
        t_.shelfDb     = Clamp(p.shelfDb, -40.f, 0.f);
        t_.width       = Clamp(p.width, 0.f, kMaxWidth);
        t_.chorusLevel = Clamp(p.chorusLevel, 0.f, 1.f);
        t_.chorusDepth = Clamp(p.chorusDepth, 0.f, 1.f);
    }

    void Process(const float* inL,
                 const float* inR,
                 float*       outL,
                 float*       outR,
                 size_t       n)
    {
        // ---- per-block parameter smoothing and derived coefficients ----
        if(n != coefN_)
        {
            const float nn = static_cast<float>(n);
            cFast_         = 1.f - std::exp(-nn / (fs_ * 0.02f));
            cSlow_         = 1.f - std::exp(-nn / (fs_ * 0.08f));
            coefN_         = n;
        }
        const float size0 = size_, pre0 = pre_, wid0 = width_, act0 = act_;
        size_ += cSlow_ * (t_.size - size_);
        pre_ += cSlow_ * (t_.predelay - pre_);
        width_ += cSlow_ * (t_.width - width_);
        act_ += cFast_ * ((t_.active ? 1.f : 0.f) - act_);
        fb_ += cFast_ * (t_.feedback - fb_);
        mix_ += cFast_ * (t_.mix - mix_);
        ap_ += cFast_ * (t_.allpassGain - ap_);
        shelfHz_ += cFast_ * (t_.shelfHz - shelfHz_);
        shelfDb_ += cFast_ * (t_.shelfDb - shelfDb_);
        erG_ += cFast_ * (DbToLin(t_.erGainDb) - erG_);
        diffG_ += cFast_ * (DbToLin(t_.diffGainDb) - diffG_);
        chorusLvl_
            += cFast_ * ((t_.chorusOn ? t_.chorusLevel : 0.f) - chorusLvl_);
        depth_ += cFast_ * (t_.chorusDepth - depth_);
        rate_ = t_.chorusRate;

        const float invN = 1.f / static_cast<float>(n);
        const float dSize = (size_ - size0) * invN;
        const float dPre  = (pre_ - pre0) * invN;
        const float dWid  = (width_ - wid0) * invN;
        const float dAct  = (act_ - act0) * invN;

        // per-branch ring gain so the whole loop has gain `fb_`
        const float gBranch = std::pow(fb_, 1.f / kBranches);

        // high-frequency shelving filter (cut case)
        const float V0  = std::pow(10.f, shelfDb_ / 20.f);
        const float H0  = V0 - 1.f;
        const float tw  = std::tan(3.14159265f * shelfHz_ / fs_);
        const float aHf = (1.f - tw * V0) / (1.f + tw * V0);

        float gErL[kBranches], gErR[kBranches];
        float gLateL[kBranches], gLateR[kBranches];
        for(int i = 0; i < kBranches; i++)
        {
            gErL[i]   = 2.f * erG_ * kGErL[i];
            gErR[i]   = 2.f * erG_ * kGErR[i];
            gLateL[i] = diffG_ * kALateL[i];
            gLateR[i] = diffG_ * kALateR[i];
        }

        const float lfoInc    = 6.2831853f * rate_ / fs_;
        const float chMsToSmp = fs_ * 0.001f;
        const float apg       = ap_;
        float       size = size0, pre = pre0, wid = wid0, act = act0;

        Memory& m = *mem_;
        for(size_t k = 0; k < n; k++)
        {
            const uint32_t w = w_++;
            const float    x = 0.5f * (inL[k] + inR[k]) * act;

            // pre-delay
            m.pre[w & (kPreSize - 1)] = x;
            const float pd            = Read(m.pre, kPreSize - 1, w, pre);

            // early reflections: 32 taps on one buffer
            m.er[w & (kErSize - 1)] = x;
            float erL = 0.f, erR = 0.f;
            for(int i = 0; i < kBranches; i++)
            {
                erL += Read(m.er, kErSize - 1, w, size * kErTapL[i]) * gErL[i];
                erR += Read(m.er, kErSize - 1, w, size * (kErTapR[i] + wid))
                       * gErR[i];
            }

            // chorus on the early reflections
            lfo_ += lfoInc;
            if(lfo_ > 6.2831853f)
                lfo_ -= 6.2831853f;
            const float cd = (20.f + depth_ * std::sin(lfo_)) * chMsToSmp;
            m.chorus[0][w & (kChorusSize - 1)] = erL;
            m.chorus[1][w & (kChorusSize - 1)] = erR;
            erL = Read(m.chorus[0], kChorusSize - 1, w, cd) * chorusLvl_
                  + erL * (1.f - chorusLvl_);
            erR = Read(m.chorus[1], kChorusSize - 1, w, cd) * chorusLvl_
                  + erR * (1.f - chorusLvl_);

            // shelved copy of every branch's previous output (ring feedback)
            float hf[kBranches];
            for(int i = 0; i < kBranches; i++)
            {
                const float s  = yd_[i] + aHf * hs_[i];
                const float ya = hs_[i] - aHf * s;
                hs_[i]         = s;
                hf[i]          = 0.5f * H0 * (yd_[i] - ya) + yd_[i];
            }

            // late field
            float lateL = 0.f, lateR = 0.f;
            for(int i = 0; i < kBranches; i++)
            {
                float fb = 0.f;
                if(i > 0)
                    fb = hf[i - 1];
                else if(kCloseLoop)
                    fb = hf[kBranches - 1];
                const float xin = pd + gBranch * fb;

                // Schroeder allpass (read before write: delay >= 1 sample)
                float*      ap  = m.ap[i];
                const float dly = Read(ap, kLineSize - 1, w, size * kM[i]);
                const float s   = xin + apg * dly;
                ap[w & (kLineSize - 1)] = s;
                const float y           = dly - apg * s;

                // output delay
                float* dl               = m.dl[i];
                dl[w & (kLineSize - 1)] = y;
                yd_[i] = Read(dl, kLineSize - 1, w, size * kD[i]);

                lateL += yd_[i] * gLateL[i];
                lateR += yd_[i] * gLateR[i];
            }

            // Mix. Dry stays at unity when inactive; the tail keeps ringing.
            const float wetL = (erL + lateL) * kWetTrim;
            const float wetR = (erR + lateR) * kWetTrim;
            const float dryG = 1.f - mix_ * act;
            outL[k] = Clamp(dryG * inL[k] + mix_ * wetL, -1.f, 1.f);
            outR[k] = Clamp(dryG * inR[k] + mix_ * wetR, -1.f, 1.f);

            size += dSize;
            pre += dPre;
            wid += dWid;
            act += dAct;
        }
    }

  private:
    static constexpr float kWetTrim = 0.25f; // -12 dB, as in the original

    static float Clamp(float v, float lo, float hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static float DbToLin(float db) { return std::pow(10.f, db / 20.f); }

    /** Linear-interpolated read `d` samples behind write slot `w`. */
    static inline float Read(const float* b, uint32_t mask, uint32_t w, float d)
    {
        const uint32_t i = static_cast<uint32_t>(d);
        const float    f = d - static_cast<float>(i);
        const float    a = b[(w - i) & mask];
        const float    c = b[(w - i - 1u) & mask];
        return a + f * (c - a);
    }

    static_assert(kMaxSize * 571 + 2 <= kLineSize, "allpass line too small");
    static_assert(kMaxSize * 3 * 179 + 2 <= kLineSize, "delay line too small");
    static_assert(kMaxSize * (525 + kMaxWidth) + 2 <= kErSize, "ER too small");

    Memory*  mem_ = nullptr;
    float    fs_  = kFs;
    uint32_t w_   = 0;
    Params   t_;

    float size_ = 3.f, pre_ = 0.f, width_ = 0.f, act_ = 1.f, fb_ = 0.f,
          mix_ = 0.f, ap_ = 0.7f, shelfHz_ = 20000.f, shelfDb_ = -10.f,
          erG_ = 1.f, diffG_ = 1.f, chorusLvl_ = 0.f, depth_ = 0.2f,
          rate_ = 1.f, lfo_ = 0.f;
    float  cFast_ = 0.f, cSlow_ = 0.f;
    size_t coefN_ = 0;

    float yd_[kBranches] = {};
    float hs_[kBranches] = {};
};
} // namespace rbfx
