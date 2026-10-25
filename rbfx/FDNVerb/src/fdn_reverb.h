#pragma once
/** FdnReverb - stereo room reverb.
 *
 *  Signal flow (mono in, stereo out):
 *
 *    in L+R -> mono -+-> pre-delay --> late ring (16 branches) --> L/R --+
 *                    |                                                   +--> mix
 *                    +-> early reflections (32 taps) --> chorus --> L/R -+
 *
 *  Each late branch is a Schroeder allpass followed by a plain delay. The
 *  delayed outputs pass through a high-frequency shelf and feed the NEXT
 *  branch, closing a ring (last branch -> first) whose total gain is the
 *  `feedback` parameter.
 *
 *  Port of FDN.m (bakerbass/REVERB_HACKATHON). Hardware-free (no libDaisy or
 *  DaisySP) so it can be unit-tested on a PC. Delay tables assume 48 kHz.
 *
 *  Comment tags:  OPT: = an optimization,  MEM: = a memory access and why.
 */
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace rbfx
{
constexpr int kBranches = 16;

// ---------------------------------------------------------------------------
// Tables. Delays are in samples at 48 kHz for Size = 1; the Size control
// multiplies all of them. All values come from FDN.m.
// ---------------------------------------------------------------------------

/** Allpass delay per late branch (mutually prime-ish, to avoid shared
 *  resonances). */
constexpr float kAllpassDelay[kBranches]
    = {223, 281, 331, 379, 419, 457, 479, 499,
       509, 523, 541, 547, 557, 563, 569, 571};

/** Plain delay after each branch's allpass: 3 * {53, 59, 67, ... 179}. */
constexpr float kBranchDelay[kBranches]
    = {159, 177, 201, 219, 237, 267, 291, 303,
       321, 339, 381, 411, 447, 471, 501, 537};

/** Early-reflection tap positions, left and right. */
constexpr float kEarlyTapL[kBranches]
    = {70,  98,  153, 204, 269, 367, 385, 418,
       419, 425, 435, 467, 472, 498, 501, 513};
constexpr float kEarlyTapR[kBranches]
    = {70,  82,  139, 204, 258, 367, 385, 409,
       419, 425, 426, 458, 464, 498, 513, 525};

/** Early-reflection tap gains, left and right. */
constexpr float kEarlyGainL[kBranches]
    = {-0.5208f, -0.4857f, 0.2715f,  -0.3877f, 0.2183f,  0.1873f,
       -0.2888f, -0.1104f, 0.1742f,  0.1729f,  0.1707f,  -0.1036f,
       -0.1029f, 0.1574f,  -0.2480f, -0.2444f};
constexpr float kEarlyGainR[kBranches]
    = {-0.5208f, -0.5049f, 0.2798f,  -0.3877f, 0.2226f,  0.1873f,
       -0.2888f, -0.1117f, 0.1742f,  0.1729f,  0.1727f,  -0.1047f,
       -0.1040f, 0.1574f,  -0.2444f, -0.2411f};

/** How much each late branch contributes to the left and right output. Mixed
 *  signs and magnitudes decorrelate the two channels. */
constexpr float kLateGainL[kBranches]
    = {-0.2846f, 0.2684f,  -0.2346f, 0.2084f,  -0.1946f, 0.1784f,
       -0.1446f, 0.1384f,  -0.1246f, 0.1384f,  -0.1546f, 0.1484f,
       -0.1646f, 0.1584f,  -0.1146f, 0.0884f};
constexpr float kLateGainR[kBranches]
    = {0.2856f,  -0.2674f, 0.2356f,  -0.2074f, 0.1956f,  -0.1774f,
       0.1456f,  -0.1374f, 0.1256f,  0.1374f,  0.1556f,  -0.1474f,
       -0.1656f, -0.1574f, 0.1156f,  -0.0874f};

class FdnReverb
{
  public:
    static constexpr float kMaxSize  = 7.f;
    static constexpr float kMaxWidth = 200.f;  // samples
    static constexpr float kMaxPre   = 2000.f; // samples
    static constexpr float kFs       = 48000.f;

    /** Feed the last branch back into the first (the intent of the original).
     *  The MATLAB code actually evaluated G^(1/0) = 0 there, i.e. open loop. */
    static constexpr bool kCloseLoop = true;

    // MEM: every buffer size is a power of two so wrapping is a bit-mask.
    // Each must exceed the longest delay read from it (see static_asserts).
    static constexpr uint32_t kLineSize   = 4096;
    static constexpr uint32_t kEarlySize  = 8192;
    static constexpr uint32_t kPreSize    = 2048;
    static constexpr uint32_t kChorusSize = 2048;

    /** All delay memory. MEM: ~0.55 MB, far too big for internal SRAM, so the
     *  caller places one instance in SDRAM (DSY_SDRAM_BSS). Each line is read
     *  and written sequentially, which suits the Cortex-M7 data cache (one
     *  cache-line fill serves several consecutive samples). */
    struct Memory
    {
        float allpass[kBranches][kLineSize]; // allpass state, one line per branch
        float branch[kBranches][kLineSize];  // plain delay, one line per branch
        float early[kEarlySize];             // mono input history for ER taps
        float pre[kPreSize];                 // pre-delay
        float chorus[2][kChorusSize];        // L/R early-reflection chorus
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
        bool  active = true; // false: stop feeding the reverb, tail rings out
    };

    /** Zero all state. `mem` must stay valid for the life of the reverb. */
    void Init(Memory* mem, float sampleRate)
    {
        mem_ = mem;
        fs_  = sampleRate;

        // MEM: clear every delay line so stale data never leaks into the
        // output. (SDRAM .bss is already zero at boot; this covers re-Init.)
        std::memset(mem_, 0, sizeof(Memory));
        std::memset(branchOut_, 0, sizeof(branchOut_));
        std::memset(shelfState_, 0, sizeof(shelfState_));

        writePos_ = 0;
        lfoPhase_ = 0.f;
        coefBlockSize_ = 0;

        // Start the smoothed values at the targets so there is no initial glide.
        cur_.size          = t_.size;
        cur_.predelay      = t_.predelay;
        cur_.width         = t_.width;
        cur_.inputGain     = t_.active ? 1.f : 0.f;
        cur_.loopGain      = t_.feedback;
        cur_.mix           = t_.mix;
        cur_.allpassGain   = t_.allpassGain;
        cur_.shelfHz       = t_.shelfHz;
        cur_.shelfDb       = t_.shelfDb;
        cur_.earlyGain     = DbToLin(t_.erGainDb);
        cur_.lateGain      = DbToLin(t_.diffGainDb);
        cur_.chorusLevel   = 0.f;
        cur_.chorusDepthMs = t_.chorusDepth;
        chorusRateHz_      = t_.chorusRate;
    }

    /** Set targets. Values are clamped here and smoothed in Process(), so this
     *  is safe to call every block with a noisy knob reading. */
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
        t_.chorusDepth = Clamp(p.chorusDepth, 0.f, 100.f);
    }

    /** Process `n` samples. Input is summed to mono; output is stereo. */
    void Process(const float* inL,
                 const float* inR,
                 float*       outL,
                 float*       outR,
                 size_t       n)
    {
        Block blk = PrepareBlock(n);
        Memory& m = *mem_;

        for(size_t k = 0; k < n; k++)
        {
            // MEM: one free-running write position shared by every buffer.
            // Each buffer wraps it with its own mask (see DelayWrite/Read), so
            // there is no per-line write pointer to advance or bounds-check.
            const uint32_t w = writePos_++;

            // inputGain fades the reverb input to 0 when bypassed (tail rings).
            const float mono = 0.5f * (inL[k] + inR[k]) * blk.inputGain.value;

            const float preDelayed = PreDelay(m, w, mono, blk.predelay.value);

            Stereo early = EarlyReflections(m, w, mono, blk);
            early        = Chorus(m, w, blk, early);

            float shelved[kBranches];
            ShelveBranchOutputs(blk, shelved);

            const Stereo late = LateField(m, w, preDelayed, shelved, blk);

            // Dry stays at unity when inactive; only the tail is scaled by mix.
            const float wetL = (early.l + late.l) * kWetTrim;
            const float wetR = (early.r + late.r) * kWetTrim;
            const float dryG = 1.f - cur_.mix * blk.inputGain.value;
            outL[k] = Clamp(dryG * inL[k] + cur_.mix * wetL, -1.f, 1.f);
            outR[k] = Clamp(dryG * inR[k] + cur_.mix * wetR, -1.f, 1.f);

            blk.size.Advance();
            blk.predelay.Advance();
            blk.width.Advance();
            blk.inputGain.Advance();
        }
    }

  private:
    static constexpr float kWetTrim  = 0.25f; // -12 dB, as in the original
    static constexpr float kTwoPi    = 6.2831853f;
    static constexpr float kChorusMs = 20.f;  // chorus center delay
    static constexpr float kFastTau  = 0.02f; // s, gains / mix / filters
    static constexpr float kSlowTau  = 0.08f; // s, anything that moves delays

    struct Stereo
    {
        float l, r;
    };

    /** A value that moves linearly across one block. */
    struct Ramp
    {
        float value, step;
        void  Advance() { value += step; }
    };

    /** The smoothed parameter values the DSP actually uses; they chase the
     *  clamped targets in `t_`. */
    struct Smoothed
    {
        float size = 3.f, predelay = 0.f, width = 0.f, inputGain = 1.f;
        float loopGain = 0.f, mix = 0.f, allpassGain = 0.7f;
        float shelfHz = 20000.f, shelfDb = -10.f;
        float earlyGain = 1.f, lateGain = 1.f;
        float chorusLevel = 0.f, chorusDepthMs = 0.2f;
    };

    /** Everything derived once per block so the sample loop does no
     *  transcendental math. */
    struct Block
    {
        Ramp  size, predelay, width, inputGain; // glide across the block
        float branchGain;                       // per-branch ring gain
        float shelfH0, shelfCoef;               // HF shelf
        float allpassGain;
        float lfoStep, chorusMsToSamples;
        float earlyGainL[kBranches], earlyGainR[kBranches]; // incl. level
        float lateGainL[kBranches], lateGainR[kBranches];   // incl. level
    };

    // ---- small helpers ----------------------------------------------------

    static float Clamp(float v, float lo, float hi)
    {
        return v < lo ? lo : (v > hi ? hi : v);
    }
    static float DbToLin(float db) { return std::pow(10.f, db / 20.f); }

    /** One-pole smoothing: move `x` a fraction `coef` of the way to `target`. */
    static void Smooth(float& x, float target, float coef)
    {
        x += coef * (target - x);
    }

    /** Ramp from `from` to `to` over `n` samples (`invN` = 1/n). */
    static Ramp MakeRamp(float from, float to, float invN)
    {
        return {from, (to - from) * invN};
    }

    /** MEM: store `x` in the slot for write position `w`. `N` is a power of
     *  two, so `& (N - 1)` wraps the ever-growing counter into the buffer. */
    template <uint32_t N>
    static inline void DelayWrite(float (&buf)[N], uint32_t w, float x)
    {
        static_assert((N & (N - 1)) == 0, "delay buffers must be power-of-two");
        buf[w & (N - 1)] = x;
    }

    /** MEM: linear-interpolated read `delay` samples behind write position `w`.
     *
     *  `w - i` is unsigned, so it wraps correctly past 0 and the mask folds it
     *  into the buffer. A fractional delay lets delay times move continuously
     *  (Size sweeps, chorus) instead of jumping whole samples.
     *
     *  Ordering rule: DelayWrite slot `w` BEFORE reading with delay < 1;
     *  reading before writing is only valid when delay >= 1 (else you'd read
     *  stale data). */
    template <uint32_t N>
    static inline float DelayRead(const float (&buf)[N], uint32_t w, float delay)
    {
        const uint32_t whole = static_cast<uint32_t>(delay);
        const float    frac  = delay - static_cast<float>(whole);
        const float    newer = buf[(w - whole) & (N - 1)];
        const float    older = buf[(w - whole - 1u) & (N - 1)];
        return newer + frac * (older - newer);
    }

    // -----------------------------------------------------------------------
    // Per-block setup
    // -----------------------------------------------------------------------

    /** Move smoothed parameters toward their targets and derive coefficients.
     *
     *  OPT: smoothing, pow(), tan() and exp() run once per block (16 samples),
     *  not per sample. Values that move delay lengths (size, pre-delay, width)
     *  and the bypass fade are then *ramped* linearly across the block, so
     *  delay times change continuously with no steps at block edges (no
     *  clicks). Other values change in small per-block steps, which is
     *  inaudible. */
    Block PrepareBlock(size_t n)
    {
        // OPT: the smoothing coefficients depend only on block size; cache.
        if(n != coefBlockSize_)
        {
            const float nn  = static_cast<float>(n);
            fastCoef_       = 1.f - std::exp(-nn / (fs_ * kFastTau));
            slowCoef_       = 1.f - std::exp(-nn / (fs_ * kSlowTau));
            coefBlockSize_  = n;
        }

        const Smoothed start = cur_; // values at the start of this block
        SmoothTowardTargets();

        Block blk;

        // Ramps run from the block-start value to the new smoothed value.
        const float invN = 1.f / static_cast<float>(n);
        blk.size      = MakeRamp(start.size, cur_.size, invN);
        blk.predelay  = MakeRamp(start.predelay, cur_.predelay, invN);
        blk.width     = MakeRamp(start.width, cur_.width, invN);
        blk.inputGain = MakeRamp(start.inputGain, cur_.inputGain, invN);

        // Spread the total loop gain evenly over the ring: 16 stages of
        // fb^(1/16) multiply to `loopGain`, so one knob sets the decay.
        blk.branchGain = std::pow(cur_.loopGain, 1.f / kBranches);

        // HF shelf (cut case): V0 is the linear shelf gain, shelfCoef the
        // allpass coefficient that places the corner at shelfHz.
        const float V0 = std::pow(10.f, cur_.shelfDb / 20.f);
        const float tw = std::tan(3.14159265f * cur_.shelfHz / fs_);
        blk.shelfH0    = V0 - 1.f;
        blk.shelfCoef  = (1.f - tw * V0) / (1.f + tw * V0);

        blk.allpassGain = cur_.allpassGain;

        // OPT: fold the level controls (and the x2 ER makeup gain) into the
        // tap gains here, so the sample loop does one multiply per tap.
        for(int i = 0; i < kBranches; i++)
        {
            blk.earlyGainL[i] = 2.f * cur_.earlyGain * kEarlyGainL[i];
            blk.earlyGainR[i] = 2.f * cur_.earlyGain * kEarlyGainR[i];
            blk.lateGainL[i]  = cur_.lateGain * kLateGainL[i];
            blk.lateGainR[i]  = cur_.lateGain * kLateGainR[i];
        }

        blk.lfoStep           = kTwoPi * chorusRateHz_ / fs_;
        blk.chorusMsToSamples = fs_ * 0.001f;
        return blk;
    }

    /** One smoothing step of every parameter toward its target. Anything that
     *  moves a delay length uses the slower constant, which keeps the
     *  resulting pitch glide gentle. */
    void SmoothTowardTargets()
    {
        Smooth(cur_.size, t_.size, slowCoef_);
        Smooth(cur_.predelay, t_.predelay, slowCoef_);
        Smooth(cur_.width, t_.width, slowCoef_);

        Smooth(cur_.inputGain, t_.active ? 1.f : 0.f, fastCoef_);
        Smooth(cur_.loopGain, t_.feedback, fastCoef_);
        Smooth(cur_.mix, t_.mix, fastCoef_);
        Smooth(cur_.allpassGain, t_.allpassGain, fastCoef_);
        Smooth(cur_.shelfHz, t_.shelfHz, fastCoef_);
        Smooth(cur_.shelfDb, t_.shelfDb, fastCoef_);
        Smooth(cur_.earlyGain, DbToLin(t_.erGainDb), fastCoef_);
        Smooth(cur_.lateGain, DbToLin(t_.diffGainDb), fastCoef_);
        Smooth(cur_.chorusLevel,
               t_.chorusOn ? t_.chorusLevel : 0.f,
               fastCoef_);
        Smooth(cur_.chorusDepthMs, t_.chorusDepth, fastCoef_);
        chorusRateHz_ = t_.chorusRate; // rate is applied directly, not smoothed
    }

    // -----------------------------------------------------------------------
    // Per-sample stages
    // -----------------------------------------------------------------------

    /** Delay the mono input before it enters the late ring. */
    inline float PreDelay(Memory& m, uint32_t w, float mono, float delay)
    {
        // MEM: write first, then read, so a delay of 0 returns `mono` itself.
        DelayWrite(m.pre, w, mono);
        return DelayRead(m.pre, w, delay);
    }

    /** 16 taps per channel, all reading the same mono history. Tap positions
     *  scale with Size; the right channel adds `width`. */
    inline Stereo EarlyReflections(Memory& m, uint32_t w, float mono, const Block& blk)
    {
        // OPT: the MATLAB kept 16 identical 4096-sample buffers per channel
        // (every row held the same input). One shared buffer gives the same
        // output with 1/32 of the memory and a single write per sample.
        DelayWrite(m.early, w, mono);

        const float size  = blk.size.value;
        const float width = blk.width.value;
        Stereo      sum   = {0.f, 0.f};
        for(int i = 0; i < kBranches; i++)
        {
            sum.l += DelayRead(m.early, w, size * kEarlyTapL[i])
                     * blk.earlyGainL[i];
            sum.r += DelayRead(m.early, w, size * (kEarlyTapR[i] + width))
                     * blk.earlyGainR[i];
        }
        return sum;
    }

    /** Short modulated delay blended into the early reflections. */
    inline Stereo Chorus(Memory& m, uint32_t w, const Block& blk, Stereo in)
    {
        lfoPhase_ += blk.lfoStep;
        if(lfoPhase_ > kTwoPi)
            lfoPhase_ -= kTwoPi;
        const float delay = (kChorusMs + cur_.chorusDepthMs * std::sin(lfoPhase_))
                            * blk.chorusMsToSamples;

        // MEM: write both channels, then read ~20 ms back (always >= 1 sample,
        // so write-then-read is safe). The interpolated read avoids zipper
        // noise from the moving delay time.
        DelayWrite(m.chorus[0], w, in.l);
        DelayWrite(m.chorus[1], w, in.r);

        const float level = cur_.chorusLevel;
        return {DelayRead(m.chorus[0], w, delay) * level + in.l * (1.f - level),
                DelayRead(m.chorus[1], w, delay) * level + in.r * (1.f - level)};
    }

    /** Shelve each branch's output from the previous sample. These are the
     *  signals that feed the next branch in the ring (damping on every pass). */
    inline void ShelveBranchOutputs(const Block& blk, float* shelved)
    {
        for(int i = 0; i < kBranches; i++)
        {
            const float x = branchOut_[i];
            const float s = x + blk.shelfCoef * shelfState_[i];
            const float allpassOut = shelfState_[i] - blk.shelfCoef * s;
            shelfState_[i]         = s;
            shelved[i]             = 0.5f * blk.shelfH0 * (x - allpassOut) + x;
        }
    }

    /** The 16-branch ring: allpass -> delay -> weighted sum to L/R. */
    inline Stereo LateField(Memory&      m,
                            uint32_t     w,
                            float        preDelayed,
                            const float* shelved,
                            const Block& blk)
    {
        const float size = blk.size.value;
        const float apg  = blk.allpassGain;
        Stereo      sum  = {0.f, 0.f};

        for(int i = 0; i < kBranches; i++)
        {
            // Branch i is fed by branch i-1; branch 0 by the last branch.
            float feedback = 0.f;
            if(i > 0)
                feedback = shelved[i - 1];
            else if(kCloseLoop)
                feedback = shelved[kBranches - 1];
            const float input = preDelayed + blk.branchGain * feedback;

            // Schroeder allpass. MEM: read the old value BEFORE writing the
            // new one. The delay (>= 223 samples) is >= 1, so the slot being
            // written this sample is never the one being read.
            auto&       apLine  = m.allpass[i];
            const float delayed = DelayRead(apLine, w, size * kAllpassDelay[i]);
            const float stored  = input + apg * delayed;
            DelayWrite(apLine, w, stored);
            const float apOut = delayed - apg * stored;

            // Plain delay on the allpass output. MEM: write, then read
            // `size * kBranchDelay` samples back (>= 159, so the sample just
            // written is not re-read).
            auto& line = m.branch[i];
            DelayWrite(line, w, apOut);
            branchOut_[i] = DelayRead(line, w, size * kBranchDelay[i]);

            sum.l += branchOut_[i] * blk.lateGainL[i];
            sum.r += branchOut_[i] * blk.lateGainR[i];
        }
        return sum;
    }

    static_assert(kMaxSize * 571 + 2 <= kLineSize, "allpass line too small");
    static_assert(kMaxSize * 3 * 179 + 2 <= kLineSize, "delay line too small");
    static_assert(kMaxSize * (525 + kMaxWidth) + 2 <= kEarlySize,
                  "early-reflection buffer too small");
    static_assert(kMaxPre + 2 <= kPreSize, "pre-delay line too small");

    Memory*  mem_ = nullptr;
    float    fs_  = kFs;
    uint32_t writePos_ = 0; // MEM: shared write position, free-running (wraps at 2^32)

    Params   t_;   // clamped targets
    Smoothed cur_; // smoothed values in use

    float  fastCoef_ = 0.f, slowCoef_ = 0.f; // smoothing coefs for this block size
    size_t coefBlockSize_ = 0;               // block size they were computed for

    float chorusRateHz_ = 1.f;
    float lfoPhase_     = 0.f; // chorus phase, radians

    float branchOut_[kBranches]   = {}; // branch outputs from the previous sample
    float shelfState_[kBranches] = {};  // HF shelf filter state per branch
};
} // namespace rbfx
