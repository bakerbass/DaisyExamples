/** fdn_render - run a WAV file through the firmware's FdnReverb on the PC.
 *
 *  usage: fdn_render in.wav out.wav [key=value ...]
 *
 *  The DSP is the exact header the firmware builds (../src/fdn_reverb.h),
 *  processed in the same block size, so offline results match the pedal's
 *  algorithm. Only the converters and analog stages are missing.
 *
 *  Keys
 *    Any FdnReverb::Params field: size predelay feedback allpassGain mix
 *      shelfHz shelfDb erGainDb diffGainDb width chorusRate chorusDepth
 *      chorusLevel chorusOn(0/1) active(0/1)
 *    block=16     samples per Process() call (firmware uses 16)
 *    settle=1.0   seconds of silence run first so parameter smoothing settles
 *    tail=0       seconds of silence appended to capture the reverb tail
 *    ramp=NAME:FROM:TO   sweep a float param linearly over the input (repeatable)
 *    repeat=1     re-run N times and report the fastest wall time
 *
 *  Input: PCM16/24/32 or float32 WAV, 48 kHz, mono or stereo.
 *  Output: stereo float32 WAV. Stdout: one JSON line with timing and peak.
 */
#include "../src/fdn_reverb.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using rbfx::FdnReverb;

static FdnReverb::Memory g_mem; // ~0.55 MB, too big for the stack
static FdnReverb         g_verb;

// ---------------------------------------------------------------------------
// Minimal WAV I/O
// ---------------------------------------------------------------------------

struct Wav
{
    int                channels = 0;
    uint32_t           rate     = 0;
    std::vector<float> l, r; // r == l for mono input
};

static uint32_t Le32(const uint8_t* p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static uint16_t Le16(const uint8_t* p) { return p[0] | p[1] << 8; }

static bool ReadWav(const char* path, Wav& w)
{
    FILE* f = std::fopen(path, "rb");
    if(!f)
        return false;
    std::vector<uint8_t> d;
    uint8_t              buf[65536];
    size_t               got;
    while((got = std::fread(buf, 1, sizeof buf, f)) > 0)
        d.insert(d.end(), buf, buf + got);
    std::fclose(f);

    if(d.size() < 12 || std::memcmp(d.data(), "RIFF", 4) || std::memcmp(d.data() + 8, "WAVE", 4))
        return false;

    uint16_t fmtTag = 0, bits = 0;
    size_t   dataOff = 0, dataLen = 0;
    for(size_t p = 12; p + 8 <= d.size();)
    {
        const uint32_t len = Le32(&d[p + 4]);
        if(!std::memcmp(&d[p], "fmt ", 4))
        {
            fmtTag     = Le16(&d[p + 8]);
            w.channels = Le16(&d[p + 10]);
            w.rate     = Le32(&d[p + 12]);
            bits       = Le16(&d[p + 22]);
            if(fmtTag == 0xFFFE && len >= 26)
                fmtTag = Le16(&d[p + 32]); // WAVE_FORMAT_EXTENSIBLE sub-format
        }
        else if(!std::memcmp(&d[p], "data", 4))
        {
            dataOff = p + 8;
            dataLen = std::min<size_t>(len, d.size() - dataOff);
            break;
        }
        p += 8 + len + (len & 1);
    }
    if(!dataOff || w.channels < 1 || (fmtTag != 1 && fmtTag != 3))
        return false;

    const int    bytes  = bits / 8;
    const size_t frames = dataLen / (bytes * w.channels);
    w.l.resize(frames);
    w.r.resize(frames);
    for(size_t i = 0; i < frames; i++)
    {
        float ch[2];
        for(int c = 0; c < 2; c++)
        {
            const uint8_t* s = &d[dataOff + (i * w.channels + (c < w.channels ? c : 0)) * bytes];
            if(fmtTag == 3 && bits == 32)
            {
                std::memcpy(&ch[c], s, 4);
            }
            else if(bits == 16)
                ch[c] = (int16_t)Le16(s) / 32768.f;
            else if(bits == 24)
                ch[c] = ((int32_t)(s[0] << 8 | s[1] << 16 | (uint32_t)s[2] << 24) >> 8) / 8388608.f;
            else if(bits == 32)
                ch[c] = (int32_t)Le32(s) / 2147483648.f;
            else
                return false;
        }
        w.l[i] = ch[0];
        w.r[i] = ch[1];
    }
    return true;
}

static void WriteWav(const char* path, const std::vector<float>& l, const std::vector<float>& r, uint32_t rate)
{
    FILE*          f      = std::fopen(path, "wb");
    const uint32_t frames = (uint32_t)l.size();
    const uint32_t dataB  = frames * 8;
    auto           u32    = [&](uint32_t v) { std::fwrite(&v, 4, 1, f); };
    auto           u16    = [&](uint16_t v) { std::fwrite(&v, 2, 1, f); };
    std::fwrite("RIFF", 1, 4, f); u32(36 + dataB); std::fwrite("WAVE", 1, 4, f);
    std::fwrite("fmt ", 1, 4, f); u32(16); u16(3); u16(2); u32(rate); u32(rate * 8); u16(8); u16(32);
    std::fwrite("data", 1, 4, f); u32(dataB);
    for(uint32_t i = 0; i < frames; i++)
    {
        std::fwrite(&l[i], 4, 1, f);
        std::fwrite(&r[i], 4, 1, f);
    }
    std::fclose(f);
}

// ---------------------------------------------------------------------------
// Parameters
// ---------------------------------------------------------------------------

struct FloatField
{
    const char* name;
    float FdnReverb::Params::*ptr;
};
static const FloatField kFields[] = {
    {"size", &FdnReverb::Params::size},
    {"predelay", &FdnReverb::Params::predelay},
    {"feedback", &FdnReverb::Params::feedback},
    {"allpassGain", &FdnReverb::Params::allpassGain},
    {"mix", &FdnReverb::Params::mix},
    {"shelfHz", &FdnReverb::Params::shelfHz},
    {"shelfDb", &FdnReverb::Params::shelfDb},
    {"erGainDb", &FdnReverb::Params::erGainDb},
    {"diffGainDb", &FdnReverb::Params::diffGainDb},
    {"width", &FdnReverb::Params::width},
    {"chorusRate", &FdnReverb::Params::chorusRate},
    {"chorusDepth", &FdnReverb::Params::chorusDepth},
    {"chorusLevel", &FdnReverb::Params::chorusLevel},
};

struct Ramp
{
    const FloatField* field;
    float             from, to;
};

static const FloatField* FindField(const std::string& name)
{
    for(const auto& f : kFields)
        if(name == f.name)
            return &f;
    return nullptr;
}

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::fprintf(stderr, "usage: fdn_render in.wav out.wav [key=value ...]\n");
        return 2;
    }

    FdnReverb::Params  params;
    std::vector<Ramp>  ramps;
    size_t             block  = 16;
    float              settle = 1.f, tail = 0.f;
    int                repeat = 1;

    for(int i = 3; i < argc; i++)
    {
        const std::string arg = argv[i];
        const size_t      eq  = arg.find('=');
        if(eq == std::string::npos)
        {
            std::fprintf(stderr, "bad argument '%s' (want key=value)\n", argv[i]);
            return 2;
        }
        const std::string key = arg.substr(0, eq), val = arg.substr(eq + 1);

        if(key == "block")       block = (size_t)std::atoi(val.c_str());
        else if(key == "settle") settle = (float)std::atof(val.c_str());
        else if(key == "tail")   tail = (float)std::atof(val.c_str());
        else if(key == "repeat") repeat = std::atoi(val.c_str());
        else if(key == "chorusOn") params.chorusOn = std::atoi(val.c_str()) != 0;
        else if(key == "active")   params.active = std::atoi(val.c_str()) != 0;
        else if(key == "ramp")
        {
            const size_t a = val.find(':'), b = val.rfind(':');
            const FloatField* f = a == std::string::npos ? nullptr : FindField(val.substr(0, a));
            if(!f || a == b)
            {
                std::fprintf(stderr, "bad ramp '%s' (want NAME:FROM:TO)\n", val.c_str());
                return 2;
            }
            ramps.push_back({f, (float)std::atof(val.substr(a + 1, b - a - 1).c_str()),
                             (float)std::atof(val.substr(b + 1).c_str())});
        }
        else if(const FloatField* f = FindField(key))
            params.*(f->ptr) = (float)std::atof(val.c_str());
        else
        {
            std::fprintf(stderr, "unknown key '%s'\n", key.c_str());
            return 2;
        }
    }

    Wav in;
    if(!ReadWav(argv[1], in))
    {
        std::fprintf(stderr, "cannot read '%s' (need PCM or float WAV)\n", argv[1]);
        return 1;
    }
    if(in.rate != 48000)
    {
        std::fprintf(stderr, "input must be 48000 Hz (got %u): the delay tables assume 48 kHz\n", in.rate);
        return 1;
    }

    // Pad input (plus tail) to whole blocks so every Process() call is full-size,
    // exactly as on the pedal.
    const size_t inLen  = in.l.size();
    size_t       total  = inLen + (size_t)(tail * 48000.f);
    total               = (total + block - 1) / block * block;
    in.l.resize(total, 0.f);
    in.r.resize(total, 0.f);
    std::vector<float> outL(total), outR(total);

    double bestSec = 1e30;
    for(int rep = 0; rep < repeat; rep++)
    {
        FdnReverb::Params p = params;
        for(const auto& r : ramps)
            p.*(r.field->ptr) = r.from;
        g_verb.SetParams(p);
        g_verb.Init(&g_mem, 48000.f); // Init snaps smoothed values to the targets
        g_verb.SetParams(p);

        // Run silence so any parameter smoothing and the LFO are in steady state.
        std::vector<float> zeros(block, 0.f), sink(block);
        for(size_t n = 0; n < (size_t)(settle * 48000.f) / block; n++)
            g_verb.Process(zeros.data(), zeros.data(), sink.data(), sink.data(), block);

        const auto t0 = std::chrono::steady_clock::now();
        for(size_t pos = 0; pos < total; pos += block)
        {
            if(!ramps.empty())
            {
                const float frac = inLen > 0 ? std::fmin(1.f, (float)pos / (float)inLen) : 0.f;
                for(const auto& r : ramps)
                    p.*(r.field->ptr) = r.from + (r.to - r.from) * frac;
                g_verb.SetParams(p);
            }
            g_verb.Process(&in.l[pos], &in.r[pos], &outL[pos], &outR[pos], block);
        }
        const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if(sec < bestSec)
            bestSec = sec;
    }

    float peak = 0.f;
    for(size_t i = 0; i < total; i++)
        peak = std::fmax(peak, std::fmax(std::fabs(outL[i]), std::fabs(outR[i])));

    WriteWav(argv[2], outL, outR, 48000);
    std::printf("{\"samples\": %zu, \"peak\": %.9g, \"ns_per_sample\": %.2f}\n",
                total, peak, bestSec / (double)total * 1e9);
    return 0;
}
