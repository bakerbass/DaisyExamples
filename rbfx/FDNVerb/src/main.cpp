/** FDNVerb
 *
 *  Pedal DevKit port of the FDN room reverb (see fdn_reverb.h).
 *
 *  Controls
 *    Pot 1  Size           (delay lengths, 1x..7x)
 *    Pot 2  Decay          (loop feedback gain, 0..0.9)
 *    Pot 3  Mix
 *    Pot 4  Damping        (HF shelf corner, 1.5k..20k; darker when low)
 *    Pot 5  Chorus depth (0..1 ms)
 *    Pot 6  Reflections <-> Tail balance (+/-9 dB tilt)
 *    Expression  
 *    SW1 (3-way) Pre-delay: 0, 300, or 2000 samples for none-short-long 
 *    FSW 1  Reverb on / off (tail rings out when turned off), LED 1 = on
 *    FSW 2  Chorus on / off on the early reflections, LED 2 blue = on
 */
#include "pedal_devkit.h"
#include "fdn_reverb.h"

using namespace daisy;
using namespace daisy::pedal_devkit;

Hardware hw;

DSY_SDRAM_BSS rbfx::FdnReverb::Memory g_mem;
rbfx::FdnReverb                       g_verb;
rbfx::FdnReverb::Params               g_params;

bool g_reverbOn = true;
bool g_chorusOn = false;


static float Deadband(float v, float& last)
{
    if(fabsf(v - last) > 0.004f)
        last = v;
    return last;
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    hw.UpdateAllControls();

    if(hw.fsw_1.RisingEdge())
        g_reverbOn = !g_reverbOn;
    if(hw.fsw_2.RisingEdge())
        g_chorusOn = !g_chorusOn;

    static float s[kNumPots] = {-1, -1, -1, -1, -1, -1};
    float        p[kNumPots];
    for(size_t i = 0; i < kNumPots; i++)
        p[i] = Deadband(hw.pot[i].Value(), s[i]);

    g_params.size       = 1.f + 6.f * p[0];
    g_params.feedback   = 0.9f * p[1];
    g_params.mix        = p[2];
    g_params.shelfHz    = 1500.f * powf(20000.f / 1500.f, p[3]);
    // g_params.predelay   = rbfx::FdnReverb::kMaxPre * p[4];
    g_params.erGainDb   = (p[5] - 0.5f) * 6.f; // 0<>1 -> -.5<>.5 -> -3<>3
    g_params.diffGainDb = -g_params.erGainDb; // -3<>3

    g_params.chorusDepth = p[4]; //hw.expression.Value();
    g_params.chorusOn    = g_chorusOn;
    g_params.active      = g_reverbOn;

    // static const float kWidth[3] = {0.f, 100.f, 200.f};
    // g_params.width               = kWidth[hw.sw1.Read()];
    static const float kPreDels[3] = {0, 300, 2000};
    g_params.predelay         = kPreDels[hw.sw1.Read()];
    g_verb.SetParams(g_params);
    g_verb.Process(in[0], in[1], out[0], out[1], size);
}

int main(void)
{
#if defined(__arm__)
    // flush denormals: decaying feedback tails otherwise slow the FPU down
    __set_FPSCR(__get_FPSCR() | (1u << 24));
#endif

    hw.Init();
    hw.seed.SetAudioBlockSize(16);
    g_verb.Init(&g_mem, hw.seed.AudioSampleRate());
    g_verb.SetParams(g_params);

    hw.seed.StartAudio(AudioCallback);

    while(true)
    {
        hw.led1.Set(g_reverbOn ? 1.f : 0.f);
        hw.led2.Set(0.f, 0.f, g_chorusOn ? 1.f : 0.f);
        hw.led1.Update();
        hw.led2.Update();
        System::Delay(10);
    }
}
