/** @NAME@
 *
 *  Pedal DevKit starter: DaisySP overdrive with a true-ish bypass footswitch.
 *  - Pot 1: drive, Pot 2: output level
 *  - FSW 1: effect on/off (LED 1 shows state)
 */
#include "pedal_devkit.h"
#include "daisysp.h"

using namespace daisy;
using namespace daisy::pedal_devkit;
using namespace daisysp;

Hardware  hw;
Overdrive drive;
bool      effect_on = true;

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    hw.UpdateAllControls();

    if(hw.fsw_1.RisingEdge())
        effect_on = !effect_on;

    drive.SetDrive(hw.pot[0].Value());
    const float level = hw.pot[1].Value();

    for(size_t i = 0; i < size; i++)
    {
        const float dry = in[0][i];
        const float wet = effect_on ? drive.Process(dry) * level : dry;
        out[0][i]       = wet;
        out[1][i]       = wet;
    }
}

int main(void)
{
    hw.Init();
    drive.Init();

    hw.seed.StartAudio(AudioCallback);

    while(true)
    {
        hw.led1.Set(effect_on ? 1.f : 0.f);
        hw.led1.Update();
        System::Delay(10);
    }
}
