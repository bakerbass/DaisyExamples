# FDNVerb offline test suite

Measures the reverb algorithm on the PC, using the exact DSP header the
firmware builds (`../src/fdn_reverb.h`), processed in the same 16-sample blocks.
Converters and analog stages are not modelled, so compare results to a physical
rig with that in mind (the `--bits 24` option adds ideal 24-bit I/O quantization).

## Run

```powershell
.\build.ps1                      # once: builds bin/fdn_render.exe (needs MSYS2 mingw64 g++)
python analyze.py                # all presets + RT60 grid (~1.5 min)
python analyze.py --quick        # skip the grid
python analyze.py --preset large_hall
python analyze.py --bits 24
python analyze.py --compare out/metrics_before.json   # diff against an earlier run
python analyze.py --selftest     # validate the analysis code against known systems
```

Results: `out/report.md` (open this), `out/metrics.json`, `out/*.png`.
Keep a copy of `metrics.json` before changing the DSP, then `--compare`.

## What is measured

| group | metrics |
|---|---|
| impulse response | RT60 (T20/T30), EDT, C50/C80, D50, centre time, first arrival, echo density, per-octave RT60, 1/3-octave response (coloration, tilt), IACC early/late |
| distortion | THD and THD+N vs level (1 kHz) and vs frequency (stepped sine, coherent FFT); THD vs frequency from an exponential sweep (Farina) with the analysis floor drawn |
| noise | idle noise, numeric SNR, AES17-style dynamic range (unweighted), wet-path gain, input level that clips |
| stability | NaN / limit-cycle check on a long max-feedback decay |
| smoothness | click index while Size, pre-delay and Feedback are ramped |
| cost | host ns/sample (only for comparing code revisions on one machine) |

## Using `fdn_render` directly

```powershell
bin\fdn_render.exe guitar_di.wav wet.wav mix=0.4 size=4 feedback=0.5 tail=5
bin\fdn_render.exe in.wav out.wav ramp=size:1:7        # sweep a parameter over the file
```

Input must be 48 kHz mono/stereo WAV. See the header of `render.cpp` for all keys.

## Reading the numbers

- With float32 processing the DSP is effectively linear: THD sits at the numeric
  floor (about -130 to -175 dB). Real distortion on the pedal will come from the
  converters and analog stages, which is what the physical rig measures.
- Stepped-sine results need the tone to reach steady state. Very long decays
  (e.g. size 7, feedback 0.9, RT60 around a minute) never settle within the
  90 s cap and are reported as n/a.
- The sweep THD plot includes a dashed analysis floor measured on a perfectly
  linear system; sweep results near it are not distortion.
