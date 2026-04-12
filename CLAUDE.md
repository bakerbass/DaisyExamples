# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## About This Repository

This is the **Daisy Examples** repository — a collection of embedded audio firmware examples for the [Daisy Audio Platform](https://daisy.audio). It contains examples for multiple hardware variants (Seed, Pod, Patch, Petal, Field, Versio, Legio, Patch_SM) plus two core submodule libraries.

## Build Commands

```bash
# Build both libraries (must be done before building any example)
./ci/build_libs.sh

# Build a single example
cd seed/Blink && make

# Flash to device via DFU bootloader (hold BOOT, plug USB)
cd seed/Blink && make program-dfu

# Flash via JTAG/SWD (requires STLink)
cd seed/Blink && make program

# Build all examples (same as CI)
python ./ci/build_examples.py

# Build everything (libs + examples)
./rebuild_all.sh

# Generate distributable .bin/.hex files
python ./ci/build_dist.py -ru
```

## Code Style

- **Formatter**: clang-format v10 (Allman brace style, 4-space indent, 80-column limit)
- **Check locally**: `./ci/local_style_check.sh` (excludes submodules)
- **Auto-fix**: GitHub Actions runs `fix_style.yml` on PR/push to master

## Project Management

`helper.py` is a Python utility for creating and managing projects:

```bash
# Create new project for a specific board
python helper.py create pod/MyNewProject --board pod

# Copy an existing project
python helper.py copy pod/MyNewProject --source patch/SomeExisting

# Update debug resources (.vscode, VisualGDB configs) for all examples
python helper.py update
```

**Supported boards**: `seed`, `pod`, `patch`, `field`, `petal`, `versio`, `legio`, `patch_sm`

## Architecture

### Library Submodules

- **`libDaisy/`** — Hardware abstraction layer (HAL) for all Daisy hardware variants
- **`DaisySP/`** — DSP component library (oscillators, filters, effects, envelopes, etc.)

Both must be built before compiling any example (`./ci/build_libs.sh`).

### Example Structure

Each example lives in `<board>/<ExampleName>/` and has a minimal `Makefile` that defines `TARGET` and `CPP_SOURCES`, then includes libDaisy's generic Makefile:

```makefile
TARGET = MyProject
CPP_SOURCES = MyProject.cpp
LIBDAISY_DIR = ../../libDaisy
DAISYSP_DIR = ../../DaisySP
include $(LIBDAISY_DIR)/core/Makefile
```

### Firmware Pattern

All examples follow this structure:

```cpp
#include "daisy_seed.h"  // or daisy_pod.h, daisy_patch.h, etc.
using namespace daisy;

DaisySeed hw;  // Board-specific hardware object

void AudioCallback(AudioHandle::InterleavingInputBuffer in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t size) {
    // Real-time audio — runs in interrupt context at ~48kHz
    // All per-sample DSP processing happens here
}

int main() {
    hw.Configure();
    hw.Init();
    hw.StartAudio(AudioCallback);
    while(1) { hw.DelayMs(1); }  // Main loop handles UI/control at low rate
}
```

### DSP Module Pattern

All DaisySP modules share the same interface:

```cpp
Oscillator osc;
osc.Init(sample_rate);      // Called once in main()
osc.SetFreq(440.f);
float out = osc.Process();  // Called per sample inside AudioCallback
```

### Hardware API Levels

- **High-level**: Use board-specific class (`DaisyPod`, `DaisyPatch`, etc.) — pre-configures all hardware
- **Low-level**: Use individual peripheral classes from `daisy.h` — full manual control

### Key libDaisy Source Organization

- `per/` — Peripherals (ADC, DAC, GPIO, I2C, SPI, SAI audio)
- `dev/` — External device drivers (codecs, OLED, SD card)
- `hid/` — Human interfaces (encoders, switches, LEDs, MIDI, USB)
- `sys/` — System (clocks, DMA, FatFS)

## Important Constraints

- **Audio callback runs in interrupt context** — no heap allocation, no blocking calls, minimal latency
- **STM32H750 has ~512KB SRAM** — prefer static allocation for buffers and DSP state; avoid dynamic allocation in audio paths
- **C++14**, `-O2`, no exceptions, ARM Cortex-M7 with FPU (`arm-none-eabi-g++`)

## Submodule Management

```bash
# Clone with submodules
git clone --recursive <repo-url>

# Update submodules to latest
git submodule update --remote
git pull --recurse-submodules
```

Submodules: `libDaisy`, `DaisySP`, and `stmlib` (pichenettes/stmlib).

## CI/CD

GitHub Actions (`.github/workflows/`) runs on push/PR to `master`:
1. Install `arm-none-eabi-gcc` v9-2019-q4
2. Build libraries
3. Build all examples
4. Generate and auto-commit distributable binaries to the `dist/` folder
