#!/usr/bin/env python
"""Offline measurement suite for FdnReverb.

Renders test signals through the firmware DSP (bin/fdn_render.exe) and measures:

  impulse response   RT60 (T20/T30), EDT, C50/C80, D50, centre time, echo
                     density, per-octave RT60, 1/3-octave response, IACC
  distortion         THD / THD+N vs level and frequency (stepped sine), THD vs
                     frequency from an exponential sweep (Farina)
  noise              idle noise, numeric SNR, AES17-style dynamic range,
                     headroom, wet-path gain
  stability          NaN / limit-cycle check on a long decay
  smoothness         click metric while Size / pre-delay / decay are ramped
  cost               host ns/sample (relative only - not Cortex-M7 cycles)

Usage (from this folder, after build.ps1):
  python analyze.py                      all presets, full suite
  python analyze.py --quick              skip the parameter grid
  python analyze.py --preset large_hall  one preset
  python analyze.py --bits 24            quantize I/O to 24 bits (codec-like)
  python analyze.py --compare out/old.json
  python analyze.py --selftest           validate the analysis code only

Results land in out/: report.md, metrics.json, *.png
"""
import argparse
import json
import math
import subprocess
import sys
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal
from scipy.io import wavfile
from scipy.ndimage import uniform_filter1d

FS = 48000
HERE = Path(__file__).resolve().parent
EXE = HERE / "bin" / "fdn_render.exe"
OUT = HERE / "out"

# Presets are FdnReverb::Params overrides. Anything not listed is the firmware
# default (size 3, feedback 0.2, mix 0.5, ...).
PRESETS = {
    "default": {},
    "small_room": dict(size=1.5, feedback=0.3, predelay=300),
    "large_hall": dict(size=5, feedback=0.75, predelay=1200, shelfHz=6000),
    "max_decay": dict(size=7, feedback=0.9),
}

OCTAVES = [125, 250, 500, 1000, 2000, 4000, 8000]
BITS = None  # set by --bits


def db(x, floor=1e-300):  # floor -> about -6000 dB
    return 20 * np.log10(np.maximum(np.abs(x), floor))


def rms(x):
    return float(np.sqrt(np.mean(np.square(x, dtype=np.float64))))


# ---------------------------------------------------------------------------
# Running the firmware DSP
# ---------------------------------------------------------------------------
def quantize(x, bits):
    """Round to a `bits`-bit grid (what an ideal converter would keep)."""
    q = 2.0 ** (bits - 1)
    return np.round(x * q) / q


def render(x, tail=0.0, settle=1.0, repeat=1, **params):
    """Run `x` (mono (n,) or stereo (n,2)) through the DSP.

    Returns (out (n,2) float64, info dict). bool params become 0/1; a `ramp`
    param may be a string or list of "name:from:to".
    """
    x = np.asarray(x, dtype=np.float64)
    if BITS:
        x = quantize(x, BITS)
    with tempfile.TemporaryDirectory() as td:
        fin, fout = Path(td) / "in.wav", Path(td) / "out.wav"
        wavfile.write(fin, FS, x.astype(np.float32))
        cmd = [str(EXE), str(fin), str(fout), f"tail={tail}", f"settle={settle}",
               f"repeat={repeat}"]
        for k, v in params.items():
            if k == "ramp":
                cmd += [f"ramp={r}" for r in ([v] if isinstance(v, str) else v)]
            else:
                cmd.append(f"{k}={int(v) if isinstance(v, bool) else v}")
        res = subprocess.run(cmd, capture_output=True, text=True)
        if res.returncode:
            raise RuntimeError(f"fdn_render failed: {res.stderr.strip()}")
        _, y = wavfile.read(fout)
    # The tool pads to whole blocks; trim back to input + tail.
    y = y.astype(np.float64)[:len(x) + int(tail * FS)]
    if BITS:
        y = quantize(y, BITS)
    return y, json.loads(res.stdout)


# ---------------------------------------------------------------------------
# Test signals
# ---------------------------------------------------------------------------
def impulse(amp=0.5, n=256):
    x = np.zeros(n)
    x[0] = amp
    return x


def pink_noise(seconds, rms_dbfs, seed=1):
    n = int(seconds * FS)
    spec = np.fft.rfft(np.random.default_rng(seed).standard_normal(n))
    f = np.fft.rfftfreq(n, 1 / FS)
    spec[1:] /= np.sqrt(f[1:])
    spec[0] = 0
    x = np.fft.irfft(spec, n)
    return x / rms(x) * 10 ** (rms_dbfs / 20)


def sine(freq, level_dbfs, n):
    return 10 ** (level_dbfs / 20) * np.sin(2 * np.pi * freq * np.arange(n) / FS)


def ess(T=10.0, f1=20.0, f2=20000.0, amp=0.25):
    """Exponential sine sweep and its inverse filter (Farina)."""
    n = int(T * FS)
    t = np.arange(n) / FS
    L = T / np.log(f2 / f1)
    x = amp * np.sin(2 * np.pi * f1 * L * (np.exp(t / L) - 1))
    taper = int(0.005 * FS)
    w = np.ones(n)
    w[:taper] = np.hanning(2 * taper)[:taper]
    w[-taper:] = np.hanning(2 * taper)[taper:]
    x *= w
    inv = x[::-1] / amp * np.exp(-t / L)
    return x, inv, L


# ---------------------------------------------------------------------------
# Impulse-response metrics
# ---------------------------------------------------------------------------
def edc_db(h):
    e = np.cumsum(np.square(h)[::-1])[::-1]
    return 10 * np.log10(np.maximum(e / e[0], 1e-300))


def fit_rt(edc, lo, hi, scale):
    """Slope of the EDC between `lo` and `hi` dB, extrapolated to 60 dB."""
    i0 = np.argmax(edc <= lo)
    i1 = np.argmax(edc <= hi)
    if i1 <= i0 + 10 or edc[i1] > hi:
        return float("nan")
    t = np.arange(i0, i1) / FS
    slope = np.polyfit(t, edc[i0:i1], 1)[0]
    return -60.0 / slope if slope < 0 else float("nan")


def rt_set(h):
    edc = edc_db(h)
    return {
        "t20": fit_rt(edc, -5, -25, 3),
        "t30": fit_rt(edc, -5, -35, 2),
        "edt": fit_rt(edc, 0, -10, 6),
    }, edc


def onset_index(h2):
    """First sample above 5 % of the combined-channel peak."""
    return int(np.argmax(np.abs(h2).max(axis=1) > 0.05 * np.abs(h2).max()))


def echo_density(h, win=0.02, hop=0.005, span=4.0):
    """Normalised echo density (Abel & Huang): 1.0 means Gaussian-noise-like."""
    n = int(win * FS)
    w = np.hanning(n)
    w /= w.sum()
    norm = math.erfc(1 / math.sqrt(2))
    times, vals = [], []
    for s in range(0, min(len(h) - n, int(span * FS)), int(hop * FS)):
        seg = h[s:s + n]
        sigma = math.sqrt(np.sum(w * seg ** 2))
        times.append((s + n / 2) / FS)
        vals.append(np.sum(w * (np.abs(seg) > sigma)) / norm if sigma > 0 else 0.0)
    return np.array(times), np.array(vals)


def third_octave_response(h, fmin=50, fmax=16000):
    n = 1 << int(np.ceil(np.log2(len(h))))
    p = np.abs(np.fft.rfft(h, n)) ** 2
    f = np.fft.rfftfreq(n, 1 / FS)
    centers = 1000 * 2.0 ** (np.arange(-13, 14) / 3)
    centers = centers[(centers >= fmin) & (centers <= fmax)]
    levels = []
    for c in centers:
        m = (f >= c / 2 ** (1 / 6)) & (f < c * 2 ** (1 / 6))
        levels.append(10 * np.log10(p[m].mean() + 1e-300))
    return centers, np.array(levels)


def iacc(l, r, max_lag_ms=1.0):
    lag = int(max_lag_ms * 1e-3 * FS)
    den = math.sqrt(np.sum(l ** 2) * np.sum(r ** 2))
    if den == 0:
        return float("nan")
    full = signal.correlate(l, r, mode="full", method="fft")
    mid = len(l) - 1
    return float(np.max(np.abs(full[mid - lag:mid + lag + 1])) / den)


def analyze_ir(h2):
    """h2: (n,2) impulse response. Returns (metrics, helper dict for plots)."""
    hm = h2.mean(axis=1)
    rt, edc = rt_set(hm)
    onset = onset_index(h2)
    e = np.sum(h2 ** 2, axis=1)
    n50, n80 = int(0.05 * FS), int(0.08 * FS)
    tot = e[onset:].sum()
    c50 = 10 * np.log10(e[onset:onset + n50].sum() / max(e[onset + n50:].sum(), 1e-300))
    c80 = 10 * np.log10(e[onset:onset + n80].sum() / max(e[onset + n80:].sum(), 1e-300))
    t = np.arange(len(e) - onset) / FS
    ts = float(np.sum(t * e[onset:]) / tot)

    et, ev = echo_density(h2[onset:, 0])
    dense = et[np.argmax(ev >= 0.99)] * 1000 if np.any(ev >= 0.99) else float("nan")

    band_rt = {}
    for fc in OCTAVES:
        sos = signal.butter(4, [fc / math.sqrt(2), min(fc * math.sqrt(2), FS * 0.45)],
                            btype="band", fs=FS, output="sos")
        band_rt[fc] = fit_rt(edc_db(signal.sosfilt(sos, hm)), -5, -25, 3)

    fc3, lev_l = third_octave_response(h2[:, 0])
    _, lev_r = third_octave_response(h2[:, 1])
    mid = (fc3 >= 100) & (fc3 <= 10000)
    lev = 0.5 * (lev_l + lev_r)
    low, high = (fc3 >= 125) & (fc3 <= 250), (fc3 >= 4000) & (fc3 <= 8000)

    a, b = onset, onset + n80
    metrics = {
        "rt60_t20_s": rt["t20"], "rt60_t30_s": rt["t30"], "edt_s": rt["edt"],
        "c50_db": float(c50), "c80_db": float(c80),
        "d50": float(e[onset:onset + n50].sum() / tot),
        "centre_time_ms": ts * 1000,
        "first_arrival_ms": onset / FS * 1000,
        "echo_density_unity_ms": float(dense),
        "rt60_octave_s": {str(k): v for k, v in band_rt.items()},
        "response_coloration_db": float(np.std(lev[mid])),
        "response_tilt_db": float(lev[high].mean() - lev[low].mean()),
        "iacc_early": iacc(h2[a:b, 0], h2[a:b, 1]),
        "iacc_late": iacc(h2[b:b + 2 * FS, 0], h2[b:b + 2 * FS, 1]),
        "peak_ir": float(np.abs(h2).max()),
        "tail_incomplete": tail_incomplete(h2),
    }
    aux = dict(edc=edc, onset=onset, echo=(et, ev), band_rt=band_rt, fc3=fc3,
               lev_l=lev_l, lev_r=lev_r, rt=rt)
    return metrics, aux


# ---------------------------------------------------------------------------
# Distortion / noise
# ---------------------------------------------------------------------------
def thd_fft(seg, k):
    """seg holds an exact integer number of cycles (bin k of N). Returns
    (thd_db, thdn_db) relative to the fundamental, worst of the channels."""
    n = len(seg)
    spec = np.abs(np.fft.rfft(seg)) / (n / 2)
    fund = spec[k]
    harm = [spec[m * k] for m in range(2, 11) if m * k < len(spec)]
    total = np.sqrt(np.sum(spec[1:] ** 2) - fund ** 2 + 1e-300)
    return db(np.sqrt(np.sum(np.square(harm))) / fund), db(total / fund)


def stepped_sine(freq, level_dbfs, params, settle_s, n=1 << 16):
    """Coherent steady-state sine measurement.

    Returns (thd_db, thdn_db, out_dbfs, steady_err_db). steady_err_db compares
    the last window with the one before it: if the reverb is still settling they
    differ, and the THD+N figure is then an upper bound only."""
    k = max(1, round(freq * n / FS))
    f = k * FS / n
    total = int(settle_s * FS) + 2 * n
    x = sine(f, level_dbfs, total)
    y, _ = render(x, settle=0.5, **params)
    seg, prev = y[-n:], y[-2 * n:-n]
    worst = max((thd_fft(seg[:, c], k) for c in range(2)), key=lambda r: r[1])
    err = db(rms(seg - prev) / rms(seg))
    return worst[0], worst[1], db(rms(seg) * math.sqrt(2)), err


def thd_from_ess(h_full, n_sweep, L, f1=20.0, f2=20000.0):
    """THD vs frequency from the deconvolved sweep. `h_full` is the full
    deconvolution output (linear IR starts at index n_sweep-1)."""
    nfft = 1 << 18
    c1 = n_sweep - 1
    centers = [c1 - int(round(L * math.log(m) * FS)) for m in range(1, 7)]
    lin = h_full[c1:c1 + nfft]
    lin_spec = np.abs(np.fft.rfft(lin, nfft)) ** 2
    f = np.fft.rfftfreq(nfft, 1 / FS)

    def band_energy(spec, fc):
        # Mean energy per bin (a spectral density), NOT a sum: a 1/3-octave
        # band holds m times more bins at the m-th harmonic.
        m = (f >= fc / 2 ** (1 / 6)) & (f < fc * 2 ** (1 / 6))
        return spec[m].mean() if m.any() else 0.0

    harm_specs = {}
    for m in range(2, 6):
        lo = (centers[m - 1] + centers[m]) // 2
        hi = (centers[m - 1] + centers[m - 2]) // 2
        harm_specs[m] = np.abs(np.fft.rfft(h_full[max(lo, 0):hi], nfft)) ** 2

    fs_axis = 1000 * 2.0 ** (np.arange(-13, 14) / 3)
    fs_axis = fs_axis[(fs_axis >= 50) & (fs_axis <= 4000)]
    thd = []
    for fc in fs_axis:
        p1 = band_energy(lin_spec, fc)
        ph = sum(band_energy(harm_specs[m], m * fc) for m in range(2, 6)
                 if m * fc < FS * 0.45)
        thd.append(10 * np.log10((ph + 1e-300) / (p1 + 1e-300)))
    return fs_axis, np.array(thd)


def sweep_floor(amp_dbfs):
    """THD curve of a perfectly linear system (identity): the analysis floor."""
    x, inv, L = ess(amp=10 ** (amp_dbfs / 20))
    h = signal.fftconvolve(x, inv) / np.abs(signal.fftconvolve(x, inv)).max()
    return thd_from_ess(h, len(x), L)


def sweep_measure(params, amp_dbfs, tail=6.0):
    """Run an ESS through the DSP; return (ir (n,2) , thd axes)."""
    x, inv, L = ess(amp=10 ** (amp_dbfs / 20))
    y, _ = render(x, tail=tail, **params)
    ref = signal.fftconvolve(x, inv)
    norm = np.abs(ref).max()
    out = []
    for c in range(2):
        out.append(signal.fftconvolve(y[:, c], inv) / norm)
    h = np.stack(out, axis=1)
    n = len(x)
    ir = h[n - 1:n - 1 + int((tail + 1) * FS)]
    fax, thd = thd_from_ess(h[:, 0], n, L)
    return ir, fax, thd


# ---------------------------------------------------------------------------
# Measurement suites
# ---------------------------------------------------------------------------
def tail_incomplete(h2):
    """True if the last 0.5 s still holds more than -60 dB of the IR peak."""
    return bool(rms(h2[-FS // 2:]) > 1e-3 * np.abs(h2).max())


def measure_ir(preset, tail=20.0, max_tail=180.0):
    """Impulse response at mix = 1. The record is lengthened (x3, up to
    `max_tail` s) until the tail has decayed, so long decays are not truncated."""
    p = dict(preset, mix=1.0, chorusOn=False)
    while True:
        y, info = render(impulse(), tail=tail, **p)
        if not tail_incomplete(y) or tail >= max_tail:
            return y / 0.5, info
        tail = min(max_tail, tail * 3)


def measure_levels(preset, rt60):
    p = dict(preset)
    mix = p.get("mix", 0.5)
    out = {}
    pn = pink_noise(20, -20)
    yw, _ = render(pn, settle=0.5, **dict(preset, mix=1.0))
    seg = yw[int(8 * FS):]
    out["wet_gain_db"] = float(db(rms(seg) / rms(pn[int(8 * FS):])))
    yp, _ = render(pn, settle=0.5, **p)
    peak = float(np.abs(yp).max())
    out["mix"] = mix
    out["noise_clip_input_rms_dbfs"] = float(-20 - db(peak))
    sine_in = sine(1000, -12, int(12 * FS))
    ys, _ = render(sine_in, settle=0.5, **p)
    out["sine_clip_input_dbfs"] = float(-12 - db(np.abs(ys).max()))
    return out


def late_rt60(edc):
    """Decay rate of the deep tail (-30..-60 dB, else -20..-40), as an RT60.
    The late tail usually decays slower than T20 suggests."""
    for lo, hi in ((-30, -60), (-20, -40), (-5, -25)):
        v = fit_rt(edc, lo, hi, 1)
        if np.isfinite(v):
            return v
    return float("nan")


MAX_SETTLE_S = 90.0


def measure_distortion(preset, rt60_late, quick):
    # Run the tone until its start-up transient has fallen ~150 dB (2.5 x the
    # late RT60); otherwise "THD+N" just measures the unsettled reverb tail.
    # For very long decays the run is capped; `steady_state_err_db` then shows
    # the tone never settled and the THD+N numbers are not valid.
    r = rt60_late if np.isfinite(rt60_late) else 5.0
    settle = float(np.clip(2.5 * r, 2.0, MAX_SETTLE_S))
    levels = [-80, -60, -40, -20, -12, -6, -3, 0]
    freqs = [100, 500, 1000, 5000, 10000]
    res = {"levels": {}, "freqs": {}}
    errs = []
    for lv in levels:
        thd, thdn, _, err = stepped_sine(1000, lv, preset, settle)
        res["levels"][lv] = (thd, thdn)
        errs.append(err)
    for f in freqs:
        thd, thdn, _, err = stepped_sine(f, -12, preset, settle)
        res["freqs"][f] = (thd, thdn)
        errs.append(err)
    thdn60 = res["levels"][-60][1]
    res["snr_numeric_db"] = -res["levels"][-20][1]
    # AES17: DR = 60 dB - (THD+N ratio of a -60 dBFS tone)   (unweighted here)
    res["dynamic_range_db"] = 60 - thdn60
    res["settle_s"] = settle
    res["steady_state_err_db"] = float(max(errs))
    return res


def measure_idle(preset):
    p = dict(preset, feedback=0.9, size=7)
    burst = pink_noise(0.2, -6, seed=3)
    x = np.concatenate([burst, np.zeros(int(30 * FS))])
    y, _ = render(x, settle=0.5, **p)
    last = y[-FS:]
    zeros, _ = render(np.zeros(5 * FS), settle=0.5, **preset)
    return {
        "idle_noise_dbfs": float(db(rms(zeros))),
        "decay_final_rms_dbfs": float(db(rms(last))),
        "nan_or_inf": bool(not np.all(np.isfinite(y))),
        "decay_peak_after_burst": float(np.abs(y[int(0.3 * FS):]).max()),
    }


def click_ratio(y):
    """Impulsiveness: the largest |second difference| relative to the local RMS
    of the second difference (+/- 25 ms). A smooth signal, even a modulated
    one, stays at a few; a click is a sample far above its neighbourhood."""
    d2 = np.diff(y, 2)[FS // 2:]  # skip start-up
    local = np.sqrt(uniform_filter1d(d2 ** 2, 2401)) + 1e-300
    return float(np.max(np.abs(d2) / local))


def measure_clicks(preset):
    x = sine(1000, -20, 10 * FS)
    base = dict(preset, mix=0.5)
    res, outs = {}, {}
    for name, spec in {"size": "size:1:7", "predelay": "predelay:0:2000",
                       "feedback": "feedback:0.1:0.9"}.items():
        ramped, _ = render(x, settle=0.5, ramp=spec, **base)
        mid = {"size": 4.0, "predelay": 1000.0, "feedback": 0.5}[name]
        static, _ = render(x, settle=0.5, **dict(base, **{name: mid}))
        res[name] = {"ramped": click_ratio(ramped[:, 0]),
                     "static": click_ratio(static[:, 0])}
        res[name]["ratio"] = res[name]["ramped"] / res[name]["static"]
        outs[name] = ramped[:, 0]
    return res, outs


def measure_cost(preset):
    x = pink_noise(10, -20)
    _, info = render(x, repeat=3, **dict(preset, chorusOn=True))
    return {"host_ns_per_sample": info["ns_per_sample"]}


def parameter_grid():
    sizes, fbs = [1, 2, 3, 5, 7], [0.1, 0.3, 0.5, 0.7, 0.9]
    grid = np.full((len(fbs), len(sizes)), np.nan)
    for i, fb in enumerate(fbs):
        for j, sz in enumerate(sizes):
            y, _ = render(impulse(), tail=25, settle=0.2, size=sz, feedback=fb,
                          mix=1.0, shelfHz=20000)
            grid[i, j] = fit_rt(edc_db(y.mean(axis=1) / 0.5), -5, -25, 3)
    return sizes, fbs, grid


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------
def plot_ir(name, h2, m, aux):
    fig, ax = plt.subplots(3, 2, figsize=(13, 11))
    t = np.arange(len(h2)) / FS
    hop = 240
    env = lambda h: 10 * np.log10(np.maximum(
        np.convolve(h ** 2, np.ones(hop) / hop, "same")[::hop], 1e-20))
    for c, lbl in enumerate("LR"):
        ax[0, 0].plot(t[::hop], env(h2[:, c]), lw=0.8, label=lbl)
    ax[0, 0].set(title="IR energy envelope (5 ms)", xlabel="s", ylabel="dB",
                 ylim=(-140, 0))
    ax[0, 0].legend()

    ax[0, 1].plot(np.arange(len(aux["edc"]))[::48] / FS, aux["edc"][::48])
    for key, (lo, hi) in {"t20": (-5, -25), "t30": (-5, -35)}.items():
        v = m[f"rt60_{key}_s"]
        if np.isfinite(v):
            ax[0, 1].plot([0, v], [0, -60], "--", lw=0.8, label=f"{key.upper()} {v:.2f}s")
    ax[0, 1].set(title="Schroeder decay curve (EDC)", xlabel="s", ylabel="dB",
                 ylim=(-80, 3), xlim=(0, min(len(h2) / FS, 3 * (m["rt60_t20_s"] if np.isfinite(m["rt60_t20_s"]) else 5))))
    ax[0, 1].legend()

    f, tt, S = signal.spectrogram(h2[:, 0], FS, nperseg=1024, noverlap=768)
    S = 10 * np.log10(S + 1e-20)
    ax[1, 0].pcolormesh(tt, f, S, vmin=S.max() - 100, vmax=S.max(), shading="auto")
    ax[1, 0].set(title="Spectrogram (L)", xlabel="s", ylabel="Hz", ylim=(0, 16000),
                 xlim=(0, min(3, len(h2) / FS)))

    ax[1, 1].semilogx(aux["fc3"], aux["lev_l"] - aux["lev_l"].max(), label="L")
    ax[1, 1].semilogx(aux["fc3"], aux["lev_r"] - aux["lev_l"].max(), label="R")
    ax[1, 1].set(title="1/3-octave response (full IR)", xlabel="Hz", ylabel="dB")
    ax[1, 1].grid(True, which="both", alpha=0.3)
    ax[1, 1].legend()

    bands = list(aux["band_rt"].keys())
    ax[2, 0].bar([str(b) for b in bands], [aux["band_rt"][b] for b in bands])
    ax[2, 0].set(title="RT60 (T20) per octave band", xlabel="Hz", ylabel="s")

    et, ev = aux["echo"]
    ax[2, 1].semilogx(et * 1000, ev)
    ax[2, 1].axhline(1.0, color="k", lw=0.5)
    ax[2, 1].set(title="Normalised echo density", xlabel="ms", ylabel="NED")
    ax[2, 1].grid(True, which="both", alpha=0.3)
    fig.suptitle(f"{name}: impulse response", y=0.995)
    fig.tight_layout()
    fig.savefig(OUT / f"{name}_ir.png", dpi=110)
    plt.close(fig)


def plot_distortion(name, dist, ess_curves):
    fig, ax = plt.subplots(1, 3, figsize=(15, 4))
    lv = sorted(dist["levels"])
    ax[0].plot(lv, [dist["levels"][k][0] for k in lv], "o-", label="THD")
    ax[0].plot(lv, [dist["levels"][k][1] for k in lv], "s-", label="THD+N")
    ax[0].set(title="1 kHz vs input level", xlabel="dBFS", ylabel="dB re fundamental")
    fr = sorted(dist["freqs"])
    ax[1].semilogx(fr, [dist["freqs"][k][0] for k in fr], "o-", label="THD")
    ax[1].semilogx(fr, [dist["freqs"][k][1] for k in fr], "s-", label="THD+N")
    ax[1].set(title="at -12 dBFS vs frequency", xlabel="Hz", ylabel="dB")
    for lbl, (fax, thd) in ess_curves.items():
        ax[2].semilogx(fax, thd, "k--" if "floor" in lbl else "o-", label=lbl)
    ax[2].set(title="THD from sweep (harmonic IRs)", xlabel="Hz", ylabel="dB")
    for a in ax:
        a.grid(True, which="both", alpha=0.3)
        a.legend()
    fig.suptitle(f"{name}: distortion & noise (float32 DSP floor)")
    fig.tight_layout()
    fig.savefig(OUT / f"{name}_distortion.png", dpi=110)
    plt.close(fig)


def plot_clicks(name, outs):
    ranges = {"size": "Size 1 -> 7", "predelay": "Pre-delay 0 -> 2000",
              "feedback": "Feedback 0.1 -> 0.9"}
    fig, axes = plt.subplots(1, len(outs), figsize=(5 * len(outs), 3.8))
    for ax, (key, y) in zip(np.atleast_1d(axes), outs.items()):
        f, t, S = signal.spectrogram(y, FS, nperseg=2048, noverlap=1536)
        S = 10 * np.log10(S + 1e-20)
        ax.pcolormesh(t, f, S, vmin=S.max() - 110, vmax=S.max(), shading="auto")
        ax.set(title=ranges[key], xlabel="s", ylabel="Hz", ylim=(0, 6000))
    fig.suptitle(f"{name}: 1 kHz tone while a parameter is ramped over 10 s")
    fig.tight_layout()
    fig.savefig(OUT / f"{name}_ramp.png", dpi=110)
    plt.close(fig)


def plot_grid(sizes, fbs, grid):
    fig, ax = plt.subplots(figsize=(6, 4.5))
    im = ax.imshow(grid, origin="lower", aspect="auto", cmap="viridis")
    ax.set_xticks(range(len(sizes)), [str(s) for s in sizes])
    ax.set_yticks(range(len(fbs)), [str(f) for f in fbs])
    ax.set(xlabel="Size", ylabel="Feedback", title="RT60 (T20, s), mix = 1")
    for i in range(len(fbs)):
        for j in range(len(sizes)):
            v = grid[i, j]
            ax.text(j, i, "n/a" if np.isnan(v) else f"{v:.1f}", ha="center", va="center",
                    color="w", fontsize=8)
    fig.colorbar(im, label="s")
    fig.tight_layout()
    fig.savefig(OUT / "rt60_grid.png", dpi=110)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Self-test: validate the analysis chain against known systems
# ---------------------------------------------------------------------------
def selftest():
    ok = True

    def check(name, got, want, tol):
        nonlocal ok
        good = abs(got - want) <= tol
        ok &= good
        print(f"  [{'ok' if good else 'FAIL'}] {name}: {got:.3f} (want {want:.3f} +/- {tol})")

    print("selftest: synthetic exponential-decay IR")
    rng = np.random.default_rng(0)
    t = np.arange(int(8 * FS)) / FS
    h = rng.standard_normal(len(t)) * 10 ** (-60 * t / 2.0 / 20)  # RT60 = 2.0 s
    rt, _ = rt_set(h)
    check("T20", rt["t20"], 2.0, 0.1)
    check("T30", rt["t30"], 2.0, 0.1)
    check("EDT", rt["edt"], 2.0, 0.15)
    et, ev = echo_density(h)
    check("echo density (gaussian noise)", float(np.median(ev[10:])), 1.0, 0.1)

    print("selftest: THD of a known nonlinearity, stepped sine vs sweep")
    g, amp = 4.0, 0.25
    nl = lambda v: np.tanh(g * v) / g
    n = 1 << 16
    k = 1000 * n // FS
    seg = nl(amp * np.sin(2 * np.pi * k * np.arange(n) / n))
    thd_sine, _ = thd_fft(seg, k)

    x, inv, L = ess(amp=amp)
    h_full = signal.fftconvolve(nl(x), inv) / np.abs(signal.fftconvolve(x, inv)).max()
    fax, thd = thd_from_ess(h_full, len(x), L)
    i = int(np.argmin(np.abs(fax - 1000)))
    print(f"  stepped sine THD @1kHz: {thd_sine:.2f} dB, sweep: {thd[i]:.2f} dB")
    check("sweep THD vs stepped-sine THD", thd[i], thd_sine, 1.5)
    print("selftest: click detector")
    tone = sine(1000, -20, 2 * FS)
    clicked = tone.copy()
    clicked[FS + 100:] += 0.02  # a 0.02 step: a classic parameter-jump click
    c0, c1 = click_ratio(tone), click_ratio(clicked)
    print(f"  clean tone {c0:.1f}, tone with step {c1:.1f}")
    ok &= c1 > 5 * c0
    print(f"  [{'ok' if c1 > 5 * c0 else 'FAIL'}] click detector separates a step from a clean tone")
    print("selftest:", "PASS" if ok else "FAIL")
    return ok


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------
def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}{k}"
        if isinstance(v, dict):
            out.update(flatten(v, key + "."))
        elif isinstance(v, (int, float)) and not isinstance(v, bool):
            out[key] = float(v)
    return out


def fmt(v, nd=2):
    if isinstance(v, float) and v <= -300:
        return "-inf (exact 0)"
    return "n/a" if v is None or (isinstance(v, float) and not np.isfinite(v)) else f"{v:.{nd}f}"


def settled(r, value):
    """Stepped-sine results are only valid if the tone reached steady state."""
    return value if r["dist"]["steady_state_err_db"] <= -100 else None


def write_report(results, grid):
    L = ["# FDNVerb offline measurement report", ""]
    L.append(f"I/O quantization: {'%d-bit' % BITS if BITS else 'none (float32)'}. "
             "Metrics describe the algorithm only; the converters and analog "
             "stages of the pedal are not modelled.\n")
    names = list(results)
    L.append("| metric | " + " | ".join(names) + " |")
    L.append("|---|" + "---|" * len(names))

    def row(label, getter, nd=2):
        L.append(f"| {label} | " + " | ".join(fmt(getter(results[n]), nd) for n in names) + " |")

    row("RT60 T20 (s)", lambda r: r["ir"]["rt60_t20_s"])
    row("RT60 T30 (s)", lambda r: r["ir"]["rt60_t30_s"])
    row("EDT (s)", lambda r: r["ir"]["edt_s"])
    row("first arrival (ms)", lambda r: r["ir"]["first_arrival_ms"], 1)
    row("echo density = 1 at (ms)", lambda r: r["ir"]["echo_density_unity_ms"], 0)
    row("C50 (dB)", lambda r: r["ir"]["c50_db"])
    row("C80 (dB)", lambda r: r["ir"]["c80_db"])
    row("D50", lambda r: r["ir"]["d50"], 3)
    row("centre time (ms)", lambda r: r["ir"]["centre_time_ms"], 0)
    row("IACC early / late", lambda r: None)
    L[-1] = "| IACC early / late | " + " | ".join(
        f"{fmt(results[n]['ir']['iacc_early'])} / {fmt(results[n]['ir']['iacc_late'])}"
        for n in names) + " |"
    row("coloration, 100 Hz-10 kHz (dB rms)", lambda r: r["ir"]["response_coloration_db"])
    row("tilt 4-8 kHz vs 125-250 Hz (dB)", lambda r: r["ir"]["response_tilt_db"], 1)
    for fc in OCTAVES:
        row(f"RT60 {fc} Hz (s)", lambda r, fc=fc: r["ir"]["rt60_octave_s"][str(fc)])
    row("wet-path gain, pink noise (dB)", lambda r: r["levels"]["wet_gain_db"], 1)
    row("noise level that clips (RMS dBFS)", lambda r: r["levels"]["noise_clip_input_rms_dbfs"], 1)
    row("1 kHz sine level that clips (dBFS)", lambda r: r["levels"]["sine_clip_input_dbfs"], 1)
    row("THD @ 1 kHz -12 dBFS (dB)", lambda r: settled(r, r["dist"]["levels"]["-12"][0]), 1)
    row("THD+N @ 1 kHz -12 dBFS (dB)", lambda r: settled(r, r["dist"]["levels"]["-12"][1]), 1)
    row("tone settled? steady-state error (dB)", lambda r: r["dist"]["steady_state_err_db"], 0)
    row("numeric SNR, -20 dBFS tone (dB)", lambda r: settled(r, r["dist"]["snr_numeric_db"]), 1)
    row("dynamic range, AES17-style (dB)", lambda r: settled(r, r["dist"]["dynamic_range_db"]), 1)
    row("idle noise (dBFS)", lambda r: r["idle"]["idle_noise_dbfs"], 1)
    row("max-feedback decay, RMS 30 s after burst (dBFS)", lambda r: r["idle"]["decay_final_rms_dbfs"], 1)
    row("click index, size ramp", lambda r: r["clicks"]["size"]["ramped"])
    row("click index, predelay ramp", lambda r: r["clicks"]["predelay"]["ramped"])
    row("click index, feedback ramp", lambda r: r["clicks"]["feedback"]["ramped"])
    row("click index, static (reference)", lambda r: r["clicks"]["size"]["static"])
    row("host cost (ns/sample)", lambda r: r["cost"]["host_ns_per_sample"], 0)
    L += ["", "THD, THD+N, SNR and dynamic range are shown only when the tone "
          "reached steady state (steady-state error <= -100 dB). A reverb driven "
          "by a tone needs time to settle; very long decays are capped at %d s "
          "and are then reported as n/a." % MAX_SETTLE_S, ""]
    L += ["Click index = largest |second difference| of the output relative to "
          "its local RMS (+/- 25 ms), while a parameter is ramped over 10 s. A "
          "smooth signal stays near the static reference; an audible click would "
          "stand well above it.", ""]
    L.append("Dynamic range follows AES17 (60 dB minus the THD+N ratio of a "
             "-60 dBFS 1 kHz tone) but is unweighted. Host ns/sample is useful "
             "only for comparing code revisions on this machine.\n")
    if any(r["ir"]["tail_incomplete"] for r in results.values()):
        L.append("**Warning:** at least one tail had not decayed within the "
                 "record; its RT60 values may be unreliable.\n")
    for n in names:
        L.append(f"![{n} IR]({n}_ir.png)\n\n![{n} distortion]({n}_distortion.png)\n")
    if grid is not None:
        L.append("![RT60 grid](rt60_grid.png)\n")
    (OUT / "report.md").write_text("\n".join(L), encoding="utf-8")


def compare(new, old_path):
    old = json.loads(Path(old_path).read_text())
    a, b = flatten(old), flatten(new)
    print(f"\ncomparison vs {old_path} (changes > 2 % or > 0.5 dB shown)")
    shown = 0
    for k in sorted(set(a) & set(b)):
        if "host_ns" in k:
            continue
        x, y = a[k], b[k]
        if not (np.isfinite(x) and np.isfinite(y)):
            continue
        if abs(y - x) > max(0.5, 0.02 * abs(x)) if k.endswith("_db") or "dbfs" in k \
                else abs(y - x) > 0.02 * max(abs(x), 1e-3):
            print(f"  {k}: {x:.4g} -> {y:.4g}")
            shown += 1
    if not shown:
        print("  no significant changes")


def main():
    global BITS
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--preset", default="all", choices=["all"] + list(PRESETS))
    ap.add_argument("--quick", action="store_true", help="skip the RT60 parameter grid")
    ap.add_argument("--bits", type=int, help="quantize input and output to N bits")
    ap.add_argument("--compare", help="earlier metrics.json to diff against")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()
    BITS = args.bits

    if args.selftest:
        sys.exit(0 if selftest() else 1)
    if not EXE.exists():
        sys.exit(f"{EXE} not found - run build.ps1 first")
    OUT.mkdir(exist_ok=True)
    if not selftest():
        sys.exit("analysis self-test failed; refusing to report")

    names = list(PRESETS) if args.preset == "all" else [args.preset]
    results = {}
    for name in names:
        preset = PRESETS[name]
        print(f"\n== {name} {preset}")
        h, _ = measure_ir(preset)
        irm, aux = analyze_ir(h)
        plot_ir(name, h, irm, aux)
        print(f"  RT60 T20 {fmt(irm['rt60_t20_s'])} s, C80 {fmt(irm['c80_db'])} dB")

        levels = measure_levels(preset, irm["rt60_t20_s"])
        dist = measure_distortion(preset, late_rt60(aux["edc"]), args.quick)
        print(f"  THD+N @-12 dBFS {fmt(dist['levels'][-12][1], 1)} dB, "
              f"DR {fmt(dist['dynamic_range_db'], 1)} dB")

        # Sweep-based THD: nominal level and full scale (full scale can clip the
        # output sum, which shows the headroom limit).
        ess_curves = {}
        for lvl in (-12, 0):
            ir_s, fax, thd = sweep_measure(dict(preset, mix=0.5, chorusOn=False), lvl)
            ess_curves[f"sweep {lvl:+d} dBFS"] = (fax, thd)
        ess_curves["analysis floor (linear system)"] = sweep_floor(-12)
        plot_distortion(name, dist, ess_curves)

        idle = measure_idle(preset)
        clicks, ramp_outs = measure_clicks(preset)
        plot_clicks(name, ramp_outs)
        cost = measure_cost(preset)
        dist_json = {"levels": {str(k): v for k, v in dist["levels"].items()},
                     "freqs": {str(k): v for k, v in dist["freqs"].items()},
                     "snr_numeric_db": dist["snr_numeric_db"],
                     "dynamic_range_db": dist["dynamic_range_db"],
                     "settle_s": dist["settle_s"],
                     "steady_state_err_db": dist["steady_state_err_db"],
                     "sweep_thd_db": {k: {"freq_hz": v[0].tolist(), "thd_db": v[1].tolist()}
                                      for k, v in ess_curves.items()}}
        results[name] = {"params": preset, "ir": irm, "levels": levels,
                         "dist": dist_json, "idle": idle, "clicks": clicks, "cost": cost}

    grid = None
    if not args.quick and args.preset == "all":
        print("\n== parameter grid")
        sizes, fbs, g = parameter_grid()
        plot_grid(sizes, fbs, g)
        grid = {"sizes": sizes, "feedback": fbs, "rt60_t20_s": g.tolist()}
        results["_grid"] = grid
    write_report({k: v for k, v in results.items() if not k.startswith("_")}, grid)
    (OUT / "metrics.json").write_text(json.dumps(results, indent=1, default=float))
    print(f"\nreport: {OUT / 'report.md'}")
    if args.compare:
        compare(results, args.compare)


if __name__ == "__main__":
    main()
