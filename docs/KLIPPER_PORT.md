# Porting the Core R-Theta (S4) printer to Klipper — scope & plan

Status: **planning**. Target machine: Joshua Bird's [Core-R-Theta-4-Axis-Printer](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer),
currently RepRapFirmware. Goal: run the S4 pipeline's 4-axis output on **Klipper**.

## TL;DR

- Klipper's motion core is **hardcoded to 3 axes** (C code: `trapq.c`, iterative
  solver). A 4th/5th coordinated axis is not a config change.
- Base fork: [`gear2nd-droid/klipper`](https://github.com/gear2nd-droid/klipper)
  `6axes_support` — already extends the motion core past 3 axes and ships C
  kinematics for rotary/tilt axes. No G93 in it either (F shared, Euclidean).
  ([`naikymen/klipper-for-cnc`](https://github.com/naikymen/klipper-for-cnc) is
  the simpler fallback.)
- Therefore the coupling (X/B CoreXY pair) and the feedrate model move **into
  Stage C of this slicer, which we own** — the hard kinematics unknown becomes
  tunable Python instead of C.
- RRF remains the low-effort, more-native option (`M669` matrix + `G93` native).
  This port is a deliberate choice to gain the Klipper ecosystem, not capability.

## Machine definition (extracted from his RRF `to4axis.g` / homing)

Board: Fly-E3-Pro-v3 (STM32, RRF 3.5), TMC2209, 24 V. Extruder BIQU H2, long nozzle.

| Axis | Role | Range | steps/mm | Drive |
|---|---|---|---|---|
| C | rotary bed, infinite (homes to C0, no endstop) | ±∞ | 88.889 | independent (d0) |
| X | radial | −37.5 … 115.5 mm | 100 | **core pair (d1,d2)** |
| B | nozzle tilt | −180 … +90° | 100 | **core pair (d1,d2)** |
| Z | vertical (leadscrew) | −50 … 200 mm | 400 | independent (d4) |
| E | extruder | — | 932 | d3 |

- **X and B are a CoreXY-style coupled pair** (drivers 1+2): homing drives them
  together (`X20 B-20`), common-mode ≈ radial, differential ≈ tilt. Belt uses
  16T/40T GT2 pulleys (→ ratio 2.5 somewhere; steps/mm 100 & the `M669` `0.2222`
  coefficient encode the coupling).
- Sensorless homing on X-high and B-high; Z via probe (`M558 P5` on `^xstop`).
- Heterogeneous axes (mm + deg) → RRF uses **G93 inverse-time feed**.
- A second RRF mode (`topolar.g`, `M669 K7` polar) exists for his simpler radial
  slicer — **out of scope**, we only port the 4-axis mode.

## Prior art — 4/5-axis Klipper (GitHub sweep, 2026-07)

| Project | What | Relevance |
|---|---|---|
| **[gear2nd-droid/klipper](https://github.com/gear2nd-droid/klipper)** `6axes_support` | Active 6-axis Klipper fork, Klipper 0.12 (2026-06). C kinematics `kin_corexybc.c`, `kin_trunnion_bc.c`, `kin_polar.c` (CoreXY/trunnion **+ B + C**). | **Best base** — 3-axis ceiling solved + rotary/tilt templates. |
| **[gear2nd-droid/MageSlicer](https://github.com/gear2nd-droid/MageSlicer)** | NURBS multi-axis slicer (4+ axes) + MAGE 5-axis printer. | Proves toolchain; feed baked slicer-side (no G93). Alt to S4. |
| **[naikymen/klipper-for-cnc](https://github.com/naikymen/klipper-for-cnc)** | `cartesian_abc` XYZABC, independent linear axes. | Simpler fallback; no coupling/rotary. |
| **[XyrusB2010/XYmatics-E64](https://github.com/XyrusB2010/XYmatics-E64)** + Alpha | 5-axis Klipper mainboard (CM4+STM32F407, CAN) + CoreXY-BC printer. | Hardware ref; real corexybc machine. |
| Open5x, RotBot | Duet/RRF, not Klipper. | Kinematics reference only. |

## Chosen route: gear2nd-droid fork + a `polar_bc` kinematics

Re-based from naikymen to **gear2nd-droid/klipper `6axes_support`**: it already
extends Klipper's motion core past 3 axes and ships C kinematics for added B/C
rotary+tilt axes. Our work is to write **one new kinematics, `polar_bc`**
(`kin_polar_bc.c` + `polar_bc.py`), combining:
- radial (X) + rotary (C) ← model on `kin_polar.c`
- B tilt + the X/B CoreXY core-pair mixing ← model on `kin_corexybc.c` /
  `kin_trunnion_bc.c`
- independent Z.

Pin a specific gear2nd commit (hard fork off mainline). Feed still has **no
G93** in this fork either — it stays slicer-side (Stage C), as MageSlicer does.

Two design decisions, to be settled in Phase 0:

**D1 — coupling location (motor-space vs coupled kinematic).**
- *Option A (motor-space, least Klipper code):* Stage C emits belt-motor targets
  for the core pair; Klipper axes = raw motors, zero coupling in Klipper.
  Downside: **homing** — the radial/tilt endstops trigger on logical position,
  not motor position, so the coupled-pair homing needs custom macros.
- *Option B (coupled kinematic, cleaner semantics):* keep logical axes
  (radial=X, tilt=B); port the mainline `generic_cartesian` `kinematics: x+y`
  mixing idea into `cartesian_abc` for the core pair. Homing works on logical
  axes. Cost: a small Klipper kinematic patch (the "custom kinematics" work).
- *Leaning B* for correct homing; decide after deriving the matrix (Phase 0).

**D2 — feedrate (no G93).** Stage C emits plain `mm/min` F, computing move time
from a Euclidean length over active axes with a chosen **deg↔mm weight** per
rotary axis (matches naikymen's shared-F model). Replaces the current
`inv_time_feed` / G93 output. Tune the weight so tool-tip speed stays sane during
rotation (no native rotary TCP in Klipper — this is the known quality tradeoff).

**Axis mapping (proposed):** Klipper `X`=radial, `Z`=vertical, `A`=C rotary
(deg-as-linear), `B`=tilt (deg-as-linear); `Y` idle. (Under Option A, `X`/`B`
become the two core motors instead.)

## Phases

### Phase 0 — Derive & validate the kinematics ✅ DONE
Decoded directly from his `M669 K0` matrix (authoritative — his machine prints
with it), column order `[X, Y, Z, B, C]`:

```
mC = C            (identity, drive0, 88.889 st/mm)
mZ = Z            (identity, drive4, 400 st/mm)
m1 = -X + B       (coupled, drive1, 100 st/mm)     radial-motor
m2 = K·(X + B)    (coupled, drive2, 100 st/mm)     tilt-motor,  K = 2/9 = 1/4.5
```
X in mm, B in deg. Only the X/B pair is coupled; det = −0.4444 (invertible).
The `1/4.5` is the tilt reduction (16T→72T stage; the earlier "16T/40T" note was
incomplete — the matrix supersedes it). Cross-checked against his homing moves:
`homex.g X300`→(−300, +66.67), `homeb.g B1000`→(+1000, +222.2), both motors move
on either logical home = correct coupled-pair behavior.

**Reference impl + self-check:** `s4/kinematics_rtheta.py` (`forward`/`inverse`,
5 round-trip poses + 2 homing fixtures, `python3 -m` runnable). PASS.

**D1 SETTLED → Option B (coupled kinematic in Klipper; logical axes X=radial,
B=tilt).** His RRF homing is already expressed as *logical-axis* sensorless
moves (`G1 H1 X300`, `G1 H1 B1000`), so keeping logical axes maps homing 1:1.
The decisive win: **Stage C stays logical R-θ (unchanged); only the feedrate
model changes (drop G93).** Option A would push motor-space math into Stage C
*and* still need custom homing macros.

Full machine ground truth now in `reprap firmware config/` of the cloned
`Core-R-Theta-4-Axis-Printer` repo (config.g, to4axis.g, homex/homeb/homez/homec.g).

### Phase 1 — Stage C Klipper output mode (slicer, we own this) ✅ DONE
Added `kinematics: "klipper"` to `s4/transform.py`. Because D1=Option B keeps
logical axes, Stage C emits the **same** `C/X/Z/B` R-θ letters as `rtheta` — the
only changes vs `rtheta` are:
- **Preamble:** `G94` mm/min throughout, **no `G93`** inverse-time.
- **Feed (D2):** new pure helper `klipper_feed(dX,dZ,dC,dB, inv_time_feed, wC,wB,
  travel)`. `F = weighted_len · (1/planar_time)` in mm/min, over Klipper's own
  metric (deg treated as mm 1:1). Weights `FEED_WEIGHT_C/B` default `1.0`, which
  **preserves the planar move time exactly** → extrusion stays consistent;
  weight `< 1` slows rotary-heavy moves. `TRAVEL_FEED` (20000) fallback when time
  is undefined. There is no free weight that both tames rotary speed *and* keeps
  time — the real velocity ceiling is firmware-side (Klipper `max_velocity` /
  per-stepper), set in Phase 2.
- Nozzle-offset comp / tilt clamp / C accumulation all reused unchanged.

Verified: 18/18 unit tests (4 new on the feed helper); full pipeline run
(`pi 3mm`, klipper config) → clean gcode, 0×`G93`, `G94` mm/min, all 390 moves
carry a plain `F`, feeds ~8000 mm/min (sane). Config: `s4/config.yaml`
`kinematics: "klipper"` + optional `FEED_WEIGHT_C/B`, `TRAVEL_FEED`.

### Phase 2 — Klipper firmware & config (gear2nd base + `polar_bc` kinematics)
- Fork `gear2nd-droid/klipper` at a pinned `6axes_support` commit; stand it up on
  a Klipper-capable MCU (Fly-E3-Pro can run Klipper, or use an existing board).
- Write `polar_bc` (`kin_polar_bc.c` + `klippy/kinematics/polar_bc.py`) modeled on
  `kin_polar.c` (radial+C) + `kin_corexybc.c`/`kin_trunnion_bc.c` (tilt + core
  pair). This is the D1=Option B "custom kinematics" — homing works on logical
  radial/tilt axes.
- Config: steppers (steps/mm, currents, limits), sensorless homing on the core
  pair, Z probe, extruder, TMC2209.
- Replicate the RRF homing sequence (X-high/B-high sensorless, C=G92 home, Z probe).

### Phase 3 — Bring-up & validation
- Bench each axis in isolation: pure radial → no tilt; pure tilt → in place;
  C spins; Z moves. Compare against documented RRF behavior.
- Dry-run an S4 gcode (e.g. `pi 3mm`) with no filament; watch motion.
- First real non-planar print; compare to the RRF reference.

### Phase 4 — Calibration & tuning
- Nozzle-to-pivot offset (42 mm), tilt zero, C center, feed deg↔mm weight,
  input shaping (the actual reason to be on Klipper).

## Risks / open questions
1. **Coupling matrix correctness** (Phase 0) — highest risk; mitigated by putting
   it in tunable Python + bench validation before any print.
2. **Coupled-pair sensorless homing in Klipper** — RRF's core-pair homing doesn't
   map 1:1; may need `[manual_stepper]` or homing-override macros.
3. **Feed quality without rotary TCP** — Euclidean feed may cause speed surges
   near the rotary axis (small r); tune D2 weight, may need slicer-side clamping.
4. **Board Klipper compatibility** — confirm the Fly board flashes Klipper, or
   pick a board.
5. **gear2nd fork drift** — pin a commit; it's a hard fork off mainline, and the
   author explicitly declines support ("do not use unless you can write your own
   kinematics"). We are writing our own — but expect no hand-holding.
6. **`polar_bc` C kinematics correctness** — new C code; validate the forward/
   inverse against the RRF reference and bench (ties to risk 1).

## Fallback
If Klipper bring-up stalls, his RRF configs run the S4 output **today** on a
Duet-class board — keep that as the reference/validation target throughout.
