# Porting the Core R-Theta (S4) printer to Klipper — scope & plan

Status: **planning**. Target machine: Joshua Bird's [Core-R-Theta-4-Axis-Printer](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer),
currently RepRapFirmware. Goal: run the S4 pipeline's 4-axis output on **Klipper**.

## TL;DR

- Klipper's motion core is **hardcoded to 3 axes** (C code: `trapq.c`, iterative
  solver). A 4th/5th coordinated axis is not a config change.
- The only maintained multi-axis base is [`naikymen/klipper-for-cnc`](https://github.com/naikymen/klipper-for-cnc)
  (`cartesian_abc` kinematics: coordinated `XYZABC`). It treats extra axes as
  **independent linear** axes and has **no G93** (F is shared, Euclidean).
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

## Chosen route: naikymen `cartesian_abc` + slicer-side kinematics

Klipper runs 4 **independent** steppers (the 4 physical motors); all Core-RΘ
kinematics live in Stage C. Two design decisions, to be settled in Phase 0:

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

### Phase 0 — Derive & validate the kinematics (BLOCKING, highest risk)
- Derive the core-pair coupling: `(motor_a, motor_b) ↔ (radial, tilt)` from the
  belt routing + 16T/40T pulley ratios, **cross-checked** against his `M669 K0`
  matrix (`X-1:0:0:1:0`, `B0.222…:0:0:0.222…`) and steps/mm.
- Validate numerically against ≥3 known poses (e.g. B-home = nozzle down; pure
  radial move = no tilt). A wrong matrix = a machine that moves wrong.
- Settle D1 (motor-space vs coupled kinematic).
- Deliverable: forward+inverse equations + a `test_kinematics` self-check.

### Phase 1 — Stage C Klipper output mode (slicer, we own this)
- Add `kinematics: "klipper_core_rtheta"` to `s4/transform.py`: apply the D1
  transform, emit plain-F `mm/min` moves (D2), drop G93.
- Reuse the existing 42 mm nozzle-offset comp, tilt clamp, C accumulation.
- Unit tests for the transform + feed conversion.

### Phase 2 — Klipper firmware & config
- Stand up `naikymen/klipper-for-cnc` on a Klipper-capable MCU (the Fly-E3-Pro
  can run Klipper, or use an existing Klipper board). Confirm `cartesian_abc`.
- Config: 4 steppers (steps/mm, currents, limits), sensorless homing on the core
  pair (the tricky bit), Z probe, extruder, TMC2209.
- If D1=Option B: apply the coupled-kinematic patch.
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
5. **naikymen fork drift** — pin a commit; it's a hard fork off mainline.

## Fallback
If Klipper bring-up stalls, his RRF configs run the S4 output **today** on a
Duet-class board — keep that as the reference/validation target throughout.
