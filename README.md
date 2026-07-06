# S4 Slicer
A generic non-planar slicer, that can print almost any part without support.

Please use the [dicussions tab](https://github.com/jyjblrd/S4_Slicer/discussions) to ask questions and help others.

[Try it now](https://colab.research.google.com/github/jyjblrd/S4_Slicer) on Google Colab! (note: colab free tier is only powerful enough to slice very simple models)

[![Watch the video](https://github.com/jyjblrd/S4_Slicer/blob/main/thumnail.jpeg?raw=true)](https://www.youtube.com/watch?v=M51bMMVWbC8)

Check out my [YouTube video](https://youtu.be/M51bMMVWbC8?si=pfud7bHgjYDnO2_z) for more details!

Thank you to JLCCNC for helping create the extruder mount and build plate for my [4 Axis Core R-Theta Printer](https://github.com/jyjblrd/Core-R-Theta-4-Axis-Printer).



Bibtex Citation:
```
@software{Bird_S4_Slicer,
author = {Bird, Joshua},
license = {GPL-3.0},
title = {{S4 Slicer}},
url = {https://github.com/jyjblrd/S4_Slicer}
}
```

## Automated pipeline (OrcaSlicer, no manual Cura step)

The original S4 workflow required exporting the deformed STL, slicing it
manually in Cura, then feeding the planar gcode back into the inverse-transform.
This pipeline automates the slicing seam with OrcaSlicer headless, giving you
a single-command deform -> slice -> 4-axis transform flow.

### Install

```bash
pip install -r requirements.txt
```

### Usage

```bash
python -m s4.pipeline "input_models/pi 3mm.stl" -o "output_gcode/pi 3mm.gcode"
```

Options:

| Flag | Description |
|---|---|
| `-o`, `--output` | Output `.gcode` file path (required) |
| `--config` | Path to `config.yaml` (default: built-in defaults) |
| `--workdir` | Working directory for intermediate files (default: parent of `--output`) |

### OrcaSlicer binary + profiles

Set these top-level keys in `config.yaml` (see the shipped example):

- `orca_binary` — path to an OrcaSlicer binary. A native build (`build/src/orca-slicer`)
  avoids AppImage/FUSE issues.
- `orca_config` — `"machine.json;process.json"` (`;`-joined) passed to `--load-settings`.
- `orca_filament` — filament json passed to `--load-filaments` (a **separate** flag;
  putting the filament inside `orca_config` makes Orca exit 206).
- `bed_center` — `[x, y]` mm, the bed center of that machine profile.

### Coordinate frame (bed_center)

S4 slices with the part at **XY origin** (the rotary axis). The shipped machine
profile (`s4/profiles/S4_klipper_centered.json`) is therefore a **center-origin**
250 mm bed, so the part slices as-placed and `bed_center` is `null` — no reframe.

For a **corner-origin** bed instead, set `bed_center` to that bed's center (e.g.
a 256 mm BBL bed → `[128, 128]`): the pipeline then translates the mesh onto the
bed to slice and shifts the planar gcode's X/Y back to origin for Stage C.

Either way the pipeline trims the slicer's start/end gcode (purge, calibration)
down to the model toolpath — Stage C rebuilds its own preamble, and those moves
would otherwise be mapped into spurious extrusions.

**Note on Orca CLI profiles:** OrcaSlicer's headless `--load-settings` only accepts
a machine+process pair whose compatibility its CLI recognizes. Empirically, stock
**Klipper** (`Generic Klipper Printer`) and **Bambu** pairs slice; stock **Marlin**
and **Voron** pairs return exit `-17` "not compatible". The shipped profile inherits
the Generic Klipper Printer, overrides the bed to center-origin, and adds `G92 E0`
to `layer_change_gcode` (Klipper relative-E requires it). Point `orca_config` at
your own printer's profile when you have one.

### Planar-slice constraints (soft-verified)

S4's kinematics require the planar gcode to have:

- **No Z-hop** (z hop when retracting = 0; disable retract lift)
- **Relative extrusion** (`M83`; absolute `M82` breaks the inverse-transform)

The pipeline logs warnings if these are violated but does not halt.

### Folder watcher

For continuous use (drop STLs into a folder, get gcode out):

```bash
python watch.py --in ~/drop_stls --out ~/gcode_output --config s4/config.yaml
```

### Credit

Algorithm: Joshua Bird's S4 (GPL-3.0, https://github.com/jyjblrd/S4_Slicer).
This package only automates the slicing seam — all deformation and
inverse-transform logic remains Joshua's original work.
