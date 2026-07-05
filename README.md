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

### OrcaSlicer profile requirements

S4's belt-printer kinematics require a planar gcode with these constraints.
Your OrcaSlicer profile **must** enforce:

- **Origin at bed center** (`printable_area` / origin 0,0 at center)
- **No Z-hop** (z hop when retracting = 0; disable retract lift)
- **Relative extrusion** (`M83`; absolute extrusion `M82` will break the inverse-transform)

The pipeline runs a soft verification on the produced gcode and logs warnings
if any of these are violated, but does not halt — it is your responsibility
to use a correct profile.

### Folder watcher

For continuous use (drop STLs into a folder, get gcode out):

```bash
python watch.py --in ~/drop_stls --out ~/gcode_output --config s4/config.yaml
```

### Credit

Algorithm: Joshua Bird's S4 (GPL-3.0, https://github.com/jyjblrd/S4_Slicer).
This package only automates the slicing seam — all deformation and
inverse-transform logic remains Joshua's original work.
