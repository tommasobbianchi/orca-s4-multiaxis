"""S4 pipeline — orchestrate Stage A (deform) -> Stage B (Orca slice) -> Stage C (transform).

Usage:
    python -m s4.pipeline INPUT.stl -o OUT.gcode [--config config.yaml] [--workdir DIR]
"""

import argparse
import logging
import os
import time
from pathlib import Path

log = logging.getLogger(__name__)


def load_config(path):
    import yaml

    from s4.deform import DeformParams
    from s4.transform import TransformParams

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config not found: {path}")
    raw = yaml.safe_load(path.read_text())

    deform_params = DeformParams(**raw.get("deform", {}))
    transform_params = TransformParams(**raw.get("transform", {}))
    # orca_binary / orca_config are TOP-LEVEL keys in config.yaml.
    misc = {
        "orca_binary": raw.get("orca_binary"),
        "orca_config": raw.get("orca_config") or None,
        "orca_filament": raw.get("orca_filament") or None,
        "bed_center": raw.get("bed_center") or None,
    }

    return deform_params, transform_params, misc


def run(stl_path, out_gcode, config_path=None, workdir=None, orca_binary=None):
    from s4.deform import DeformParams, run_deform, export_surface_stl
    from s4.transform import TransformParams, prepare_transform_fields, transform_gcode, write_gcode
    from s4.slice_orca import detect_orca_binary, slice_stl, verify_planar_constraints

    workdir = Path(workdir) if workdir else Path(out_gcode).parent
    workdir.mkdir(parents=True, exist_ok=True)

    if config_path:
        deform_params, transform_params, misc = load_config(config_path)
    else:
        deform_params = DeformParams()
        transform_params = TransformParams()
        misc = {"orca_binary": None, "orca_config": None}

    # Explicit orca_binary arg (e.g. CLI --orca-binary) overrides the config.
    resolved_binary = detect_orca_binary(orca_binary or misc.get("orca_binary"))
    orca_config = misc.get("orca_config") or None
    if orca_config and not all(os.path.isfile(p) for p in orca_config.split(";")):
        log.warning("orca_config %s not fully found, slicing without a profile.", orca_config)
        orca_config = None
    orca_filament = misc.get("orca_filament") or None
    if orca_filament and not os.path.isfile(orca_filament):
        log.warning("orca_filament %s not found, slicing without a filament.", orca_filament)
        orca_filament = None

    t0 = time.time()
    log.info("Stage A: deforming %s ...", stl_path)
    deformed_tet, original_tet = run_deform(stl_path, deform_params)
    deformed_stl = str(workdir / "deformed.stl")
    export_surface_stl(deformed_tet, deformed_stl)
    log.info("Stage A done (%.1fs), deformed STL at %s",
             time.time() - t0, deformed_stl)

    t1 = time.time()
    log.info("Stage B: slicing deformed STL ...")
    planar = slice_stl(deformed_stl, str(workdir), resolved_binary, orca_config,
                       orca_filament, bed_center=misc.get("bed_center"))
    log.info("Stage B done (%.1fs), planar gcode at %s",
             time.time() - t1, planar)

    for w in verify_planar_constraints(planar):
        log.warning(w)

    t2 = time.time()
    log.info("Stage C: inverse-transforming planar gcode -> 4-axis gcode ...")
    fields = prepare_transform_fields(original_tet, deformed_tet, transform_params)
    lines = transform_gcode(planar, deformed_tet, fields, transform_params)
    write_gcode(lines, out_gcode)
    log.info("Stage C done (%.1fs), output gcode at %s",
             time.time() - t2, out_gcode)

    log.info("Pipeline complete (total %.1fs)", time.time() - t0)
    return out_gcode


def main():
    parser = argparse.ArgumentParser(
        description="S4 non-planar slicer pipeline (deform -> Orca slice -> transform)",
    )
    parser.add_argument("input", help="Input STL file")
    parser.add_argument("-o", "--output", required=True, help="Output .gcode file")
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    parser.add_argument("--workdir", default=None, help="Working directory for intermediate files")
    parser.add_argument("--orca-binary", default=None, help="Override OrcaSlicer binary path")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")

    run(
        args.input,
        args.output,
        config_path=args.config,
        workdir=args.workdir,
        orca_binary=args.orca_binary,
    )


if __name__ == "__main__":
    main()
