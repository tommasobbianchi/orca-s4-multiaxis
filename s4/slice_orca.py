# S4_Slicer Stage B — headless OrcaSlicer planar slice
#
# Replaces the manual "slice deformed STL in Cura" step with a subprocess
# call to OrcaSlicer CLI, then runs soft constraint verification on the
# produced planar gcode (relative extrusion, no Z-hop).

import logging
import os
import re
import shutil
import subprocess
import time
from pathlib import Path

log = logging.getLogger(__name__)

# `--slice 0` slices all plates and exports G-code into --outputdir. This build
# rejects PrusaSlicer's --export-gcode; keep the arg list here for easy tweaking.
ORCA_SLICE_ARGS = ["--slice", "0", "--arrange", "0", "--orient", "0"]


def detect_orca_binary(configured=None):
    if configured and os.path.isfile(configured):
        return configured
    found = shutil.which("orca-slicer")
    if found:
        return found
    fallback = "/home/tommaso/projects/orca-cad-primitives/build/Snapmaker_Orca_Linux_V2.3.4.AppImage"
    if os.path.isfile(fallback):
        return fallback
    raise FileNotFoundError(
        "No OrcaSlicer binary found. Provide a path via config (orca_binary) "
        "or install orca-slicer on PATH."
    )


def _translate_stl(src, dst, dx, dy):
    """Rigid XY translation of a mesh (to place an origin-centered part onto a
    corner-origin bed). pyvista is already a Stage-A dependency."""
    import pyvista as pv
    m = pv.read(src)
    m.points[:, 0] += dx
    m.points[:, 1] += dy
    m.save(dst)


def _trim_to_toolpath(gcode_path):
    """Keep only the model toolpath, dropping the slicer's start-gcode (nozzle
    purge, bed leveling, calibration) and end-gcode. Stage C rebuilds its own
    preamble; left in, those preamble moves get mapped through the tet field into
    spurious extrusions, and they also bloat Stage C's per-point cell lookup.
    Boundary = OrcaSlicer's first '; CHANGE_LAYER' .. '; EXECUTABLE_BLOCK_END'.
    ponytail: markers absent (non-Orca gcode) -> leave the file unchanged."""
    lines = Path(gcode_path).read_text().splitlines()
    start = end = None
    for i, ln in enumerate(lines):
        s = ln.strip()
        if start is None and s == "; CHANGE_LAYER":
            start = i
        elif s == "; EXECUTABLE_BLOCK_END":
            end = i
            break
    if start is None:
        return
    body = lines[start:end] if end is not None else lines[start:]
    Path(gcode_path).write_text("\n".join(body) + "\n")


def _reframe_gcode(gcode_path, dx, dy):
    """Subtract (dx, dy) from absolute X/Y on G0/G1 moves, undoing the bed
    translation so the toolpath is back in the part frame (rotary axis at 0,0)
    that Stage C requires. Orca's start/end gcode is discarded by Stage C, so a
    blanket shift of every move is safe.
    ponytail: assumes absolute XYZ (G90) and no arc moves (G2/G3, off by default
    in Orca). If arc fitting is ever enabled, reframe I/J too."""
    out = []
    for line in Path(gcode_path).read_text().splitlines():
        head = line.lstrip()[:4].upper()
        if head.startswith(("G1 ", "G0 ", "G01 ", "G00 ")):
            parts = line.split()
            for i, p in enumerate(parts):
                if len(p) > 1 and p[0] in "Xx":
                    try:
                        parts[i] = f"X{float(p[1:]) - dx:.5f}"
                    except ValueError:
                        pass
                elif len(p) > 1 and p[0] in "Yy":
                    try:
                        parts[i] = f"Y{float(p[1:]) - dy:.5f}"
                    except ValueError:
                        pass
            line = " ".join(parts)
        out.append(line)
    Path(gcode_path).write_text("\n".join(out) + "\n")


def slice_stl(stl_path, out_dir, orca_binary, orca_config=None,
              orca_filament=None, bed_center=None, extra_args=None, timeout=600):
    os.makedirs(out_dir, exist_ok=True)

    # S4 places the part at XY origin; Orca beds are corner-origin. Translate the
    # part onto the bed to slice, remember the shift to undo on the gcode.
    dx = dy = 0.0
    if bed_center:
        dx, dy = float(bed_center[0]), float(bed_center[1])
        onbed = str(Path(out_dir) / (Path(stl_path).stem + "_onbed.stl"))
        _translate_stl(stl_path, onbed, dx, dy)
        stl_path = onbed

    args = [orca_binary] + ORCA_SLICE_ARGS
    if orca_config:
        # machine + process, ";"-joined (Orca CLI convention).
        args += ["--load-settings", orca_config]
    # Filament is loaded via its OWN flag, NOT inside --load-settings. Passing it
    # in --load-settings makes Orca exit 206 with an empty/unsatisfiable config.
    if orca_filament:
        args += ["--load-filaments", orca_filament]
    if extra_args:
        args += extra_args
    args += ["--outputdir", out_dir, stl_path]

    env = os.environ.copy()
    if orca_binary.endswith(".AppImage"):
        env["APPIMAGE_EXTRACT_AND_RUN"] = "1"

    # Only accept a gcode written AFTER this point, so a stale file from a prior
    # run can never be mistaken for this slice's output. (-1s covers fs mtime
    # granularity.)
    start = time.time() - 1

    log.info("OrcaSlicer slice: %s", " ".join(str(a) for a in args))
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=timeout, env=env,
    )

    if proc.returncode != 0:
        is_fuse = "fuse" in proc.stderr.lower() and orca_binary.endswith(".AppImage")
        if is_fuse:
            log.info("FUSE error detected, retrying with --appimage-extract-and-run")
            retry_args = [orca_binary, "--appimage-extract-and-run"] + args[1:]
            proc = subprocess.run(
                retry_args, capture_output=True, text=True, timeout=timeout, env=env,
            )
        # Raise on ANY non-zero exit (not only the FUSE branch): otherwise a real
        # slice failure fell through and returned a stale gcode.
        if proc.returncode != 0:
            stderr_tail = "\n".join(proc.stderr.strip().splitlines()[-20:])
            raise RuntimeError(
                f"OrcaSlicer failed (returncode {proc.returncode}):\n{stderr_tail}"
            )

    fresh = [p for p in Path(out_dir).glob("*.gcode") if p.stat().st_mtime >= start]
    if not fresh:
        stderr_tail = "\n".join(proc.stderr.strip().splitlines()[-20:])
        raise RuntimeError(
            f"OrcaSlicer produced no fresh .gcode in {out_dir}.\n"
            f"stdout tail:\n"
            + "\n".join(proc.stdout.strip().splitlines()[-20:])
            + f"\nstderr tail:\n{stderr_tail}"
        )

    gcode = str(max(fresh, key=lambda p: p.stat().st_mtime))
    # Drop slicer preamble/postamble first (smaller file -> faster reframe), then
    # move the toolpath back to the part frame (rotary axis at 0,0) for Stage C.
    _trim_to_toolpath(gcode)
    if bed_center:
        _reframe_gcode(gcode, dx, dy)
    return gcode


def verify_planar_constraints(gcode_path):
    """Soft check of S4's planar-slice constraints. Warns (never raises)."""
    warnings = []
    lines = Path(gcode_path).read_text().splitlines()

    if not any("M83" in line for line in lines):
        warnings.append("WARNING: no M83 (relative extrusion) found in gcode — "
                        "S4 requires relative E mode on the printer.")

    g1_re = re.compile(r"^\s*G1\b", re.IGNORECASE)
    x_re = re.compile(r"\bX-?[\d.]+", re.IGNORECASE)
    y_re = re.compile(r"\bY-?[\d.]+", re.IGNORECASE)
    z_re = re.compile(r"\bZ-?[\d.]+", re.IGNORECASE)
    e_re = re.compile(r"\bE(-?[\d.]+)", re.IGNORECASE)
    # A Z-hop is a Z move with no X/Y travel and no positive extrusion. Detect it
    # independent of any preceding retract (retract makes prev E != 0, which the
    # old heuristic wrongly treated as an all-clear). Report every occurrence.
    # ponytail: may also flag layer-change Z moves; it's a soft advisory, so
    # over-reporting is acceptable — upgrade to Z-delta tracking if it gets noisy.
    for i, line in enumerate(lines):
        if not g1_re.match(line) or not z_re.search(line):
            continue
        if x_re.search(line) or y_re.search(line):
            continue
        em = e_re.search(line)
        if em and float(em.group(1)) > 0:
            continue
        warnings.append(
            f"WARNING: line {i + 1}: possible Z-hop / lift (Z with no X/Y, no "
            f"extrusion): {line.strip()}. S4 requires no Z-hop and no retract lift."
        )

    return warnings
