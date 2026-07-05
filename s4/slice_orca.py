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


def slice_stl(stl_path, out_dir, orca_binary, orca_config=None,
              extra_args=None, timeout=600):
    os.makedirs(out_dir, exist_ok=True)
    args = [orca_binary] + ORCA_SLICE_ARGS
    if orca_config:
        args += ["--load-settings", orca_config]
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

    return str(max(fresh, key=lambda p: p.stat().st_mtime))


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
