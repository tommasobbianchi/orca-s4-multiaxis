#!/usr/bin/env python3
"""Watch an input directory for new/changed .stl files, run the S4 pipeline.

Usage:
    python watch.py --in INDIR --out OUTDIR [--config config.yaml]

# ponytail: naive single-threaded watcher, upgrade to a queue if throughput matters.
"""

import argparse
import logging
import sys
import time
from pathlib import Path

log = logging.getLogger("watch")


def _wait_settle(path, checks=2, interval=1.0, max_wait=60):
    """Wait until the file size is stable across consecutive checks, so we never
    slice a half-written STL. Returns False if the file vanished."""
    last = -1
    stable = 0
    for _ in range(int(max_wait / interval)):
        try:
            size = path.stat().st_size
        except FileNotFoundError:
            return False
        if size == last and size > 0:
            stable += 1
            if stable >= checks:
                return True
        else:
            stable = 0
        last = size
        time.sleep(interval)
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Watch folder for .stl files and run the S4 pipeline"
    )
    parser.add_argument("--in", dest="indir", required=True, help="Input directory to watch")
    parser.add_argument("--out", dest="outdir", required=True, help="Output directory for gcode files")
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s")

    try:
        from watchdog.observers import Observer
        from watchdog.events import FileSystemEventHandler
    except ImportError:
        sys.exit("watchdog not installed. Run: pip install watchdog")

    indir = Path(vars(args)["indir"]).resolve()
    outdir = Path(args.outdir).resolve()
    outdir.mkdir(parents=True, exist_ok=True)
    if indir == outdir:
        log.warning("Input and output dirs are the same; intermediate files are "
                    "skipped to avoid a reprocessing loop.")

    processed = {}

    class Handler(FileSystemEventHandler):
        def on_any_event(self, event):
            path = Path(event.src_path)
            if path.suffix.lower() != ".stl":
                return
            if event.event_type not in ("created", "modified"):
                return
            # Never reprocess our own intermediate (deformed.stl) — it would loop
            # when indir == outdir.
            if path.stem == "deformed":
                return
            log.info("Detected %s: %s", event.event_type, path)
            if not _wait_settle(path):
                log.warning("File %s did not settle, skipping", path)
                return
            # Debounce: skip if we already processed this exact size (the OS emits
            # several modified events per copy).
            try:
                sig = path.stat().st_size
            except FileNotFoundError:
                return
            if processed.get(str(path)) == sig:
                return
            processed[str(path)] = sig
            try:
                from s4.pipeline import run
                out_path = outdir / (path.stem + ".gcode")
                run(str(path), str(out_path), config_path=args.config)
            except Exception:
                log.exception("Pipeline failed for %s", path)

    observer = Observer()
    observer.schedule(Handler(), str(indir), recursive=False)
    observer.start()
    log.info("Watching %s -> %s (config=%s)", indir, outdir, args.config)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()


if __name__ == "__main__":
    main()
