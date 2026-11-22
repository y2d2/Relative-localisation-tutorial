"""Export every embedded PNG figure from the tutorial notebooks.

Walks ``part_*.ipynb`` notebooks (sorted by name), decodes every
``image/png`` output of every code cell in order and writes it to
``<out-dir>/part_<N>_fig_<K>.png`` where ``K`` is a 1-based running index
per notebook. Existing files are overwritten.

Usage:
    python scripts/export_notebook_figures.py [--notebooks-dir notebooks]
        [--out-dir docs/images]

Exit status is non-zero if any notebook contains zero images.
"""

from __future__ import annotations

import argparse
import base64
import re
import sys
from pathlib import Path

import nbformat

_PART_RE = re.compile(r"part_(\d+)\.ipynb$")


def export_notebook(nb_path: Path, out_dir: Path) -> list[tuple[str, int]]:
    """Write all PNG outputs of one notebook to ``out_dir``.

    Args:
        nb_path: Path to a ``part_<N>.ipynb`` notebook.
        out_dir: Directory where ``part_<N>_fig_<K>.png`` files are written.

    Returns:
        List of ``(filename, cell_index)`` tuples, one per exported figure,
        where ``cell_index`` is the 0-based index of the cell in the notebook.

    Raises:
        ValueError: If the notebook filename does not match ``part_<N>.ipynb``.
    """
    match = _PART_RE.search(nb_path.name)
    if match is None:
        raise ValueError(f"Unexpected notebook name: {nb_path.name}")
    part = match.group(1)

    nb = nbformat.read(nb_path, as_version=4)
    exported: list[tuple[str, int]] = []
    for cell_idx, cell in enumerate(nb.cells):
        if cell.cell_type != "code":
            continue
        for output in cell.get("outputs", []):
            png = output.get("data", {}).get("image/png")
            if png is None:
                continue
            if isinstance(png, list):
                png = "".join(png)
            fname = f"part_{part}_fig_{len(exported) + 1}.png"
            (out_dir / fname).write_bytes(base64.b64decode(png))
            exported.append((fname, cell_idx))
    return exported


def main(argv: list[str] | None = None) -> int:
    """Run the CLI.

    Args:
        argv: Optional argument list (defaults to ``sys.argv[1:]``).

    Returns:
        Process exit code: 0 on success, 1 if any notebook had no images
        or no notebooks were found.
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--notebooks-dir", type=Path, default=Path("notebooks"))
    parser.add_argument("--out-dir", type=Path, default=Path("docs/images"))
    args = parser.parse_args(argv)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    notebooks = sorted(args.notebooks_dir.glob("part_*.ipynb"))
    if not notebooks:
        print(f"No part_*.ipynb found in {args.notebooks_dir}", file=sys.stderr)
        return 1

    status = 0
    for nb_path in notebooks:
        exported = export_notebook(nb_path, args.out_dir)
        if not exported:
            print(f"ERROR: {nb_path.name} contains no image/png outputs", file=sys.stderr)
            status = 1
        for fname, cell_idx in exported:
            print(f"{args.out_dir / fname} -> {nb_path.name} cell {cell_idx}")
    return status


if __name__ == "__main__":
    sys.exit(main())
