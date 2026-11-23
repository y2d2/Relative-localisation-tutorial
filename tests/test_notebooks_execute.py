"""Smoke test: execute every tutorial notebook end-to-end.

Each ``notebooks/part_*.ipynb`` is read into memory and executed top-to-bottom
with ``nbclient``. The executed copy is never written back to disk, so the
committed notebooks (and their outputs) stay untouched. The tests carry the
``notebooks`` marker; skip them with ``pytest -m "not notebooks"``.
"""

from pathlib import Path

import nbformat
import pytest
from nbclient import NotebookClient
from nbclient.exceptions import CellExecutionError

pytestmark = pytest.mark.notebooks

NOTEBOOK_DIR = Path(__file__).resolve().parents[1].joinpath("notebooks")
NOTEBOOKS = sorted(NOTEBOOK_DIR.glob("part_*.ipynb"))
CELL_TIMEOUT_S = 1800
MIN_NOTEBOOKS = 4


def execute_notebook(nb: nbformat.NotebookNode, cwd: Path) -> nbformat.NotebookNode:
    """Execute an in-memory notebook with the ``python3`` kernel.

    Args:
        nb: Notebook node to execute; it is modified in place with outputs.
        cwd: Working directory for the kernel (notebooks use ``../src``).

    Returns:
        The executed notebook node (same object as ``nb``).

    Raises:
        CellExecutionError: If any cell raises an exception.
    """
    client = NotebookClient(
        nb,
        timeout=CELL_TIMEOUT_S,
        kernel_name="python3",
        resources={"metadata": {"path": str(cwd)}},
    )
    client.execute()
    return nb


def count_png_outputs(nb: nbformat.NotebookNode) -> int:
    """Count ``image/png`` outputs across all code cells.

    Args:
        nb: Executed notebook node.

    Returns:
        Number of outputs containing an ``image/png`` payload.
    """
    return sum(
        1
        for cell in nb.cells
        if cell.cell_type == "code"
        for out in cell.get("outputs", [])
        if "image/png" in out.get("data", {})
    )


def test_notebooks_discovered() -> None:
    """Guard: the glob must find all tutorial notebooks (parametrisation is not empty)."""
    assert len(NOTEBOOKS) >= MIN_NOTEBOOKS, (
        f"expected >= {MIN_NOTEBOOKS} notebooks in {NOTEBOOK_DIR}, found {len(NOTEBOOKS)}"
    )


@pytest.mark.parametrize("path", NOTEBOOKS, ids=[p.stem for p in NOTEBOOKS])
def test_notebook_executes(path: Path) -> None:
    """Execute one notebook in memory and check it runs cleanly and plots.

    Args:
        path: Path to the notebook file under ``notebooks/``.
    """
    nb = nbformat.read(path, as_version=4)
    try:
        execute_notebook(nb, path.parent)
    except CellExecutionError as exc:
        pytest.fail(f"{path.name} failed to execute:\n{exc}")
    assert count_png_outputs(nb) >= 1, f"{path.name} produced no image/png output"
