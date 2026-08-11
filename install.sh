#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON311="/usr/bin/python3.11"

echo "=== Relative Localisation Tutorial — Setup ==="
echo ""

# --- Pre-flight checks ---
if ! dpkg -s libeigen3-dev &>/dev/null; then
  echo "[MISSING] libeigen3-dev: sudo apt-get install -y libeigen3-dev"
fi

if ! python3.11 -c "import pybind11" 2>/dev/null; then
  echo "[MISSING] pybind11[global]: $PYTHON311 -m pip install \"pybind11[global]\""
fi

# If any check failed, exit early with instructions
if ! dpkg -s libeigen3-dev &>/dev/null || ! python3.11 -c "import pybind11" 2>/dev/null; then
  echo ""
  echo "Install the missing packages above, then re-run this script."
  exit 1
fi

echo "[OK]   All pre-flight checks passed"
echo ""

# --- Submodule init ---
echo "> Initializing git submodule ..."
git config --global --add safe.directory "$SCRIPT_DIR" 2>/dev/null || true
git submodule update --init libs/manif

# --- Venv creation (py3.11) ---
if [ -d "$SCRIPT_DIR/.venv" ] && [ -x "$SCRIPT_DIR/.venv/bin/python" ]; then
  source "$SCRIPT_DIR/.venv/bin/activate"
  echo "> .venv: using existing ($(python --version))"
else
  echo "> Creating .venv with Python 3.11 ..."
  "$PYTHON311" -m venv "$SCRIPT_DIR/.venv"
  source "$SCRIPT_DIR/.venv/bin/activate"
  echo "Python: $(python --version)"
fi

# --- Upgrade pip, setuptools, wheel ---
pip install --upgrade pip setuptools wheel

# --- Compile & install manifpy from local submodule ---
echo "> Installing mani fpy (CMake build) from libs/manif ..."
pip install "./libs/manif[docs]"

# --- Install arp_utility_code tarball as-is (no patching) ---
TARBALL="$SCRIPT_DIR/libs/arp_utility_code-0.1.0.tar.gz"
if [ ! -f "$TARBALL" ]; then
  echo "ERROR: Tarball not found at $TARBALL"
  exit 1
fi

echo "> Installing arp_utility_code from tarball ..."
pip install "$TARBALL"

# --- Install dev dependencies ---
echo "> Installing dev deps (jupyter, pytest, ipykernel) ..."
pip install jupyter pytest ipykernel

# --- Verification ---
echo ""
echo "> Verifying imports ..."
python -c "from manifpy import SE3; print('  mani fpy    : OK  ')"
python -c "import arp_utility_code; print('  arp_utility_code: OK')"

echo ""
echo "=== Done! ==="
echo "Activate your environment whenever you need it:"
echo "    source $SCRIPT_DIR/.venv/bin/activate"
