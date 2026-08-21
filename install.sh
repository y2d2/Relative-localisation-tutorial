#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON311="/usr/bin/python3.11"

echo "=== Relative Localisation Tutorial — Setup ==="
echo ""

# --- Pre-flight checks ---
if ! dpkg -s libeigen3-dev &>/dev/null; then
  echo "[MISSING] libeigen3-dev: sudo apt-get install -y libeigen3-dev python3.11-dev  build-essential cmake ninja-build"
fi
sudo apt-get install -y libeigen3-dev python3.11-dev  build-essential cmake ninja-build
if ! python3.11 -c "import pybind11" 2>/dev/null; then
  echo "[MISSING] pybind11[global]: $PYTHON311 -m pip install \"pybind11[global]\""
fi
$PYTHON311 -m pip install "pybind11[global]"
# If any check failed, exit early with instructions


echo "[OK]   All pre-flight checks passed"
echo ""

# --- Submodule init ---
echo "> Initializing git submodule ..."
git config --global --add safe.directory "$SCRIPT_DIR" 2>/dev/null || true
git submodule update --init libs/manif
echo $SCRIPT_DIR
# --- Venv creation (py3.11) ---
if [ -d "$SCRIPT_DIR/.venv" ] && [ -x "$SCRIPT_DIR/.venv/bin/python" ]; then
  source "$SCRIPT_DIR/.venv/bin/activate"
  echo "> .venv: using existing ($(python3 --version))"
else
  echo "> Creating .venv with Python 3.11 ..."
  echo $PYTHON311
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

# --- Extract and install upf_rte (requires code conversion) ---
TARBALL_UPF="$SCRIPT_DIR/libs/upf_rpe-0.0.1.tar.gz"
if [ ! -f "$TARBALL_UPF" ]; then
  echo "ERROR: Tarball not found at $TARBALL_UPF"
  exit 1
fi

echo "> Installing upf_rte from tarball (with code conversion) ..."

# Extract to temp directory
TEMP_DIR="$SCRIPT_DIR/libs/tmp_upf_install"
rm -rf "$TEMP_DIR"
mkdir -p "$TEMP_DIR"
tar -xzf "$TARBALL_UPF" -C "$TEMP_DIR"

# Convert Code/ package structure to src/upf_rte/
UPF_SRC="$SCRIPT_DIR/libs/upf_rte/src/upf_rte"
rm -rf "$(dirname $UPF_SRC)"
mkdir -p "$UPF_SRC"
cp -r "$TEMP_DIR/upf_rpe-0.0.1/Code/"* "$UPF_SRC"

# Replace Code References with upf_rte
find "$UPF_SRC" -name "*.py" | xargs sed -i 's/from Code\./from upf_rte./g'
find "$UPF_SRC" -name "*.py" | xargs sed -i 's/import Code\./import upf_rte./g'

# Install the converted package
pip install "$SCRIPT_DIR/libs/upf_rte"

# Cleanup
rm -rf "$TEMP_DIR"

# --- Install dev dependencies ---
echo "> Installing dev deps (jupyter, pytest, ipykernel) ..."
pip install jupyter pytest ipykernel

# --- Verification ---
echo ""
echo "> Verifying imports ..."
python -c "from manifpy import SE3; print('  mani fpy    : OK  ')"
python -c "import arp_utility_code; print('  arp_utility_code: OK')"
python -c "from upf_rte.ParticleFilter.ConnectedAgentClass import UPFConnectedAgent; print('  upf_rte     : OK')"

echo ""
echo "=== Done! ==="
echo "Activate your environment whenever you need it:"
echo "    source $SCRIPT_DIR/.venv/bin/activate"
