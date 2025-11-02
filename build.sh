#!/bin/bash
# build.sh — configure & compile the AL5D SSC-32U CLI with CMake
# Usage:
#   ./build.sh                      # Release build (default), uses all CPU cores
#   ./build.sh -t Debug             # Debug build
#   ./build.sh -j 4                 # Use 4 compile jobs
#   ./build.sh -c                   # Clean (delete build/ and bin/) then build
#   ./build.sh -n                   # Configure only (no compile)
#
# Result:
#   - Build files in ./build/
#   - Executable copied/symlinked to ./bin/ssc32u_cli
#   - You can run: ./bin/ssc32u_cli --port /dev/ttyUSB0 --baud 9600

set -euo pipefail

BUILD_TYPE="Release"
JOBS="$(nproc || echo 4)"
CLEAN=0
CONFIG_ONLY=0

while getopts ":t:j:cn" opt; do
  case ${opt} in
    t) BUILD_TYPE="$OPTARG" ;;
    j) JOBS="$OPTARG" ;;
    c) CLEAN=1 ;;
    n) CONFIG_ONLY=1 ;;
    \?) echo "Invalid option: -$OPTARG" >&2; exit 1 ;;
    :)  echo "Option -$OPTARG requires an argument." >&2; exit 1 ;;
  esac
done

need() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Error: '$1' is not installed." >&2
    if [[ "$1" == "cmake" || "$1" == "g++" ]]; then
      echo "Hint: sudo apt update && sudo apt install -y cmake build-essential" >&2
    fi
    exit 1
  fi
}
need cmake
need g++

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BIN_DIR="${ROOT_DIR}/bin"
EXE_NAME="ssc32u_cli"

if [[ $CLEAN -eq 1 ]]; then
  echo "[clean] removing ${BUILD_DIR} and ${BIN_DIR}"
  rm -rf "${BUILD_DIR}" "${BIN_DIR}"
fi

mkdir -p "${BUILD_DIR}" "${BIN_DIR}"

echo "[configure] CMAKE_BUILD_TYPE=${BUILD_TYPE}"
cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

if [[ $CONFIG_ONLY -eq 1 ]]; then
  echo "[done] configuration only (no build)."
  exit 0
fi

echo "[build] compiling with ${JOBS} jobs…"
cmake --build "${BUILD_DIR}" -j "${JOBS}"

if [[ -f "${BUILD_DIR}/${EXE_NAME}" ]]; then
  cp -f "${BUILD_DIR}/${EXE_NAME}" "${BIN_DIR}/${EXE_NAME}"
elif [[ -f "${BUILD_DIR}/${EXE_NAME}.exe" ]]; then
  cp -f "${BUILD_DIR}/${EXE_NAME}.exe" "${BIN_DIR}/${EXE_NAME}"
else
  echo "Error: built executable not found in ${BUILD_DIR}" >&2
  exit 1
fi

echo "[ok] built ${BIN_DIR}/${EXE_NAME}"
echo
echo "Run it like this:"
echo "  ${BIN_DIR}/${EXE_NAME} --port /dev/ttyUSB0 --baud 9600"
