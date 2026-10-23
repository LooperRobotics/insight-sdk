#!/usr/bin/env bash
# Build the Insight 9 macOS SDK and example.
#   bash build.sh                          # Release build into ./build
#   BUILD_TYPE=Debug bash build.sh         # Debug build
#   bash build.sh -DINSIGHT9_DEBUG_LOG=ON  # extra CMake options are passed through
set -euo pipefail

cd "$(dirname "$0")"
BUILD_TYPE="${BUILD_TYPE:-Release}"
SRC_DIR="$(pwd -P)"

# A terminal left inside a folder that was replaced (Finder moves the old one
# to the Trash) silently builds the old sources.
case "${SRC_DIR}" in
    */.Trash/*)
        echo "[ERROR] This MAC-SDK folder is in the Trash: ${SRC_DIR}" >&2
        echo "        The terminal is still in the old folder. cd into the current MAC-SDK and rerun." >&2
        exit 1
        ;;
esac

# A build/ directory copied or moved together with the sources still points at
# the old location, and CMake refuses to use it. Recreate it in that case.
if [ -f build/CMakeCache.txt ]; then
    cached_src="$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' build/CMakeCache.txt)"
    if [ "${cached_src}" != "${SRC_DIR}" ] && [ "${cached_src}" != "$(pwd)" ]; then
        echo "[INFO] build/ was configured for ${cached_src}; recreating it for ${SRC_DIR}"
        rm -rf build
    fi
fi

echo "========================================"
echo "Building Insight9 SDK (macOS, ${BUILD_TYPE})"
echo "========================================"

if ! command -v brew >/dev/null 2>&1; then
    echo "[ERROR] Homebrew not found. Install the dependencies first:" >&2
    echo "        bash install_deps.sh" >&2
    exit 1
fi

missing=()
for formula in cmake pkg-config libusb; do
    brew list --versions "${formula}" >/dev/null 2>&1 || missing+=("${formula}")
done
if [ "${#missing[@]}" -ne 0 ]; then
    echo "[ERROR] Missing Homebrew packages: ${missing[*]}" >&2
    echo "        bash install_deps.sh" >&2
    exit 1
fi
if ! brew list --versions opencv >/dev/null 2>&1; then
    echo "[WARN] opencv not installed: the SDK library builds, the example is skipped."
    echo "       bash install_deps.sh   (or: brew install opencv)"
fi

BREW_PREFIX="$(brew --prefix)"
extra_args=()
# Pin OpenCV to Homebrew's copy (lib/cmake/opencv5 or opencv4). A second
# OpenCV (conda, a manual install in /usr/local, an old Cellar version) can
# otherwise supply headers that do not match the library, which fails at link
# time with undefined cv:: symbols.
OPENCV_PREFIX="$(brew --prefix opencv 2>/dev/null || true)"
for dir in "${OPENCV_PREFIX}/lib/cmake/opencv5" "${OPENCV_PREFIX}/lib/cmake/opencv4"; do
    if [ -n "${OPENCV_PREFIX}" ] && [ -f "${dir}/OpenCVConfig.cmake" ]; then
        extra_args+=("-DOpenCV_DIR=${dir}")
        echo "Using OpenCV: ${dir}"
        break
    fi
done

# Include paths from the environment take precedence over CMake's and can
# point at another OpenCV; build without them.
for var in CPATH C_INCLUDE_PATH CPLUS_INCLUDE_PATH; do
    if [ -n "${!var:-}" ]; then
        echo "[WARN] Ignoring ${var}=${!var} for this build"
        unset "${var}"
    fi
done

cmake -S . -B build \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DCMAKE_PREFIX_PATH="${BREW_PREFIX}" \
    "${extra_args[@]+"${extra_args[@]}"}" \
    "$@"
cmake --build build -j "$(sysctl -n hw.ncpu)"

echo
echo "========================================"
echo "Build completed successfully!"
echo "========================================"
echo "SDK library : build/libinsight9.a (or .dylib with -DBUILD_SHARED_LIBS=ON)"
if [ -x build/example ]; then
    echo "Example     : build/example"
    echo
    echo "Run it (sudo is required to take the camera over from macOS):"
    echo "  sudo ./build/example            # OpenCV window"
    echo "  sudo ./build/example --no-gui   # console statistics only"
fi
