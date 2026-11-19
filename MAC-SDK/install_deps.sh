#!/usr/bin/env bash
# Install everything needed to build the Insight 9 macOS SDK and example.
#   bash install_deps.sh              # Xcode CLT + Homebrew + cmake/pkg-config/libusb/opencv
#   bash install_deps.sh --cn-mirror  # use the Tsinghua (TUNA) Homebrew mirror (mainland China)
#   bash install_deps.sh --no-opencv  # library only, skip OpenCV (the example is not built)
#   bash install_deps.sh --build      # also run build.sh when done
# Run it as your normal user, not with sudo (Homebrew refuses to run as root).
set -euo pipefail

cd "$(dirname "$0")"

USE_MIRROR=0
WITH_OPENCV=1
RUN_BUILD=0
for arg in "$@"; do
    case "${arg}" in
        --cn-mirror) USE_MIRROR=1 ;;
        --no-opencv) WITH_OPENCV=0 ;;
        --build)     RUN_BUILD=1 ;;
        -h|--help)   sed -n '2,7p' "$0"; exit 0 ;;
        *) echo "[ERROR] Unknown option: ${arg}" >&2; sed -n '2,7p' "$0" >&2; exit 1 ;;
    esac
done

step() { echo; echo "==> $*"; }
ok()   { echo "    [OK] $*"; }

if [ "$(uname -s)" != "Darwin" ]; then
    echo "[ERROR] This script is for macOS." >&2
    exit 1
fi
if [ "$(id -u)" -eq 0 ]; then
    echo "[ERROR] Do not run this script with sudo; run it as your normal user." >&2
    echo "        (sudo is only needed later to run the example.)" >&2
    exit 1
fi

if [ "${USE_MIRROR}" -eq 1 ]; then
    export HOMEBREW_API_DOMAIN="https://mirrors.tuna.tsinghua.edu.cn/homebrew-bottles/api"
    export HOMEBREW_BOTTLE_DOMAIN="https://mirrors.tuna.tsinghua.edu.cn/homebrew-bottles"
    export HOMEBREW_BREW_GIT_REMOTE="https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/brew.git"
    export HOMEBREW_CORE_GIT_REMOTE="https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/homebrew-core.git"
    export HOMEBREW_PIP_INDEX_URL="https://pypi.tuna.tsinghua.edu.cn/simple"
fi

# ---------------------------------------------------------------- 1. Xcode CLT
step "Xcode Command Line Tools (clang, make, git)"
if xcode-select -p >/dev/null 2>&1; then
    ok "installed at $(xcode-select -p)"
else
    echo "    Opening the macOS installer; click \"Install\" in the dialog that appears."
    xcode-select --install >/dev/null 2>&1 || true
    echo -n "    Waiting for the installation to finish"
    for _ in $(seq 1 180); do                       # up to 30 minutes
        if xcode-select -p >/dev/null 2>&1; then break; fi
        echo -n "."
        sleep 10
    done
    echo
    if ! xcode-select -p >/dev/null 2>&1; then
        echo "[ERROR] Command Line Tools are still not installed. Finish the installer and rerun this script." >&2
        exit 1
    fi
    ok "installed"
fi

# ---------------------------------------------------------------- 2. Homebrew
step "Homebrew"
find_brew() {
    for b in /opt/homebrew/bin/brew /usr/local/bin/brew; do
        if [ -x "${b}" ]; then echo "${b}"; return 0; fi
    done
    command -v brew 2>/dev/null || return 1
}

BREW="$(find_brew || true)"
if [ -z "${BREW}" ]; then
    echo "    Installing Homebrew (it will ask for your login password)..."
    if [ "${USE_MIRROR}" -eq 1 ]; then
        tmp="$(mktemp -d)"
        git clone --depth=1 https://mirrors.tuna.tsinghua.edu.cn/git/homebrew/install.git "${tmp}/brew-install"
        /bin/bash "${tmp}/brew-install/install.sh"
        rm -rf "${tmp}"
    else
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    fi
    BREW="$(find_brew || true)"
    if [ -z "${BREW}" ]; then
        echo "[ERROR] Homebrew installation failed." >&2
        exit 1
    fi
fi
eval "$("${BREW}" shellenv)"
ok "$(brew --version | head -n 1) at $(brew --prefix)"

# Make brew available in new terminals (zsh is the default shell on macOS).
SHELLENV_LINE="eval \"\$(${BREW} shellenv)\""
PROFILE="${HOME}/.zprofile"
if ! grep -qsF "${SHELLENV_LINE}" "${PROFILE}"; then
    printf '\n%s\n' "${SHELLENV_LINE}" >> "${PROFILE}"
    ok "added Homebrew to ${PROFILE}"
fi
if [ "${USE_MIRROR}" -eq 1 ] && ! grep -qs "HOMEBREW_BOTTLE_DOMAIN" "${PROFILE}"; then
    {
        echo 'export HOMEBREW_API_DOMAIN="https://mirrors.tuna.tsinghua.edu.cn/homebrew-bottles/api"'
        echo 'export HOMEBREW_BOTTLE_DOMAIN="https://mirrors.tuna.tsinghua.edu.cn/homebrew-bottles"'
    } >> "${PROFILE}"
    ok "added the TUNA mirror settings to ${PROFILE}"
fi

# ---------------------------------------------------------------- 3. Packages
step "Build dependencies"
packages=(cmake pkg-config libusb)
if [ "${WITH_OPENCV}" -eq 1 ]; then packages+=(opencv); fi

missing=()
for pkg in "${packages[@]}"; do
    if brew list --versions "${pkg}" >/dev/null 2>&1; then
        ok "${pkg} $(brew list --versions "${pkg}" | awk '{print $2}')"
    else
        missing+=("${pkg}")
    fi
done
if [ "${#missing[@]}" -ne 0 ]; then
    echo "    Installing: ${missing[*]} (OpenCV is large and can take a while)"
    brew install "${missing[@]}"
fi

# ---------------------------------------------------------------- 4. Verify
step "Verifying"
fail=0
check() {
    if eval "$2" >/dev/null 2>&1; then ok "$1"; else echo "    [FAIL] $1" >&2; fail=1; fi
}
check "clang++ $(clang++ --version 2>/dev/null | head -n 1 | sed 's/.*version \([0-9.]*\).*/\1/')" "clang++ --version"
check "cmake $(cmake --version 2>/dev/null | head -n 1 | awk '{print $3}')" "cmake --version"
check "libusb $(pkg-config --modversion libusb-1.0 2>/dev/null) (>= 1.0.25 required)" \
      "pkg-config --atleast-version=1.0.25 libusb-1.0"
if [ "${WITH_OPENCV}" -eq 1 ]; then
    opencv_cfg="$(ls -d "$(brew --prefix opencv)"/lib/cmake/opencv* 2>/dev/null | head -n 1 || true)"
    check "OpenCV $(brew list --versions opencv | awk '{print $2}') (${opencv_cfg:-config not found})" "test -n '${opencv_cfg}'"
fi
if [ "${fail}" -ne 0 ]; then
    echo
    echo "[ERROR] Some dependencies are not usable; see [FAIL] above." >&2
    exit 1
fi

echo
echo "========================================"
echo "All dependencies are installed."
echo "========================================"
if [ "${RUN_BUILD}" -eq 1 ]; then
    bash ./build.sh
else
    echo "Next:"
    echo "  bash build.sh"
    echo "  sudo ./build/example"
fi
