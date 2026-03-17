#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_SH_VERSION="2026-03-03"

echo "[build.sh] version: ${BUILD_SH_VERSION}"
echo "[build.sh] script_dir: ${SCRIPT_DIR}"
echo "[build.sh] root_dir:   ${ROOT_DIR}"

die() {
    echo "[build.sh] ERROR: $*" >&2
    exit 1
}

need_cmd() {
    command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

append_env_path() {
    local var_name="$1"
    local dir="$2"
    local current_value="${!var_name-}"
    if [[ -z "${current_value}" ]]; then
        export "${var_name}=${dir}"
    else
        export "${var_name}=${current_value}:${dir}"
    fi
}

process_3rd_library() {
    local list=("$@")
    for item in "${list[@]}"; do
        local pkg_root="$HOME/.conan/data/$item"
        [[ -d "${pkg_root}" ]] || continue

        local sub_dirs
        sub_dirs=$(find "${pkg_root}/" -mindepth 1 -maxdepth 1 -type d 2>/dev/null || true)
        for sub_item in ${sub_dirs}; do
            local sub_packages
            sub_packages=$(find "${sub_item}/_/_/package" -mindepth 1 -maxdepth 1 -type d 2>/dev/null || true)
            for sub_pack in ${sub_packages}; do
                [[ -d "${sub_pack}/lib" ]] || continue
                append_env_path LIBRARY_PATH "${sub_pack}/lib"
                append_env_path LD_LIBRARY_PATH "${sub_pack}/lib"
            done
        done
    done
}

need_cmd conan
need_cmd catkin_make

[[ -d "${ROOT_DIR}/3rd" ]] || die "${ROOT_DIR}/3rd not found. Are you running the correct repo copy?"
[[ -f "${ROOT_DIR}/3rd/conanfile.py" ]] || die "${ROOT_DIR}/3rd/conanfile.py not found. 3rd/ is incomplete."
[[ -f "${ROOT_DIR}/src/CMakeLists.txt" ]] || die "${ROOT_DIR}/src/CMakeLists.txt not found. This doesn't look like a catkin workspace root."

cd "${ROOT_DIR}/3rd"
conan install . --build=missing -s compiler.libcxx=libstdc++11
process_3rd_library "gflags" "glog" "ceres-solver" "osqp" "libunwind"
cd "${ROOT_DIR}"
catkin_make