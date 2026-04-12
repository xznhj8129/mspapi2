#!/bin/bash
set -euo pipefail

DEFAULT_BRANCH="maintenance-9.x"
LOCAL_INAV_DIR="../inav"

use_local=0
branch="$DEFAULT_BRANCH"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --local)
            use_local=1
            shift
            ;;
        --branch)
            branch="$2"
            shift 2
            ;;
        -h|--help)
            cat <<EOF
Usage:
  $0 [--local]
  $0 [--branch <name>]

What it does:
  Copies msp_messages.json and inav_enums.json into mspapi2/lib/
  and regenerates mspapi2/lib/inav_version.py from INAV's CMakeLists.txt.

Options:
  --local           Read files from ${LOCAL_INAV_DIR}/docs/development/msp
  --branch <name>   Read files from a remote INAV branch or tag
                    Default: ${DEFAULT_BRANCH}

Examples:
  $0
  $0 --branch maintenance-10.x
  $0 --local
EOF
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "Run '$0 --help' for usage." >&2
            exit 1
            ;;
    esac
done

if [[ "$use_local" -eq 1 ]]; then
    cp "${LOCAL_INAV_DIR}/docs/development/msp/msp_messages.json" mspapi2/lib/msp_messages.json
    cp "${LOCAL_INAV_DIR}/docs/development/msp/inav_enums.json" mspapi2/lib/inav_enums.json
    cmake_text="$(cat "${LOCAL_INAV_DIR}/CMakeLists.txt")"
    source_desc="source=local dir=${LOCAL_INAV_DIR}"
else
    tmp_spec="$(mktemp)"
    tmp_enums="$(mktemp)"
    tmp_cmake="$(mktemp)"
    trap 'rm -f "$tmp_spec" "$tmp_enums" "$tmp_cmake"' EXIT

    resolved_base_url=""
    for candidate in heads tags; do
        base_url="https://raw.githubusercontent.com/iNavFlight/inav/refs/${candidate}/${branch}/docs/development/msp"
        cmake_url="https://raw.githubusercontent.com/iNavFlight/inav/refs/${candidate}/${branch}/CMakeLists.txt"
        if curl -fsL "${base_url}/msp_messages.json" -o "$tmp_spec" 2>/dev/null && \
           curl -fsL "${base_url}/inav_enums.json" -o "$tmp_enums" 2>/dev/null && \
           curl -fsL "${cmake_url}" -o "$tmp_cmake" 2>/dev/null; then
            resolved_base_url="$base_url"
            break
        fi
    done

    if [[ -z "$resolved_base_url" ]]; then
        echo "Failed to fetch msp JSON files for ref='${branch}' as branch or tag" >&2
        exit 1
    fi

    cp "$tmp_spec" mspapi2/lib/msp_messages.json
    cp "$tmp_enums" mspapi2/lib/inav_enums.json
    cmake_text="$(cat "$tmp_cmake")"
    source_desc="source=remote ref=${branch} base_url=${resolved_base_url}"
fi

version_triplet="$(printf '%s\n' "$cmake_text" | sed -nE 's/^[[:space:]]*project\([[:space:]]*INAV[[:space:]]+VERSION[[:space:]]+([0-9]+)\.([0-9]+)\.([0-9]+).*/\1 \2 \3/p' | head -n 1)"
read -r major minor patch <<< "$version_triplet"

cat > mspapi2/lib/inav_version.py <<PY
VERSION = "${major}.${minor}.${patch}"
MAJOR = ${major}
MINOR = ${minor}
PATCH = ${patch}
PY

echo "Updated mspapi2/lib/msp_messages.json, mspapi2/lib/inav_enums.json, and mspapi2/lib/inav_version.py"
echo "${source_desc}"
echo "INAV version: ${major}.${minor}.${patch}"
