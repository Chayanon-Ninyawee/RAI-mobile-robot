#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

if [[ -z "${DISPLAY:-}" ]]; then
    echo "ERROR: DISPLAY is not set. Run this from your KDE desktop terminal."
    exit 1
fi

if ! command -v xhost >/dev/null 2>&1; then
    echo "ERROR: xhost is missing. Install the Fedora package providing xhost."
    exit 1
fi

# Allow the root process in this container to access the XWayland display.
xhost +si:localuser:root

if ! docker compose up -d --build; then
    xhost -si:localuser:root || true
    exit 1
fi

docker compose exec client bash
