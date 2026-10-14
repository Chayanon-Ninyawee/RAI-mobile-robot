#!/usr/bin/env bash

set -Eeuo pipefail

# Usage:
#   ./setup_rpi5.sh [ROS_DOMAIN_ID]
#
# Example:
#   ./setup_rpi5.sh 102

ROS_DOMAIN_ID="${1:-102}"
ROS_APT_SOURCE_API="https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest"
UDEV_FILE="/etc/udev/rules.d/99-ired.rules"
APT_UPGRADES_FILE="/etc/apt/apt.conf.d/20auto-upgrades"
UBUNTU_SOURCES_FILE="/etc/apt/sources.list.d/ubuntu.sources"

# ------------------------------------------------------------
# Checks
# ------------------------------------------------------------

if [[ "$EUID" -eq 0 ]]; then
    echo "Please run this script as your normal user, not with sudo."
    exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
    echo "Error: sudo is required."
    exit 1
fi

if ! [[ "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] ||
   (( ROS_DOMAIN_ID < 0 || ROS_DOMAIN_ID > 232 )); then
    echo "Error: ROS_DOMAIN_ID must be an integer from 0 to 232."
    exit 1
fi

if [[ ! -r /etc/os-release ]]; then
    echo "Error: cannot identify the operating system."
    exit 1
fi

. /etc/os-release

if [[ "${ID:-}" != "ubuntu" ||
      "${VERSION_CODENAME:-}" != "noble" ]]; then
    echo "Error: this script expects Ubuntu 24.04 (Noble)."
    exit 1
fi

if [[ ! -f "$UBUNTU_SOURCES_FILE" ]]; then
    echo "Error: $UBUNTU_SOURCES_FILE does not exist."
    exit 1
fi

if ! id "$USER" >/dev/null 2>&1; then
    echo "Error: cannot identify the current user."
    exit 1
fi

TARGET_USER="$(id -un)"
TARGET_HOME="$(getent passwd "$TARGET_USER" | cut -d: -f6)"

if [[ -z "$TARGET_HOME" || ! -d "$TARGET_HOME" ]]; then
    echo "Error: cannot identify the user's home directory."
    exit 1
fi

echo "=========================================="
echo " iRED Raspberry Pi 5 setup"
echo "=========================================="
echo "User:           $TARGET_USER"
echo "ROS_DOMAIN_ID:  $ROS_DOMAIN_ID"
echo "Ubuntu:         $PRETTY_NAME"
echo
echo "This script modifies system configuration,"
echo "installs ROS packages, and masks sleep targets."
echo "Existing configuration files are backed up"
echo "before being modified."
echo
read -r -p "Press Enter to continue, or Ctrl+C to cancel..."

# ------------------------------------------------------------
# Helper: update a file only when its contents change.
# Backups are stored alongside the original file.
# ------------------------------------------------------------

backup_if_changed() {
    local file="$1"
    local temp="$2"

    if ! sudo cmp -s "$file" "$temp"; then
        if sudo test -e "$file"; then
            sudo cp -a "$file" "${file}.bak.$(date +%Y%m%d%H%M%S)"
        fi
        sudo install -m 0644 "$temp" "$file"
    fi

    rm -f "$temp"
}

# ------------------------------------------------------------
# 1. Disable unattended upgrades
# ------------------------------------------------------------

echo
echo "[1/9] Configuring unattended upgrades..."

TEMP_FILE="$(mktemp)"

sudo cat "$APT_UPGRADES_FILE" > "$TEMP_FILE"

python3 - "$TEMP_FILE" <<'PY'
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()

key = "APT::Periodic::Unattended-Upgrade"
pattern = re.compile(
    r'^\s*APT::Periodic::Unattended-Upgrade\s+.*?;\s*$',
    re.MULTILINE,
)
replacement = f'{key} "0";'

if pattern.search(text):
    text = pattern.sub(replacement, text)
else:
    if text and not text.endswith("\n"):
        text += "\n"
    text += replacement + "\n"

path.write_text(text)
PY

backup_if_changed "$APT_UPGRADES_FILE" "$TEMP_FILE"

# ------------------------------------------------------------
# 2. Mask network-online wait and sleep targets
# ------------------------------------------------------------

echo
echo "[2/9] Masking systemd services and sleep targets..."

sudo systemctl mask \
    systemd-networkd-wait-online.service \
    sleep.target \
    suspend.target \
    hibernate.target \
    hybrid-sleep.target

# ------------------------------------------------------------
# 3. Configure Ubuntu repositories
# ------------------------------------------------------------

echo
echo "[3/9] Enabling noble-updates and noble-backports..."

TEMP_FILE="$(mktemp)"

sudo cat "$UBUNTU_SOURCES_FILE" > "$TEMP_FILE"

python3 - "$TEMP_FILE" <<'PY'
import re
import sys
from pathlib import Path

path = Path(sys.argv[1])
text = path.read_text()
paragraphs = re.split(r"(\n\s*\n)", text)
changed = False

for i in range(0, len(paragraphs), 2):
    block = paragraphs[i]

    match = re.search(r"(?m)^Suites:\s*(.*?)\s*$", block)
    if not match:
        continue

    suites = match.group(1).split()

    # Only modify source stanzas containing the base Noble suite.
    if "noble" not in suites:
        continue

    for suite in ("noble-updates", "noble-backports"):
        if suite not in suites:
            suites.append(suite)
            changed = True

    if changed:
        block = block[:match.start(1)] + " ".join(suites) + block[match.end(1):]
        paragraphs[i] = block

if changed:
    path.write_text("".join(paragraphs))
PY

backup_if_changed "$UBUNTU_SOURCES_FILE" "$TEMP_FILE"

# ------------------------------------------------------------
# 4. Install prerequisites and enable Universe
# ------------------------------------------------------------

echo
echo "[4/9] Installing prerequisites..."

sudo apt-get update
sudo apt-get install -y \
    ca-certificates \
    curl \
    software-properties-common \
    locales \
    python3

sudo add-apt-repository -y universe

# ------------------------------------------------------------
# 5. Configure UTF-8 locale
# ------------------------------------------------------------

echo
echo "[5/9] Configuring UTF-8 locale..."

sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# ------------------------------------------------------------
# 6. Install the official ROS 2 apt source
# ------------------------------------------------------------

echo
echo "[6/9] Configuring the ROS 2 apt repository..."

ROS_APT_SOURCE_VERSION="$(
    curl -fsSL "$ROS_APT_SOURCE_API" |
        python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])'
)"

if [[ -z "$ROS_APT_SOURCE_VERSION" ]]; then
    echo "Error: could not determine the ROS apt source version."
    exit 1
fi

DEB_FILE="$(mktemp --suffix=.deb)"
trap 'rm -f "${DEB_FILE:-}"' EXIT

DEB_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.noble_all.deb"

curl -fL "$DEB_URL" -o "$DEB_FILE"
sudo dpkg -i "$DEB_FILE"

rm -f "$DEB_FILE"
trap - EXIT

# ------------------------------------------------------------
# 7. Install ROS 2 Jazzy and iRED dependencies
# ------------------------------------------------------------

echo
echo "[7/9] Installing ROS 2 Jazzy and dependencies..."

sudo apt-get update
sudo apt-get install -y \
    ros-dev-tools \
    ros-jazzy-ros-base \
    python3-crcmod \
    ros-jazzy-tf-transformations \
    ros-jazzy-xacro \
    ros-jazzy-laser-filters \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

# ------------------------------------------------------------
# 8. Configure the user's ROS environment
# ------------------------------------------------------------

echo
echo "[8/9] Configuring the user's ROS environment..."

sudo -u "$TARGET_USER" python3 - \
    "$TARGET_HOME/ros_environment.bash" \
    "$TARGET_HOME/.bashrc" \
    "$ROS_DOMAIN_ID" <<'PY'
import re
import sys
from pathlib import Path

env_path = Path(sys.argv[1])
bashrc_path = Path(sys.argv[2])
domain_id = sys.argv[3]

def replace_file(path, content):
    old = path.read_text() if path.exists() else ""
    if old == content:
        return

    if path.exists():
        import shutil
        from datetime import datetime
        backup = path.with_name(
            path.name + ".bak." + datetime.now().strftime("%Y%m%d%H%M%S")
        )
        shutil.copy2(path, backup)

    path.write_text(content)

# Keep unrelated environment settings while replacing the
# ROS setup and domain ID lines with one canonical version.
env_text = env_path.read_text() if env_path.exists() else ""

env_text = re.sub(
    r"(?m)^\s*source\s+/opt/ros/jazzy/setup\.bash\s*\n?",
    "",
    env_text,
)
env_text = re.sub(
    r"(?m)^\s*export\s+ROS_DOMAIN_ID=.*\n?",
    "",
    env_text,
)
env_text = env_text.rstrip()

managed = (
    "# iRED ROS environment\n"
    "source /opt/ros/jazzy/setup.bash\n"
    f"export ROS_DOMAIN_ID={domain_id}\n"
)

replace_file(
    env_path,
    (env_text + "\n\n" if env_text else "") + managed,
)

# Add one source line to .bashrc without duplicating it.
bashrc_text = bashrc_path.read_text() if bashrc_path.exists() else ""

bashrc_text = re.sub(
    r"(?m)^\s*(?:source|\.)\s+"
    r"(?:\"\$HOME/ros_environment\.bash\"|"
    r"'\$HOME/ros_environment\.bash'|"
    r"\$HOME/ros_environment\.bash|"
    r"~/ros_environment\.bash)\s*\n?",
    "",
    bashrc_text,
)

bashrc_text = bashrc_text.rstrip()
source_line = 'source "$HOME/ros_environment.bash"'

replace_file(
    bashrc_path,
    (bashrc_text + "\n\n" if bashrc_text else "") + source_line + "\n",
)
PY

# ------------------------------------------------------------
# 9. Configure USB device aliases
# ------------------------------------------------------------

echo
echo "[9/9] Configuring USB serial devices..."

TEMP_FILE="$(mktemp)"

if sudo test -f "$UDEV_FILE"; then
    sudo cat "$UDEV_FILE" > "$TEMP_FILE"
fi

python3 - "$TEMP_FILE" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
lines = path.read_text().splitlines() if path.exists() else []

# Replace only the two rules managed by this script.
lines = [
    line for line in lines
    if 'SYMLINK+="ttyMicro"' not in line
    and 'SYMLINK+="ttyLiDAR"' not in line
]

lines.extend([
    'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="iredbyaimslab2467523", GROUP="dialout", MODE="0660", SYMLINK+="ttyMicro"',
    'SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", GROUP="dialout", MODE="0660", SYMLINK+="ttyLiDAR"',
])

path.write_text("\n".join(lines) + "\n")
PY

backup_if_changed "$UDEV_FILE" "$TEMP_FILE"

if ! getent group dialout >/dev/null; then
    sudo groupadd dialout
fi

sudo usermod -aG dialout "$TARGET_USER"

sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty

# ------------------------------------------------------------
# Finish
# ------------------------------------------------------------

echo
echo "=========================================="
echo " iRED setup complete"
echo "=========================================="
echo
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS environment: $TARGET_HOME/ros_environment.bash"
echo "USB aliases: /dev/ttyMicro and /dev/ttyLiDAR"
echo
echo "Next steps:"
echo "  1. Log out and back in to apply dialout group membership."
echo "  2. Run: source ~/ros_environment.bash"
echo "  3. Check: echo \$ROS_DOMAIN_ID"
echo "  4. Check USB devices: ls -l /dev/ttyMicro /dev/ttyLiDAR"
echo "  5. Reboot if you want to verify the systemd changes."
