#!/bin/bash
# Build and install dji-rc-joystick into ~/.local (no root needed for the
# build; the udev rule at the end does ask for sudo, and is optional).
set -euo pipefail

SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${PREFIX:-$HOME/.local}"
LIBDIR="$PREFIX/lib/dji-rc-linux"
BINDIR="$PREFIX/bin"

echo "=== dji-rc-joystick install ==="

for tool in cmake make cc; do
    command -v "$tool" >/dev/null || { echo "missing build tool: $tool"; exit 1; }
done

echo "--> building"
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$SRC/build" >/dev/null
echo "    ok"

echo "--> installing to $PREFIX"
install -Dm755 "$SRC/build/dji-rc-joystick" "$LIBDIR/dji-rc-joystick"
install -Dm755 "$SRC/bin/dji-rc-start"      "$BINDIR/dji-rc-start"
ln -sf "$LIBDIR/dji-rc-joystick" "$BINDIR/dji-rc-joystick"
echo "    $BINDIR/dji-rc-start"
echo "    $BINDIR/dji-rc-joystick"

case ":$PATH:" in
    *":$BINDIR:"*) ;;
    *) echo
       echo "NOTE: $BINDIR is not in your PATH. Add it to your shell profile:"
       echo "      export PATH=\"\$PATH:$BINDIR\"" ;;
esac

# Desktop entry, if there is a desktop to put it on
APPS="${XDG_DATA_HOME:-$HOME/.local/share}/applications"
mkdir -p "$APPS"
cat > "$APPS/dji-rc-joystick.desktop" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=DJI RC Joystick
Comment=Use a DJI RC smart controller as a gamepad
Exec=$BINDIR/dji-rc-start
Icon=input-gaming
Terminal=true
Categories=Utility;
Keywords=DJI;drone;joystick;gamepad;FPV;
DESKTOP
command -v update-desktop-database >/dev/null && update-desktop-database "$APPS" 2>/dev/null || true
echo "    desktop entry installed"

echo
if [ -w /dev/uinput ]; then
    echo "--> /dev/uinput is already writable, nothing to do."
else
    echo "--> /dev/uinput is not writable by you."
    echo "    Without it the virtual joystick can only be created as root."
    read -rp "    Install the udev rule now (needs sudo)? [y/N] " a
    case "$a" in
        [yYjJ]*)
            sudo install -Dm644 "$SRC/dist/99-dji-rc-uinput.rules" \
                 /etc/udev/rules.d/99-dji-rc-uinput.rules
            sudo install -Dm644 "$SRC/dist/uinput.conf" \
                 /etc/modules-load.d/uinput.conf
            sudo udevadm control --reload-rules
            sudo modprobe uinput || true
            sudo udevadm trigger /dev/uinput 2>/dev/null || true
            if [ -w /dev/uinput ]; then
                echo "    ok, /dev/uinput is writable now."
            else
                echo "    installed, but still not writable - log out and back in."
            fi ;;
        *)  echo "    skipped. See dist/99-dji-rc-uinput.rules to do it later." ;;
    esac
fi

echo
echo "Done. Power on the controller, connect it to your WiFi, then run:"
echo "    dji-rc-start"
