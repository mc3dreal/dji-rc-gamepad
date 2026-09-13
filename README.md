# dji-rc-gamepad

Use a **DJI RC smart controller as a gamepad on Linux.** The controller
exposes an undocumented DUML service over WiFi; this connects to it and
publishes the sticks, wheels and buttons as a standard virtual joystick via
`uinput`. Works with any game, simulator or application that reads joysticks.

No ADB, no developer options, no root on the controller, no modified
firmware.

> Fork of [stiad/dji-rc-linux](https://github.com/stiad/dji-rc-linux) (MIT).
> See [Changes from upstream](#changes-from-upstream).
> The built binary keeps the upstream name `dji-rc-joystick`.

## Supported hardware

**Confirmed working**

- DJI RC (RM330)

**Likely compatible** — same Android/DUML platform, untested

- DJI RC 2
- DJI RC Pro (RM510)
- DJI Smart Controller (RM500)
- DJI RC Pro Enterprise

If you try another model, please open an issue either way.

## Why WiFi and not USB

Over USB these controllers expose an ADB interface, but it always reports
`unauthorized`: DJI removed the version screen from the Android settings, so
developer options cannot be enabled and no new ADB key can be authorised.
That route is a dead end.

The WiFi DUML service on TCP **40007** needs no authentication at all.

## Install

Requirements: a C compiler, CMake, ncurses (for `--visual`).

```bash
git clone https://github.com/mc3dreal/dji-rc-gamepad
cd dji-rc-gamepad
./install.sh
```

`install.sh` builds the driver, installs it under `~/.local`, and offers to
set up the udev rule so the virtual joystick can be created **without root**.

Manual build:

```bash
cmake -B build && cmake --build build
```

## Usage

Power the controller on and connect it to the same WiFi network as your
computer. Then:

```bash
dji-rc-start                    # asks for the IP, remembers it afterwards
dji-rc-start 192.168.1.42       # skip the lookup
```

Or call the driver directly:

```bash
dji-rc-joystick 192.168.1.42
dji-rc-joystick -V 192.168.1.42     # ncurses live view
dji-rc-joystick -t 192.168.1.42     # print values, create no joystick
```

Auto-discovery (`dji-rc-joystick` with no address) scans the local subnets,
but it is slow and misses the controller on larger networks — giving the IP
is faster and more reliable. The controller shows its address under
*Settings → WiFi → (connected network)*.

## Input mapping

| Input | Axis / button |
|---|---|
| Left stick horizontal | `ABS_X` |
| Left stick vertical | `ABS_Y` |
| Right stick horizontal | `ABS_RX` |
| Right stick vertical | `ABS_RY` |
| Left back wheel | `ABS_Z` |
| Right back wheel | `ABS_RZ` |
| Flight mode switch (S/N/C) | `ABS_MISC` (0/1/2) |
| RTH / Flight Pause | button 0 |
| Record | button 1 |
| Shutter, half press | button 2 |
| Shutter, full press | button 3 |
| C1 (left back) | button 4 |
| C2 (right back) | button 5 |

The shutter's half and full press are one physical button: a full press
fires button 2 **and** button 3. Bind only one of them.

## Update rate

**About 9 Hz.** The driver holds a persistent TCP connection and sends DUML
keepalives to keep data flowing; without them it drops to roughly 5 Hz.
Polling harder does not help — sending keepalives twice as often was measured
to give exactly the same 9 Hz, so the limit is in the controller.

That is fine for stabilised flight and for anything that is not a
twitch-reflex task. For FPV acro it feels noticeably steppy. Know this before
you plan around it.

## Using it in games via Steam

The driver reports joystick-style buttons (`BTN_TRIGGER` and friends), so SDL
classifies the device as a *joystick*, not a *gamepad*. Most games want a
gamepad. Steam Input bridges that: enable **generic gamepad configuration
support** under Settings → Controller, then bind the device.

To skip Steam's mapping UI you can hand SDL a ready-made mapping through the
game's launch options:

```
SDL_GAMECONTROLLERCONFIG="060074d9a32c00002310000001000000,DJI RC RM330,crc:d974,platform:Linux,leftx:a0~,lefty:a1,rightx:a3~,righty:a4,leftshoulder:b4,dpright:b5,a:b1,b:b3,guide:b0,lefttrigger:+a2,righttrigger:+a5," %command%
```

Two details in there worth knowing:

- `a0~` and `a3~` — the horizontal stick axes are inverted relative to the
  evdev convention (right reads negative), so they need the tilde.
- `+a2` / `+a5` — the back wheels are bidirectional (`-32767…+32767`) while
  gamepad triggers are one-directional. Using the positive half-axis keeps
  wheel centre at "released"; binding the full axis would leave the trigger
  permanently half-pulled.

If you use this mapping, leave Steam Input on the default gamepad template
rather than adding bindings on top, or you map twice.

## Changes from upstream

- **Fixed the build.** Upstream has a stray line
  (`DJI RM330 remote controller front view.png`) inside `main()`, which makes
  the source fail to compile.
- **Added `dji-rc-start`**, `install.sh`, a udev rule, a desktop entry and an
  optional systemd user unit, so it installs and runs without root.
- **Documented the measured update rate** and the Steam/SDL setup.

## Troubleshooting

**`Virtual joystick` fails / permission denied** — no write access to
`/dev/uinput`. Install the udev rule from `dist/`, or run with `sudo`.

**Nothing found on the network** — the controller drops off the network
entirely when it is switched off or asleep (no ping, no open ports). Wake it
and check it is on the same subnet.

**Axes work, buttons do nothing** — the game likely wants a gamepad, not a
joystick. See the Steam section above.

## License

MIT, inherited from upstream. See [LICENSE](LICENSE).

## Credits

Upstream driver by [stiad](https://github.com/stiad/dji-rc-linux). The
packaging, build fix and documentation in this fork were developed with the
help of Claude.
