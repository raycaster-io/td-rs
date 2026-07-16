# Laser Device CHOP — packaging and use in TouchDesigner

The compiled plugin binary works two ways with no code changes, because the
td-rs C++ glue exports the standard `FillCHOPPluginInfo` /
`CreateCHOPInstance` / `DestroyCHOPInstance` entry points used by both of
TouchDesigner's loaders.

## 1. Native custom operator (Plugins folder)

Build and install:

```bash
just build laser-device
just install laser-device
```

This drops `laser_device.plugin` (macOS) / `laser-device.dll` (Windows) into
the global Plugins folder, and "Laser Device" appears in the OP Create
dialog under Custom. A project-relative `Plugins/` folder next to the `.toe`
works too. (The names differ per platform: the xtask replaces hyphens with
underscores when it assembles the macOS bundle but keeps the package name
for the Windows DLL.)

## 2. Reusable TOX (no install step for end users)

A Base COMP wrapper containing a stock **CPlusPlus CHOP** whose *Plugin
Path* resolves the platform binary next to the `.toe` file:

```
my_project/
├── my_project.toe
├── laser_device.tox        # the wrapper COMP
├── laser_device.plugin     # macOS arm64 binary
└── laser-device.dll        # Windows x64 binary
```

To (re)generate `laser_device.tox`, open a project whose folder contains the
binary for your platform and run in the textport:

```python
exec(open('/path/to/td-rs/plugins/chop/laser-device/td/build_tox.py').read())
```

The script wires In → CPlusPlus CHOP → Out, points *Plugin Path* at
`project.folder` with an `app.osName` switch, and promotes the plugin's
parameters (Active, Backend, Device, Refresh Devices, Pps, Intensity, Scale,
Default Color, Address, Sender Name) to a custom "Laser" page on the COMP
with two-way bindings. The Refresh Devices pulse is forwarded through a
Parameter Execute DAT (value bindings don't deliver pulse events), and the
Device menu uses a COMP-relative menu source so the .tox survives renames.
Binaries are platform-specific, so ship both files alongside the .tox when
distributing.

## Using the node

- **Input**: one CHOP, one sample per laser point. Channels are matched by
  name, case-insensitively: `x`/`tx`, `y`/`ty`, plus optional `r`, `g`, `b`
  and `i`/`intensity`. With no recognized x/y names, channel order is used
  (x, y, r, g, b, i); naming only one of x/y is reported as an error rather
  than guessing. Coordinates are in [-1, 1]; colors in [0, 1]. This is the
  same layout TouchDesigner's native laser tools produce.
- **Backend = Hardware**: press *Refresh Devices*, pick a DAC from the
  *Device* menu (Ether Dream, LaserCube WiFi, IDN are discovered on the
  network), set *Points Per Second* for your scanner, then enable *Active*.
- **Backend = Ponk Network**: paths are encoded with MadMapper's PONK
  protocol and sent as UDP datagrams to *Address*
  (default `239.255.10.24:5583`, the PONK multicast convention — a unicast
  `host:port` works too). Point MadMapper or any PONK receiver at the same
  group.
- **Output channels**: `connected`, `active`, `points` (points in the last
  sent frame), `frames` (frames sent since connect).
- Warnings (no device selected, missing x/y channels, send failures) appear
  on the node; the info popup shows what the node is connected to.

## Safety notes

- *Active* defaults to **off**; the node never emits on creation.
- x/y are hard-clamped to [-1, 1]; non-finite coordinates (NaN/infinity from
  upstream math) are emitted as blanked beam-off points. A blank frame is
  sent on input loss, and hardware sessions are disarmed (output forced off)
  on deactivation and teardown.
- The DAC keeps drawing the **last received frame** if TouchDesigner stops
  cooking while Active is on (frame sessions auto-loop). Pause the timeline
  with the beam blocked, or toggle Active off first.
- Galvo velocity/density limiting ("scanner safety") is not implemented
  here — that responsibility stays with the DAC and your show setup. Test
  with the beam blocked and follow normal laser-safety practice.

## Testing without a laser

On macOS/Windows builds the `audio-dacs` feature of the `laser-dac` crate is
enabled, which exposes an oscilloscope/audio output device in the Device
menu — an XY oscilloscope (or scope software on an audio interface) shows
the drawn path without any laser hardware. For PONK, MadMapper's free demo
receives frames.
