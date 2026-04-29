# Snapshot-Replay

> VEX V5 driver-control recorder for PROS + LemLib — snapshot poses and actions
> to SD card during a match, replay them autonomously with full odometry accuracy.

![Platform](https://img.shields.io/badge/platform-VEX%20V5-blue)
![PROS](https://img.shields.io/badge/PROS-4.2.2-blueviolet)
![LemLib](https://img.shields.io/badge/LemLib-0.5.6-orange)
![License](https://img.shields.io/badge/license-MIT-green)

---

## Table of Contents

- [How It Works](#how-it-works)
- [Requirements](#requirements)
- [Hardware Setup](#hardware-setup)
- [Installation](#installation)
- [Recording a Routine](#recording-a-routine)
- [Replaying a Routine](#replaying-a-routine)
- [Button Reference](#button-reference)
- [Token File Format](#token-file-format)
- [Brain Screen](#brain-screen)
- [SD Card Notes](#sd-card-notes)
- [License](#license)

---

## How It Works

During driver control, each d-pad or face button press appends one plain-text
**token** to `/usd/replay.txt` on the SD card:

```
↑ UP pressed  →  MOVE_POSE_FWD 12.00 24.00 90.00
↓ DOWN pressed →  MOVE_POSE_BWD 0.00  0.00  180.00
← LEFT pressed →  TURN_TO 45.00
→ RIGHT pressed → INTAKE_STATE 1 0 0 0
B pressed      →  SCRAPER_TOGGLE
Y pressed      →  WING_TOGGLE
```

When autonomous starts, the robot opens that file, reads it line by line, and
calls the matching LemLib function for each token. No path-planning software
needed — you drive the route, the robot reproduces it.

The file is opened **once** at the start of driver control and kept open for
the entire match. Each write is followed by `fflush()` so data is committed to
disk immediately. This avoids exhausting VexOS's limit of roughly 8 total
`fopen` calls per power cycle — the bug that causes `USD WRITE FAILED` after
a handful of button presses.

---

## Requirements

| Dependency | Version |
| --- | --- |
| [PROS](https://pros.cs.purdue.edu/) | 4.2.2 |
| [LemLib](https://github.com/LemLib/LemLib) | 0.5.6 |
| VEX V5 Brain + Controller | any |
| microSD card (FAT32) | any size |

> The SD card must be inserted before the Brain powers on. If it is missing,
> `USD OPEN FAILED` appears on brain screen line 6 and autonomous exits silently.

---

## Hardware Setup

Update the port constants near the top of
[`src/main.cpp`](src/main.cpp) to match your robot before uploading.

**Motor ports**

| Motor group | Ports | Cartridge |
| --- | --- | --- |
| Left drive | `10, 9, 8` | Blue (600 RPM) |
| Right drive | `-7, -6, -5` | Blue, reversed |
| Bottom roller | `-3` | — |
| Top roller | `-2` | — |

**Other devices**

| Device | Port |
| --- | --- |
| Scraper (pneumatic) | ADI `B` |
| Wing (pneumatic) | ADI `A` |
| IMU | `19` |
| Optical sensor | `20` |

**Chassis tuning**

| Parameter | Value |
| --- | --- |
| Track width | 10.8 in |
| Wheel diameter | 3.25 in (NEW\_325 omni) |
| Drive RPM | 450 |
| Left tracking offset | −5.4 in |
| Right tracking offset | +5.4 in |

---

## Installation

```bash
# Clone the repository
git clone https://github.com/your-org/Snapshot-Replay.git
cd Snapshot-Replay

# Build
pros build

# Upload to Brain slot 2
pros upload --slot 2
```

Insert a FAT32-formatted microSD card into the Brain before powering on.

---

## Recording a Routine

1. Start **driver control**.
2. Drive the robot to the first desired position.
3. Press the matching button to snapshot the current state
   (see [Button Reference](#button-reference)).
4. Continue driving and snapshotting for every step in the routine.
5. Press **X** when finished — this flushes the file, closes it, and exits
   the driver-control loop cleanly.

> You do not need to press X before the match ends. Every write is flushed
> to disk immediately with `fflush()`, so no data is lost if time runs out.

---

## Replaying a Routine

Start **autonomous** (slot 2). The robot will:

1. Open `/usd/replay.txt`
2. Execute each token in sequence using LemLib motion primitives
3. Show `Replaying...` on brain screen line 0 while running
4. Show `Replay Done` when the last token finishes

If the file does not exist or the SD card is absent, autonomous exits without
error and without moving.

At the start of the **next** driver-control session, the previous
`replay.txt` is automatically copied to `replay-archive.txt` and a fresh
`replay.txt` is opened.

---

## Button Reference

### D-pad — Pose & State Recording

| Button | Token written | What it does in autonomous |
| --- | --- | --- |
| **↑ UP** | `MOVE_POSE_FWD x y θ` | Drive forward to the recorded pose |
| **↓ DOWN** | `MOVE_POSE_BWD x y θ` | Drive backward to the recorded pose |
| **← LEFT** | `TURN_TO θ` | Turn in place to the recorded heading |
| **→ RIGHT** | `INTAKE_STATE l1 l2 r1 r2` | Set rollers to the recorded button state |

### Face Buttons — Mechanisms

| Button | Live action | Token written |
| --- | --- | --- |
| **B** | Toggle scraper piston | `SCRAPER_TOGGLE` |
| **Y** | Toggle wing piston | `WING_TOGGLE` |
| **X** | Close file + stop recording | *(none — exits driver loop)* |

### Shoulder Buttons — Live Intake (not individually recorded)

| Button | Bottom roller | Top roller |
| --- | --- | --- |
| **L2** | Dump | Hold (brake) |
| **L1** | Intake | Intake |
| **R2** | Intake fast | Intake fast |
| **R1** | Outtake | Outtake |
| *(none)* | Stop | Stop |

Intake state is captured as a snapshot when **→ RIGHT** is pressed, not
recorded continuously.

---

## Token File Format

`/usd/replay.txt` is a plain UTF-8 text file with one token per line.
It can be opened and edited in any text editor.

```
MOVE_POSE_FWD <x> <y> <theta>
MOVE_POSE_BWD <x> <y> <theta>
TURN_TO <theta>
INTAKE_STATE <l1> <l2> <r1> <r2>
SCRAPER_TOGGLE
WING_TOGGLE
```

- `x`, `y` — position in **inches**, LemLib odometry frame
- `theta` — heading in **degrees** (0 = forward, 90 = left, etc.)
- `l1 l2 r1 r2` — `1` or `0` for each shoulder button at snapshot time
- Blank lines are ignored by the parser

**Example file:**

```
MOVE_POSE_FWD 0.00 24.00 90.00
INTAKE_STATE 1 0 0 0
MOVE_POSE_FWD 12.00 24.00 90.00
SCRAPER_TOGGLE
TURN_TO 180.00
MOVE_POSE_BWD 12.00 0.00 180.00
WING_TOGGLE
```

You can hand-edit this file to fine-tune coordinates without changing any code.

---

## Brain Screen

| Line | Content |
| --- | --- |
| 0 | X position (in) — or status message during autonomous |
| 1 | Y position (in) |
| 2 | Heading (°) |
| 4 | Brain battery % |
| 5 | Controller battery % |
| 6 | Last record event (`Saved FWD`, `USD OPEN FAILED`, `Recording Closed`, …) |

---

## SD Card Notes

VexOS caps total `fopen` calls at roughly **8 per power cycle** (not just
simultaneously open files). This system consumes exactly 3 per session:

1. `replay.txt` → read (to copy into the archive)
2. `replay-archive.txt` → write (archive destination)
3. `replay.txt` → write (kept open all match)

Staying well within the limit means recording never silently fails mid-match.

**Format the card as FAT32** before first use. exFAT and NTFS are not
supported by VexOS.

The archive file (`replay-archive.txt`) is **overwritten** every time driver
control starts. If you want to preserve a recording permanently, copy it off
the card with a card reader before the next driver-control run.

---

## License

Distributed under the MIT License. See [`LICENSE`](LICENSE) for details.

License
Distributed under the MIT License. See LICENSE for details.
