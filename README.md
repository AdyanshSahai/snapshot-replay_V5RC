# Snapshot-Replay
This is the coding repository which we are using to go to VEX V5 World's!!! Team 20785R, Season 2025-2026, Pushback.

Snapshot-Replay
VEX V5 driver-control recorder for PROS + LemLib — snapshot poses and actions during a match, replay them autonomously with one button press.

What It Does
During driver control, pressing a d-pad button writes the robot's current pose or mechanism state as a single token to a file on the SD card. When autonomous runs, the robot reads that file line-by-line and executes every saved command — driving to recorded coordinates, turning to recorded headings, and firing recorded mechanism states — with no manual path-planning required.

Features
Zero path-planning — drive the route yourself, let the robot repeat it
Persistent recording — file is flushed to disk after every press; safe even if the match ends early
Auto-archive — previous session is copied to replay-archive.txt before a new recording starts
Full mechanism support — records intake state, scraper toggles, and wing toggles alongside motion
Single-open file design — avoids VexOS's ~8 total fopen per power-cycle limit
Requirements
Dependency	Version
PROS	4.2.2
LemLib	0.5.6
VEX V5 Brain + Controller	—
microSD card (FAT32)	any size
The SD card must be inserted before powering on. If missing, USD OPEN FAILED appears on brain screen line 6, and autonomous exits silently.

Hardware Configuration
Edit the port constants at the top of src/main.cpp to match your wiring.

Component	Port / ADI
Left drive	10, 9, 8 (blue cartridge)
Right drive	-7, -6, -5 (blue, reversed)
Bottom roller	-3
Top roller	-2
Scraper	ADI B
Wing	ADI A
IMU	19
Optical	20
Chassis parameters

Parameter	Value
Track width	10.8 in
Wheel	3.25 in omni (NEW_325)
RPM	450
Installation

# 1. Clone the repo
git clone https://github.com/your-org/Snapshot-Replay.git

# 2. Build with PROS
pros build

# 3. Upload to brain slot 2
pros upload --slot 2
Insert a FAT32-formatted microSD card into the Brain before powering on.

Usage
Recording
Start driver control.
Drive the robot to each position or perform each action.
Press the matching button to snapshot it (see Button Reference).
Repeat for every step of the routine.
Press X when done — this closes the file and ends the loop.
Replaying
Start autonomous (slot 2). The robot will:

Open /usd/replay.txt
Execute each saved command in order using LemLib motion
Display Replaying... on the Brain screen
Display Replay Done when finished
If no file is found, autonomous exits without error.

Button Reference
D-pad — Pose Recording
Button	Saves	Replays as
↑ UP	Current pose	moveToPose() forward
↓ DOWN	Current pose	moveToPose() backward
← LEFT	Current heading	turnToHeading()
→ RIGHT	Current intake state	Intake motors to recorded state
Face Buttons — Mechanisms
Button	Live action	Also saves
B	Toggle scraper	SCRAPER_TOGGLE
Y	Toggle wing	WING_TOGGLE
X	Close file + stop	(no token — exits driver loop)
Shoulder Buttons — Live Intake Control
Button	Bottom roller	Top roller
L2	Dump	Hold
L1	Intake	Intake
R2	Intake fast	Intake fast
R1	Outtake	Outtake
(none)	Stop	Stop
Intake is not recorded continuously. Press → RIGHT to snapshot the current intake state at a specific moment.

Token File Format
/usd/replay.txt is a plain UTF-8 text file — one token per line, readable in any editor. Blank lines are ignored.


MOVE_POSE_FWD <x> <y> <theta>
MOVE_POSE_BWD <x> <y> <theta>
TURN_TO <theta>
INTAKE_STATE <l1> <l2> <r1> <r2>
SCRAPER_TOGGLE
WING_TOGGLE
x, y — position in inches, LemLib odometry frame
theta — heading in degrees
l1 l2 r1 r2 — 0 or 1 for each shoulder button
Example:


MOVE_POSE_FWD 0.00 24.00 90.00
INTAKE_STATE 1 0 0 0
MOVE_POSE_FWD 12.00 24.00 90.00
SCRAPER_TOGGLE
TURN_TO 180.00
MOVE_POSE_BWD 12.00 0.00 180.00
WING_TOGGLE
You can hand-edit this file to tune waypoints without touching code.

Brain Screen
LCD Line	Content
0	X position (in)
1	Y position (in)
2	Heading (°)
4	Brain battery %
5	Controller battery %
6	Last record event (Saved FWD, USD OPEN FAILED, etc.)
SD Card Notes
VexOS limits total fopen calls to approximately 8 per power cycle (not just simultaneous). This system uses only 3 opens per session:

replay.txt → read (archive copy)
replay-archive.txt → write (archive copy)
replay.txt → write (kept open all match, fflush after each token)
If you ever see USD OPEN FAILED, the card is missing, not FAT32, or was not seated before power-on.

The archive (replay-archive.txt) is overwritten at the start of every driver-control session. Copy it off the card with a card reader before running driver control if you want to keep it permanently.

License
Distributed under the MIT License. See LICENSE for details.
