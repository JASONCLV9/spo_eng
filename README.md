# E2 Final Exam
**Professor:** Kim, W.

**Students:** Chambe, J., Gonzalez, S., Kamt, L.

## Folder structure
```
│   README.md
│
├───motion-data
│       Forward_SegmentOrientation.csv
│       Forward_SegmentPosition.csv
│       Smash_SegmentOrientation.csv
│       Smash_SegmentPosition.csv
│       Top_spin_SegmentOrientation.csv
│       Top_spin_SegmentPosition.csv
│
├───required-assets
│       elbow_flex_l.csv
│       elbow_flex_l.sto
│       info_theory.m
│       left_hand_height.csv
│       left_hand_height.sto
│       opensim_data.m
│       Rajagopal2015.osim
│       Top_spin-001_Xsens_jointangle_q.mot
│
└───script
        topspin_affordance_scene.py
        visualize_motion_data.py
```
Scripts read the CSV files from *motion-data/* automatically.

## Software version
- 8.3.1 Vizard Free 64-bit

## Dependencies
- Vizard 8.3.1 only 
- Vizard built-in resources: *ground.osgb*, *sky_day.osgb*, *vcc_male.cfg*.
- OpenSim and MATLAB are not needed to run the scene.

## Controls

**topspin_affordance_scene.py**

| Key | Action |
|---|---|
| SPACE | Play / pause |
| R | Restart |
| S | Speed: real time 1.0x / slow motion 0.25x (shown in red) |
| 1 - 4 | Select condition (ball position) |
| V | Show / hide criterion result |
| Left / Right | Previous / next frame (while paused) |
| G | Jump to next validation time (2.500, 3.083, 3.600 s) |
| P | Print current values in the console |
| C | Change camera view |
| T | Show / hide racket trajectory |
| A | Show / hide avatar |
| K | Show / hide skeleton |
| N / M | Rotate table -5 / +5 degrees |
| U / I | Move table closer / farther 5 cm |
| H | Show / hide help |

**visualize_motion_data.py**

| Key | Action |
|---|---|
| 1 or T | Topspin |
| 2 or F | Forward |
| 3 or S | Smash |

## Units
- Length: meters. 
- Angles: degrees. 
- Time: seconds. 

## Frames
- Xsens axes: X forward, Y left, Z up. 
- Vizard axes: X right, Y up, Z forward.

## Conditions
- Sampling rate: 60 Hz.
- Ball offsets (conditions): 0.00, 0.08, 0.15, 0.30 m. 
- Contact threshold: 0.095 m (paddle radius 0.075 m + ball radius 0.020 m).

## Launch instructions
1. Install Vizard 8.3.1 and unzip the package *src* keeping the folder structure above.
2. Open Vizard and go to *File > Open*.
3. Open *script/topspin_affordance_scene.py* and click Run (F5).
4. Press H in the scene to see the help. The console prints the startup summary and the criterion results.
5. (Optional) Open and run *script/visualize_motion_data.py* to view the recorded motions.

The scripts must be run from Vizard.