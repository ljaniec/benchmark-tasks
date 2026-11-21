# Pull a graphics card out of its PCIe x16 slot while holding the spring retention latch open

Instruction given to the policy: `Put the green graphics card into the grey tray, pressing the orange latch tab to free it.`

Files in this folder: `task.json`, `task.py` (mechkit slot, latch and pad articulation, scoring, latched violations, open-loop demo), `offline_check.py`, `demokit.py` (matrix TCP grips), `mechkit.py`, `arm_watch.py` (per-step finger/TCP read), `cad/` (BUILD.md, layout DXF, BOM PDF, two judging gauges, generator), `LICENSE`, `README.md`.

## Layout in one paragraph

Table frame in metres: x forward from the arm-base line, y left, h up from the table top. The card and slot are turned 90 deg about vertical: the card plane is x = 0.180, the long axis runs along +y (tail end at y = -0.030, bracket end toward +y) and the shroud face points to -x, toward the arms and the top camera. The card origin (tail end, bottom of the gold fingers) is (0.180, -0.030, h 0.0146). The x16 gold fingers run 52.5 to 141.5 mm from the tail end (about 14.5 mm from the bracket, as on a real card), so they sit in the slot at y 0.0225 to 0.1115 and the card overhangs the slot end by about 52 mm. The retention notch jaw is at the tail-side end of the fingers (52.5 to 57.5 mm from the tail). The orange latch tab sits under the overhang, offset to the PCB back face (+x side, 3 to 22 mm off the PCB plane, x 0.183 to 0.202, y -0.0095 to 0.0005, top at h 0.0261): it is reached by pressing down beside the card's back face, between the card and the CPU heatsink. The system board (x 0.116 to 0.360, y -0.120 to 0.124), the system's CPU heatsink (modelled as a 95 x 95 x 54 mm block, x 0.220 to 0.315, y 0.000 to 0.095), two DDR5 DIMMs (x 0.220 to 0.353, y -0.1085 to -0.0905) and a flat plate standing in for the chassis wall under the board (x 0.115 to 0.365, y -0.150 to 0.145) are laid out in the same rotated frame; the chassis rear wall is not modelled and the bracket is free (screw removed, PCIe door open). The grey antistatic tray (x 0.150 to 0.450, y 0.145 to 0.325) sits on the mat (x 0.060 to 0.500, y -0.300 to 0.340) to the left (+y) of the bench. The full list with every object's world coordinates is in the `setup` field of `task.json`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (in the standard YAM / Inspect Robots environment):

```bash
python tasks/pcie-card-latch-pull/task.py
```

It exits 0 when every check passes (about 10 s). Every gripper placed at the tab is a closed gripper in the demo's press orientation, its lowest finger-pad point put on (or a set height above) the tab-top surface. What it checks:

- Score ladder: seated card 0; tail 6 mm up with a finger pad on the pressed tab 0.25; tail 4 mm up 0; fingers out 0.5; 20 consecutive end-state steps turn 0.5 into 1.0 and success.
- The 1.0 state is refused for one closed gripper, a gripper within 40 mm of the card, a card tilted 15 deg, moving, standing on its edge or lying over the tray wall.
- latch_forced fires for a slow tug (0.5 mm steps to 10 mm), a fast tug (4 mm in one step), a rocked card whose own tail pushes the latch open, a bracket-end-first pitch to 35 deg, a straight 12 mm pull, and a reseat followed by a raise with no hand on the tab.
- latch_forced does not fire for a rise or pitch with a finger pad on the pressed tab, for the latch springing back after the press has freed the tail (score stays 0.25), or for a re-raise with a pad on the tab after a reseat. A legal full sequence scores 1.0.
- Tab contact during a 6 mm tail rise, latch at -0.15 rad (-8.6 deg): a pad on the tab top at its centre, at 3, 5, 7 and 9 mm either way along the 19 mm side, at 3 and 4 mm either way across the 10 mm side, at all four corners and at the four edge midpoints is clean.
- Hovering: a gripper 3, 4, 10 or 20 mm above the tab top gives latch_forced; with the latch at -5.5 deg or -8.6 deg, hovering 3.1 or 4 mm gives latch_forced and contact (0 mm) counts as pressed; contact with the latch only 3.5 deg open gives latch_forced.
- Other violations: card_dropped after a 50 mm fall following release (not for a release from a tilted edge onto the tray floor), card_hit_cooler, card_hit_board, card_off_bench; every violation persists in an otherwise perfect state and reset clears it.
- Cheats: card upright, card held flat over the tray, hands clear with the card still seated; a gripper whose lowest probe point is 45 mm above the tray floor centre is not clear, at 60 mm it is.
- success() and score() share one evaluation per step.
- The open-loop demo takes no arguments, is deterministic, and both arms' plans replay at 20 Hz with every joint inside the URDF limits and an IK error under 0.5 mm and 0.5 deg.
- No two rigid boxes of the card, slot, chassis plate, board, heatsink and DIMMs overlap by more than 0.5 mm (0.2 mm for the gold fingers in the slot).

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/pcie-card-latch-pull/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the green graphics card into the grey tray, pressing the orange latch tab to free it." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir logs/pcie-dream
```

The CLI prints `task_score: <value>` and the log keeps it at `samples[0].epochs[0].task_score`.

## Scoring and how a human judges it

Scoring is automatic (SCORE_OVER=end, SUCCESS_HOLD_S=0.05) and is exactly `_evaluate` and `success` in `task.py`. `success()` and `score()` share one cached evaluation per step, so they never lag each other. The score is the value at the final step.

Ladder (the highest rung whose state holds at that step; no violation):

- 0.25: the card's tail-end bottom corner is at least 5 mm above its seated height. Tape or video: the corner rises above the 5 mm gauge step (15.3 mm step on the system board top) in the side view.
- 0.5: the gold finger bottoms are at least 10 mm above the slot top (h >= 0.0336), or the fingers are wholly outside the slot footprint (x 0.176 to 0.184, y 0.0105 to 0.1165, +-2 mm) seen from above. Video: finger bottoms above the 21.0 mm gauge step, or the fingers clear of the slot outline in the overhead view.
- 1.0: for 20 consecutive steps (1.0 s) the card is entirely inside the tray interior (x 0.152 to 0.448, y 0.147 to 0.323, h up to 0.060), its PCB is within 10 deg of flat, it is at rest (< 10 mm/s, < 0.5 rad/s), both grippers are open (pad gap at least 30 mm) and every gripper probe point (TCP, tips, pad surfaces) is at least 50 mm from the card, the slot housing with latch tab (x 0.176 to 0.202, y -0.0095 to 0.1165) and the tray walls and floor (the slot, tab and tray are sampled as real surfaces at 6 mm spacing, so the 50 mm clearance is real). Video: card inside the tray outline (overhead), flatter than the 10 deg wedge, still, both hands open and clear for one second.

Latched violations (score 0.0 for the rest of the episode):

- latch_forced: the card is "seated" while its tail corner and its notch jaw (bottom of the jaw under the hook, 52.5 to 57.5 mm from the tail) are both under 2 mm above their seated height and the gold fingers are in the slot; while seated a flag is reset to "tab pressed now". After that the flag becomes true on any step where the tab is pressed: the latch is at least 5 deg open (of 12 deg travel) AND a finger pad touches the orange tab top, that is, some point of either finger pad (both pads sampled as solid boxes on a 2 mm grid) is within 2 mm of the tab-top surface, anywhere on the 10 x 19 mm patch, taken at the current latch angle. A gripper hovering more than 2 mm above the tab does not count, even while the card holds the latch partly open. Pressed means contact plus latch angle, not a measured force. latch_forced fires when the tail corner or the notch jaw is 3 mm or more above its seated height, or the gold fingers are out of the slot (10 mm above the slot top or outside its footprint), while the flag is false. Video: the card's tail end or connector comes up while no finger is on the orange tab with the tab pushed down.
- card_dropped: while no gripper holds the card (a closed gripper with a pad within 8 mm of the card), its lowest point falls more than 30 mm below where it was when it was released. Video with tape on the background: the card falls 30 mm or more after leaving the hand.
- card_off_bench: any card point beyond the mat (x 0.060 to 0.500, y -0.300 to 0.340) or 20 mm below the table top. Video: the card leaves the mat or goes over the table edge.
- card_hit_cooler, card_hit_dimm0, card_hit_dimm1: a card surface within 1 mm of the CPU heatsink (the block named cooler in the code), or of a DIMM with its socket. Video: visible contact.
- card_hit_board: the gold fingers touch the board top (h <= 0.0136) outside the slot footprint +-3 mm. Video: the fingers on the board away from the slot.

The 1.0 state also means the card came out undamaged: latch_forced rules out pulling the card past the latch hook, and card_dropped, card_hit_cooler, card_hit_dimm0, card_hit_dimm1 and card_hit_board rule out the other ways of damaging it.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/pcie-card-latch-pull/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put the green graphics card into the grey tray, pressing the orange latch tab to free it." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir logs/pcie-demo
```

The demo is `demo()` in `task.py`, open loop and deterministic (no observations, no scorer branching). The right arm starts above the orange tab with its closed gripper tilted so the hand leans out toward the tail side, clear of the heatsink, descends to touch the tab top, presses it down past its open stop (the arm lags its command, so the press force builds), holds the latch open for several seconds while the card is peeled, lifts off, then parks open and waits. In the submitted demo run its finger pad stayed 0.0 to 0.15 mm from the tab-top surface throughout the press, inside the 2 mm contact rule.

The left arm opens over the green band (95 to 135 mm from the tail, on the card top edge) above grip height, descends, settles, closes 9 mm narrower than the 15.8 mm card-plus-shroud thickness so the pads stop on the card and squeeze, then peels: it commands a large rise with a few degrees of pitch of the gripper about the card's long-axis normal (the arm lags, so the tail peels first). The arm's Cartesian stiffness is low (about 350 N/m), so it lags its command by force over stiffness; the large commanded rise builds the peel force. The card then lifts well clear of the heatsink, carries over the tray while turning 90 deg about vertical so the card's long axis lies along x, tilts about the x axis, sets its edge down in the tray, settles, releases, slides the pads out, backs off and parks open. Exact waypoints and timings are in `demo()`; the plan fits well inside the 1,600-step limit.

## Run limit, runtime and seeds

- Run limit: 1,600 control steps at 20 Hz (80 s of simulated time), `--max-steps 1600`.
- Runtime per trial: scripted demo 575 control steps (28.8 s simulated, ends on success), about 7 min wall time on an L40S shared with other simulation jobs; Dream-1 trial 1,600 steps (80 s simulated), about 16 min wall time.
- Seeds: `--seed 0` with `-E object_jitter=0` for the open-loop demo.

## Fidelity and limits

- Real counterpart: a Dell Precision 3660 Tower with its own system board, its own CPU heatsink and two DDR5 DIMMs, and an NVIDIA T400 4GB low-profile card (listed as a discrete graphics option in the Precision 3660 Tower Setup and Specifications) in the primary x16 slot; side cover and PCIe door open, tower laid on its side on the bench. Its Service Manual, section "Removing the graphics card", says to push and hold the securing tab on the graphics-card slot and lift the graphics card from the slot, and the latch springs back to locked when released. The pre-trial check in `cad/BUILD.md` (press the tab, release: it must return to locked and hold the seated card) is mandatory.
- The board layout (heatsink and DIMM positions relative to the slot) is approximate, not measured from a 3660 board. The heatsink is a 95 x 95 x 54 mm block, the DIMMs and board are boxes, and the chassis is a flat plate under the board; the chassis walls (which rise above the card in the real tower), the rear wall with its expansion-slot openings, the power supply and the drive bays are not modelled.
- Rigid-body PhysX, 20 Hz control, 3 cameras at 320 x 176 (top and two wrist). The card is the only free body; the slot housing, board, heatsink, DIMMs, chassis plate, mat and tray are fixed to the world. Real parts are modelled as boxes.
- All force numbers are model estimates, not measurements. Amphenol product specification GS-12-1406 (PCI Express connector, section 7.1) and the Gen 3 card-edge datasheet (citing GS-12-233) give 0.15 N minimum unmating force per contact pair, tested with a 1.44 mm steel gauge; a x16 slot has 82 pairs, so the floor is 12.3 N. The model's four sprung pads (10,250 N/m, 1 mm preload, pad friction 0.2 against the card's 1.0, which PhysX average friction combine makes an effective pair friction of 0.6) give about 24.6 N for a straight pull and about 12.3 N for the tail half. No board was pulled to measure it; `cad/BUILD.md` has the real readings taken with a push-pull digital force gauge (0 to 50 N, 0.01 N resolution).
- Tab force is a model estimate, not measured: the latch spring 0.53 N m/rad gives roughly 2 N at the start of the press, 2.4 to 3.4 N at 3 deg and 4.9 to 7.0 N at the 12 deg open stop. The hook flex compliance (30 N/mm, read by no rule) is also an estimate.
- The slot joint drive targets (pad preload, latch preload) are re-applied every step; this per-step drive-target write is the only external write to the slot. There are no teleports, no glue and no injected forces.
- The hook lip geometry is simplified to a box 1 mm over the notch jaw edge (0.5 mm card end play); real latches have a ramped lip and a detent.
- The arm's low Cartesian stiffness (about 350 N/m) is why the demo commands a large rise: the lag between command and arm position builds the peel force.
- The policy sees only the 3 cameras, no force or latch state.

## Submitted videos and scores

Task file SHA-256, first 12 hex: 9eac17408b88 (`sha256sum task.py | cut -c1-12`). inspect-robots 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta`.

- Videos: composites of the three cameras (top camera above, left and right wrist cameras below).
- Dream-1 run: score 0.0, ran the full 1,600 steps (seed 0, the one Dream-1 trial at this task file; Dream-1 runs are not bitwise reproducible). Both arms rise about 13 mm from the ready pose and then descend; the right gripper ends resting on the mat beside the bench, the left gripper ends about 190 mm above the near-left corner of the tray; each gripper closes once, about 23 to 26 s into the run, and stays closed; neither arm reaches down to the card, the latch tab or the tray, and no part moves. Closest TCP approach, from the measured joint states: left gripper 209 mm from the green band on the card (step 1256), right gripper 217 mm from the orange latch tab (step 1591).
- Demo: full score 1.0, ended by `success`.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

## Reset between trials

Reseat the card until the latch closes with a click (tail end at x = 0.1800 m, y = -0.0300 m, card plane at x = 0.180, card overhanging the slot end by about 52 mm); check the green tape band (40 mm, 95 to 135 mm from the tail end, over the top edge and down both faces) and the orange patch on the tab (beside the card's back face); check the latch spring returns the tab hands-free; empty the tray and square the tower and tray to the template.

## Robot asset

The external **YAM robot asset is MIT licensed** (I2RT YAM via the standard Inspect / YAM embodiment). It is not redistributed here.
