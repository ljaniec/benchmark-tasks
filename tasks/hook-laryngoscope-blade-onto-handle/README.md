# Hook a laryngoscope blade onto its handle and swing it out until the light comes on

Instruction given to the policy: `Pick up the blue blade from the yellow card and hook it onto the orange laryngoscope handle until the light comes on, then let go.`

Files in this folder: `task.json`, `task.py` (scene, scoring, open-loop demo, offline self-test), `cad/` (`gen_cad.py`, `BUILD.md`, cradle STL files), `README.md`.

## What is on the table

A stainless instrument tray (light grey, 260 x 380 x 20 mm, 1.5 mm walls) centred at (0.250, -0.085) m. On its floor a printed closed cradle (5 mm wide, 9 mm high): a 108 mm rail along the handle's far side, a 114 mm blade-side rail (handle frame x -139 to -25 mm, clear of the blade's swing), a rear end stop whose centre is 6 mm behind the barrel end, and a 12 mm high front post (3 x 9 mm) just ahead of the head cheeks. Together they stop the handle moving in +x and turning, so no human hand is needed in a real trial; the handle can still be lifted out. An orange hook-on laryngoscope handle (orange paint, painted for visibility, real handles are chrome or black; 18.3 mm barrel, 149 mm overall, 0.18 kg) lies in the cradle along x, head toward +x. Its head has a 4 mm steel hinge pin standing upright between two cheeks, a lamp switch plunger (1 mm proud, 4000 N/m spring, on at 0.4 mm) in the stop face, and a spring plunger whose 1.5 mm ball presses down on the blade heel. Pin axis at (0.300, -0.100) m. A Macintosh 3 class stainless blade (142 mm overall, 75 g, painted blue overall, 16 mm wide heel, hook slot 4.6 mm) lies on its side on a yellow printed carrier card (190 x 50 mm, a flat visual marking on the tray floor that nothing collides with) beside the handle, blade origin (slot bottom) at (0.300, -0.175) m, pointing toward -x. The blade origin (slot bottom) is the reference point; the heel back end is 12 mm further along the blade, and the slot mouth has only a 1.5 mm per side chamfer over 3 mm.

The slot: ISO 7376 hook-on fittings use a hinge pin about 4 mm in diameter with a matching slot in the blade; the 4.6 mm slot and 4.0 mm pin are class-typical dimensions. Nothing guides the blade onto the pin beyond that small chamfer.

The detent: the ball is pressed down on the heel top by a spring plunger (an M4-M5 steel spring plunger, end force 10-40 N, e.g. Misumi/Kipp class). It is placed on the handle head here; on real ISO 7376 fittings the spring balls sit in the blade and the detents on the handle (equivalent kinematics). The heel's -t prong has an engage chamfer along its outer edge, a flat and a 1 mm step down (the lock). During the swing the ball climbs the chamfer (engage resistance), rides the flat and snaps over the step at about 85 deg; the step then holds the blade at 90 deg. The force is chosen so the swing needs ENGAGE_N at the blade tip, inside the 1.1-4.4 N (1/4 to 1 lb) of the Medline laryngoscope instructions for use; the ball rides CREST_MM up the chamfer and the lock holds with HOLD_N at the tip. The lamp switch spring pushes the blade back off 90 deg, and tray friction (0.25 static, 0.20 dynamic, stainless, a slick contact where the lower friction applies; handle 0.15/0.12) cannot hold it, so the lock must.

The cradle holds the handle, so one arm is enough: this is not a bimanual task. The second arm may stay out of the way. Hook, swing, detent and lamp switch are contact and springs on rigid bodies, with no scripted forces; the lamp's light is a visual glow patch only, teleported onto the blade's light outlet while the switch is pressed, and the yellow card is a flat marking (both visual only, no collision). The lamp switch and the detent ball are sprung sliders whose collisions with the handle body are filtered. The blade is simplified as straight boxes, the real Macintosh 3 spatula is curved. Dimensions are class-typical, not measured from a sample. Nothing else is scripted: no teleport of the blade or handle, no glue, no injected force, no rate pinning. The demo's 2 mm lift and 92 deg overdrive are chosen only so that the open-loop plan seats the blade through the arm's compliance.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (set `PYTHONPATH` to the folder that contains `yam_arena`):

```bash
PYTHONPATH=<folder containing yam_arena> python tasks/hook-laryngoscope-blade-onto-handle/task.py
```

It asserts `task.json` carries the same instruction as `task.py`, runs the scoring cases on hand-built states (start layout, slot mouth short of the pin, pin in the mouth chamfer, hooked, hooked but still held, 59 and 61 deg, 97 deg, light off, still moving, closed gripper far away, handle outside the tray, 19 versus 20 clear steps, latched violations, latch reset, fast 4-8 deg steps through the 80-84 deg crest, swing stopped before 80), logs the detent plunger compression and tip force along the real swing path (asserting the engage force inside 1.1-4.4 N and the hold against the lamp spring), checks the heel against the handle in plan along the slide and the swing, and replays both arms' plans through the kinematics at 20 Hz with every joint inside the URDF limits and IK error under 0.5 mm.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/hook-laryngoscope-blade-onto-handle/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the blue blade from the yellow card and hook it onto the orange laryngoscope handle until the light comes on, then let go." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ./logs/laryngoscope-dream
```

Scoring is automatic (`SCORE_OVER=end`). `score()` returns 0.25 when the blade is hooked (pin centre within 1 mm of the slot centreline, handle's lowest point within 10 mm of the tray floor, pin 2 mm past the end of the 3 mm mouth chamfer, inside the 4.6 mm slot and no more than 0.7 mm beyond the slot bottom, pin at mid-height of the heel within 4 mm, blade flat within 10 deg, no gripper centre (tool centre point) within 30 mm of the blade), 0.5 when also swung out 60 deg or more, and 1.0 after 20 consecutive control steps (1 s) with the blade at 85 to 95 deg, the lamp switch pressed 0.4 mm or more, the latch crossed (while hooked and turning through 60 to 89 deg the detent plunger was pushed in at least 0.3 mm over the 80-84 deg crest; this counts if a control step lands in that crest band or the angle change between steps crosses 80-84 deg on the opening swing) and seated at the end (ball behind the step, plunger travel under 0.3 mm), both parts inside the tray outline and still (under 1 cm/s, 0.5 rad/s), both grippers open at least 30 mm and their centres at least 50 mm from blade and handle. The score is 0 for the whole episode once the handle or blade leaves the table, or the pin is forced more than 1 mm into the blade's material instead of through the slot. The CLI prints `task_score: <value>` and the log keeps it at `samples[0].epochs[0].task_score`.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/hook-laryngoscope-blade-onto-handle/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the blue blade from the yellow card and hook it onto the orange laryngoscope handle until the light comes on, then let go." \
  --scorer task_score --max-steps 1600 --seed 0 --no-prompt --store-frames --log-dir ./logs/laryngoscope-demo
```

The demo is `demo()` in `task.py`, open loop and deterministic. The right hand takes the blade by its flange, turns it 40 deg out from the handle, sets it on the floor with the slot 45 mm behind the pin, slides it back onto the pin, swings it out in 4 deg steps to 92 deg (compliance presses the heel on the stop and the heel presses the lamp switch), eases back to 90 deg, lets go, backs up and withdraws. The left hand only parks out of the way, because the cradle holds the handle.

## Run limit, runtime and seeds

- Run limit: 1,600 control steps at 20 Hz (80 s of simulated time), `--max-steps 1600`.
- Runtime per trial: the demo ends at step 421 (about 21 s simulated), about 2 to 6 minutes wall time on one GPU depending on how many jobs share it; Dream-1 uses all 1,600 steps (80 s simulated), about 6 to 20 minutes wall time.
- Seeds: `--seed 0` with `-E object_jitter=0`. Dream-1 trials are not bitwise reproducible.

## Submitted videos and scores

Task file SHA-256, first 12 hex: 4e681f7701b2 (`sha256sum task.py | cut -c1-12`). Dream-1 run at this folder (m4, 1,600 steps): score 0.0; closest TCP to the blue blade 60 mm (right arm, step 1514); the arm reaches and grasps toward the blade. Demo: 1.0, termination success, 421 steps (d4 freeze).

- Videos: top camera above, left and right wrist cameras below, each at the 320 x 176 the policy sees.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

## Reset between trials

Lay the handle in the closed cradle, head toward +x, pin axis at (0.300, -0.100) m. Lay the blade on its side, heel at (0.300, -0.175) m, pointing toward -x and parallel to the handle. Check that the lamp switch stands 1 mm proud of the stop face and moves freely, and that the detent plunger moves freely with its ball resting on the heel. Printable cradle and parts list: `cad/BUILD.md`.

## Physical judging protocol

Tools: protractor, feeler gauge or ruler, stopwatch, flat tray outline. Judge at the end of the episode (or at the run limit) after the arms stop.

1. Disqualify (0 for the whole episode, whatever follows): the handle or blade has left the table at any time, or the pin was forced through the blade's material (a visible gouge or bend) instead of entering the slot. Real-trial rule: the blade must not be bent or gouged; the simulator uses the pin pushed more than 1 mm into the blade steel as its proxy.
2. Hooked (0.25): the pin stands inside the hook slot, 2 mm past the end of the 3 mm mouth chamfer, inside the 4.6 mm slot and no more than 0.7 mm beyond the slot bottom, at mid-height of the heel within 4 mm, with the pin centred in the slot width to within 1 mm (feeler gauge or ruler). Ruler check: measure from the heel's back end to the pin axis; it must be between 5 mm and 13 mm (blade origin = slot bottom, the heel back end is 12 mm behind it). The blade lies on its side, its vertical axis within 10 deg of the pin axis. The handle's lowest point is within 10 mm of the tray floor. No gripper centre (tool centre point) within 30 mm of the blade.
3. Swung (0.5): the protractor, placed on the handle with its centre on the pin, reads the blade angle from the folded position (blade along the handle). 60 deg or more.
4. Full (1.0): all of these held together for 1 s on the stopwatch.
   - Blade angle reads 85 to 95 deg on the protractor.
   - The lamp switch is pressed in 0.4 mm or more: the lamp is seen lit.
   - The latch clicked: you hear and feel the ball snap over the step during the swing, and the blade stays at the operating angle.
   - Handle and blade lie wholly inside the tray outline.
   - Both parts are still (under 1 cm/s).
   - Both grippers are open at least 30 mm and their centres at least 50 mm (ruler) from blade and handle.
   After the hold, a disclosed lift probe: a gentle push back on the blade tip, below HOLD_N, must not move it.
5. Anything else scores 0.

## Robot asset

The external **YAM robot asset is MIT licensed** (I2RT YAM via the standard Inspect / YAM embodiment). It is not redistributed here.
