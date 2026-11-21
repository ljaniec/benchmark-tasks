# Rehang a wall-mount enclosure door on its two removable-pin hinges

Enclosure doors on removable-pin hinges are hung by lining the door knuckles up between the frame knuckles and dropping the pins through. The body here is modelled on the nVent Hoffman INLINE LHC252015 (250 x 200 x 150 mm, 16 gauge steel, [datasheet](https://datasheet.octopart.com/LHC252015-Hoffman-datasheet-17579.pdf), Hoffman catalog pages 6.10-6.11, "INLINE Landscape ... Instrumentation Enclosures"; product page [LHC252015](https://us.rs-online.com/product/nvent-hoffman/lhc252015/70066796/)). The datasheet describes a "Hinged door with removable hinge pins and self-grounding quarter-turn latch system" and "Door opens 180 degrees", Type 4/12, wall mount. Loose hinge pins are sold as separate parts: the Hammond Eclipse enclosures have hidden hinges with stainless steel hinge pins, and the [Hammond Electrical Products Guide](https://www.hammfg.com/files/downloads/catalog/epg/epg-full.pdf) (pages 287-288) lists "Eclipse Replacement Hinge Pins, part no. EPINSS: These are supplied with enclosure but offered here as replacements should originals be misplaced. Stainless steel. Packaged in quantities of 5." Neither maker publishes knuckle or pin dimensions, so the hinge here is a representative 3-knuckle loose-pin hinge and its geometry is our assumption, to verify on a first build: two fixed knuckles (10 mm OD x 15 mm, 5.1 mm bore, 16 mm gap) and a 14.5 mm door knuckle between them, so the door has 0.75 mm of axial play per side, with a 4.8 mm stainless pin. The enclosure's flange trough and gasket are not modelled (their dimensions are unpublished); the body is a plain 1.5 mm steel box with plain front edges, built to the drawings in `cad/`. Rehanging needs two hands: the door rests on an acetal (POM) support block whose top is 2.5 mm below the height that puts the door knuckles in the hinge gaps (the block is disclosed and is not an alignment stop, like an installer's shim or bench support, deliberately below hinge height). The right hand slides the door along the block, lifts it by about 1.75 to 3.25 mm so the 14.5 mm knuckles fit the 16 mm gaps, and holds it in line while the left hand drops a 4.8 mm pin through three 5.1 mm bores (two fixed knuckles and the door knuckle between them), for each of the two hinges; the door is then closed.

Instruction given to the policy:

`Pick up the orange-headed hinge pins and push them fully down through both hinges of the grey enclosure door, then close the door and let go.`

Files in folder: `task.json`, `task.py` (scene, scoring, latched violations, open-loop demo), `demokit.py` (matrix TCP grips), `mechkit.py`, `arm_watch.py` (per-step finger and TCP read), `selftest.py` (offline checks), `cad/` (BUILD.md plus STL and DXF), `LICENSE`, `README.md`.

## Scene

Table frame: x forward from the midpoint of the two arm bases, y left, heights above the table top. Start layout is fixed (`object_jitter=0`).

- Grey enclosure body, 250 x 200 x 150 mm (LHC252015 outer dimensions), 1.5 mm (16 gauge) steel, powder-coated, friction 0.5, fixed. Front plane at x = 0.3066 m, back at x = 0.4566 m, spans y = -0.1300 to 0.0700 m, bottom 15 mm above the table. Mounted on a 15 mm plywood spacer (200 x 145 mm) on an 18 mm plywood backboard (500 x 400 mm, face at x = 0.4566 m). A white back panel with DIN rail and terminal blocks is visual only (no collision). The product's flange trough and gasket are not modelled.
- Two removable-pin hinges on the body's left side wall. Hinge axis x = 0.3000 m, y = 0.0750 m, centres 55 mm and 225 mm above the table top (40 mm from the body bottom and top). Each is a representative 3-knuckle loose-pin hinge (our assumption, no maker publishes knuckle geometry): two fixed knuckles (10 mm OD, 5.1 mm bore, 15 mm long, 16 mm gap between them) on a 1.5 mm leaf screwed to the side wall (leaf spans x = 303.0 to 326.6 mm in table x, one strap alongside each fixed knuckle, none across the gap).
- Grey door, 250 x 200 mm, 16 gauge steel (1.52 mm nominal, drawn as 1.5 mm; the simulated plate collider is 1.6 mm) with 12 mm return flanges (side flanges 4.5 mm in from the door edges, top flange inset 7.5 mm, bottom flange flush), 0.78 kg (a 16 gauge steel plate of this size is about 0.6 kg and the flanges about 0.12 kg; knob, hinge leaves and knuckles make up the rest), friction 0.7, free body (no hinge joint). One door knuckle per hinge (10 mm OD x 14.5 mm long, same bore, so 0.75 mm of axial play per side in the 16 mm gap) on a 1.5 mm door leaf 24 mm wide, mounted on the outer face of the door plate (leaf spans 3.0 to 27.0 mm from the hinge axis across the door width). The hinge-side plate edge is 6.6 mm from the hinge axis; at 90 degrees open with the knuckles coaxial it is 13.2 mm short of the side wall's front edge. Black molded-plastic quarter-turn wing knob (20 x 12 x 30 mm) on the outer face, 25 mm from the latch edge, mid height. Door-leaf heights: the door knuckle centres are 35.5 mm and 205.5 mm from the door bottom (44.5 mm from the door top), whereas the body hinges are 40 mm from the body bottom and top, because the door hangs 4.5 mm above the body bottom when its knuckles are centred in the gaps (3.75 mm once it rests on the lower fixed knuckles). Mounting the door leaves at 40 mm from the door ends would not work. At the start the door stands upright opened 90 degrees, hinge axis at x = 0.2600 m, y = 0.0750 m, its bottom edge 17 mm above the table (2 mm above the body bottom level).
- White acetal (POM) support block, 17 mm tall, 63 mm wide (y = 0.030 to 0.093 m), x = 0.0384 to 0.288 m, friction 0.08. The door stands and slides on it. Its top is 2.5 mm below the height that puts the door knuckles in the hinge gaps (19.5 mm), so the knuckles start 1.75 to 3.25 mm too low to enter the gaps: the right hand must lift the door by that amount and slide it the last 12 mm into line. The block is disclosed, like an installer's shim or bench support, deliberately below hinge height; it is not an alignment stop.
- Two short blue acetal (POM) guide strips (16 mm long, 20 mm wide, 30 mm tall, top at 47 mm above the table, friction 0.08) at the door's latch end, x = 0.0414 to 0.0574 m, covering the first 4 mm of the door's latch edge. The outer strip (y = 0.0715 to 0.0915 m) is 1.5 mm from the plate's outer face (y = 0.070 m). The inner strip (y = 0.0349 to 0.0549 m) is 13.5 mm from the plate's inner face (y = 0.0684 m); the latch-side flange starts 4.5 mm in from the latch edge, so it is not beside the strip. They keep the door upright at the start. The door leaves them after its first 4 mm of travel.
- The support block and the two guide strips are not an alignment stop. At coaxial the door's hinge-side edge is 13.2 mm short of the side wall's front edge, so nothing stops the slide: the right hand slides the door along the block to 12 mm short of the hinge axis, lifts it off the block, and slides it the last 12 mm until the door knuckles sit in line between the fixed knuckles (seen from above or on the wrist camera); the ball tip gathers small offsets, and a larger offset leaves the pin resting on the door knuckle until the door is moved into line. The skill tested is lifting and aligning the door, steadying it while the other hand drops the pins, then closing it. No teleports, glue, injected forces, collision filters or state machines are used.
- Two stainless hinge pins, 4.8 mm shank, 52 mm long including a ball tip, 0.012 kg, shank friction 0.3. Steel head 9 mm OD x 6 mm, press-fit on the shank, with an orange vinyl cap (generic 11 mm vinyl tube cap, 9 mm bore, 14 mm tall) pushed over the head; the simulated head collider is the 11 x 14 mm cap, friction 0.9.
- Blue foam pin block, 60 x 75 x 40 mm, friction 0.8, with two 6 mm square holes 30 mm deep at (x, y) = (0.220, 0.195) m and (0.220, 0.225) m. The pins stand head up in them with their heads 76 mm above the table top.

## Scoring

Score is the final state of the episode. Early success ends the episode after the 1 s hold. Zero conditions are latched: once set they stay set for the rest of the episode.

| Score | Condition | Human check |
|---|---|---|
| 0.25 | One hinge pinned | The door knuckle is on the pin (centre within 1 mm of the hinge axis) and lies wholly between the two fixed knuckles (no overlap more than 0.5 mm with either), door within 5 degrees of upright. A pin passes all three knuckles: head sits on the top knuckle (no visible gap over 1 mm, not more than 0.5 mm below its face), pin within 3 degrees of vertical, its head within 1 mm of the hinge axis and its tip within 3.7 mm of it (1 mm plus the 3 degree tilt over the 52 mm shank, as coded), pin tip visible at least 3 mm below the bottom knuckle |
| 0.5 | Both hinges pinned | Same check on both hinges. Either pin may go in either hinge |
| 0.75 | Both hinges pinned and door closed | A 3 mm feeler gauge does not pass between the door and the enclosure front at the top and at the bottom of the latch edge (gap at most 3 mm at both corners; the simulation also allows 1.5 mm of soft contact) |
| 1.0 | 0.75 state, door hangs from the hinges only, both grippers open and 50 mm clear, held for 1 s | Door off the support block and guide strips (lowest corner more than 2 mm above the table, door touching neither the support block nor the guide strips); the door is otherwise held only by the hinges and the enclosure front. Both gripper openings at least 40 mm and every pad point at least 50 mm from the door and the pins, for 20 consecutive steps (1 s on video). The check uses the gripper pad points; the wrist links are higher and farther from the parts |
| 0 | Any latched violation | Door tilted more than 45 degrees from upright at any time; door origin more than 50 mm below the table top or outside the table area (|x - 0.36| > 0.55 m or |y| > 0.65 m); or a pin head more than 30 mm below the table top or outside that area |

## Simulation notes and fidelity

- No hinge joint. The door turns only on the pins: pin-in-bore contact between the pin (4.8 mm shank, ball tip) and 16-sided bores (5.1 mm across flats) in the knuckles. Without both pins the door is just a loose plate.
- Friction: door 0.7 (powder-coated steel), pin shank 0.3, pin head cap 0.9, acetal (POM) support block and guide strips 0.08, foam pin block 0.8, enclosure 0.5. PhysX combines two materials as the average, so the effective pair frictions are: door on acetal block 0.39 (dry acetal on powder-coated steel, about 0.3 to 0.4), door on enclosure 0.6, pin in knuckles 0.4 to 0.5 (shank on enclosure 0.4, in the door knuckle 0.5). Door on guide strips is also 0.39. The acetal block sits below hinge height by design and is not an alignment stop (see Scene): no teleports, glue, injected forces, collision filters or state machines. Masses: door 0.78 kg, each pin 12 g, enclosure and blocks fixed.
- The quarter-turn latch is fixed and is not scored: in the real job the door is finally latched, but turning the latch is outside this task's scope and the door is left closed but unlatched (a stated limitation). There is no gasket, no inward front collar, no door stop and no hinge friction other than the pin contact.
- Under the 0.78 kg door the arm sags and tilts; the demo pre-compensates its carried pose for this (open-loop tuning, LOAD_COMP and LOAD_TILT in `task.py`). The door's weight rests on the support block while the hand steadies it and slides it, and the hand carries it while it is lifted. YAM gripper force 50 N, wrist effort 7 N m.
- The 14.5 mm door knuckle in a 16 mm gap leaves 0.75 mm of vertical clearance on each side (representative hinge, our assumption). The support block top is 2.5 mm below the height that centres the knuckles in the gaps (door bottom 17 mm against 19.5 mm), so the door must be lifted by about 1.75 to 3.25 mm (door bottom between 18.75 and 20.25 mm) to get the knuckles inside the gaps. Once the pins are in and the door leaves the block it settles onto the lower fixed knuckles, 3.75 mm above the body bottom.

## Running it

From the repository root, with the yam-arena package importable.

Offline check, no GPU:

```bash
python tasks/rehang-enclosure-door-on-pin-hinges/task.py
```

It checks: the scoring ladder on hand-built states, cheats, latched violations, the 1 s hands-clear hold and reset; collider interpenetration of every pair of separate bodies at the start, hung, pinned and closed poses and over the door's whole swing on its pins; the demo's joint plan replayed at 20 Hz (IK error, URDF joint limits, and every arm link's collision mesh against the scene, the door and the pins, fingertips may touch only the part they hold or push); and that the instruction text is identical in `task.py`, `task.json` and this README.

Dream-1 trial:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/rehang-enclosure-door-on-pin-hinges/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Pick up the orange-headed hinge pins and push them fully down through both hinges of the grey enclosure door, then close the door and let go." \
  --scorer task_score --max-steps 3000 --seed 0
```

Scripted demo: the same command with `--policy yam_scripted` instead of `--policy dream`.

Run limit 3000 steps (150 s at 20 Hz). Seeds: `--seed 0` and `-E object_jitter=0`, so the start layout is identical every trial. The demo is open loop.

## Results

- Demo (scripted, open loop): 1.0, ended by early success at step 1645 of the 3000-step limit (82.25 s). Logged every 20 steps: one hinge pinned at step 620, both at step 1080, door shut at step 1480. When the right hand draws back, the door springs back 0.8 degrees and rests 2.8 mm (bottom) and 2.7 mm (top) from the enclosure front at the latch edge, inside the 3 mm closed tolerance; both hands then stay clear for 1 s.
- Dream-1 (the only full-length Dream-1 run at this hash): 0.0, ran to the 3000-step limit. Both arms moved; the left arm reached over the blue pin block with its gripper closed, its wrist camera looking down at the orange caps, but it grasped neither pin. Closest left-TCP approach: 53 mm and 57 mm to the tops of the two pin caps (step 2716); the right TCP stayed farther from every part. Door and pins stayed at their start poses. Distances are from the measured joint state of every step (v16m1, folder hash 2891f6dedb82).
- Runtime per trial: a full 3000-step trial took 708 s (about 12 minutes) on one NVIDIA L40S shared with other jobs; the demo, which ends at step 1645, took 508 s.
- task.py SHA-256 (first 12): c865b1ce7361
- Videos: top camera (enlarged) above, left and right wrist cameras side by side below

## Demo plan

`demo()` in `task.py` is a fixed joint plan built from Cartesian waypoints. It reads no simulator state and has no branching.

1. The right hand approaches the door's top edge from beside and above, pinches it above its centre of mass and holds. It slides the door along the support block to 12 mm short of the hinge axis (PRE_LIFT_GAP), lifts it off the block to hinge height, slides it the last 12 mm so the knuckles enter the gaps, and holds it there while the left hand works. The hand-carried pose is pre-compensated for the arm's measured sag and wrist tilt under the 0.78 kg load (LOAD_COMP, LOAD_TILT; open-loop tuning). The slide targets include a small sideways offset (Y_COMP); the slide along the block also aims 6 mm past its end point (SLIDE_OVERSHOOT) to make up for the arm lagging behind the block friction, while the lifted final slide, clear of the block, aims 0.8 mm short of the hinge axis. All of these are tuned open loop.
2. Left hand waits, then picks each pin in turn by its orange cap from the pin block (upper hinge first, then lower hinge), lifts it, carries it beside the hinge and over it.
3. It lowers the pin over the hinge until the head underside is 12 mm above the seat, opens its fingers there, lifts clear, then closes them over the head and makes its push-home move before moving off the hinge. A pin may rest on top of a door knuckle that is not yet fully in line; it drops home once the door moves into line when the right hand starts swinging it toward closed. The same sequence follows for the lower hinge, then the left hand parks.
4. The right hand then lowers the door onto the lower knuckles (it aims 1.5 mm low, DOOR_PRESS), swings it from 90 to 30 degrees open in 10 degree steps, releases it, lifts off the edge, and pushes the door's outer face with closed fingertips, 110 mm from the hinge axis (PUSH_R) and 130 mm above the table, in steps (30, 20, 12, 6, 2, 0 and finally -0.8 degrees, so the last push target is 0.8 degrees past closed) until it is shut. It then eases the target back to closed, holds for 0.6 s, draws the hand 20 mm straight back from the door over 2.5 s and backs off along the hand's axis.
5. Both hands park clear of the door and the pins: left at (0.20, 0.19) m, right at (0.16, -0.20) m, grippers open.

## Physical validation

Physical validation: the hardware has not yet been built and tested. A first build must check four things: (1) the real knuckle geometry and axial play of the hinge actually used (the 14.5 mm door knuckle in a 16 mm gap is our assumption); (2) a pin drops through all three bores under its own weight with the knuckles aligned; (3) the friction of the acetal block under the door; (4) an unlatched door stays within 3 mm of closed after release.

## Real-world build

See `cad/BUILD.md` for the parts list, procedure and judging tools. Files in `cad/`:

- `BUILD.md`
- `backboard_spacer_v1.stl` (15 mm spacer)
- `door_setting_block_v1.stl` (acetal support block, 17 mm, and guide strips)
- `pin_block_v1.stl`
- `enclosure_body_v1.dxf`, `door_v1.dxf` (fabrication drawings: body, door with flanges, wing knob and door-leaf heights)
- `hinge_frame_leaf_v1.stl`, `hinge_door_leaf_v1.stl` (hinge leaves with knuckles)
- `hinge_pin_v1.dxf` (dimensioned pin, head and cap)
- `hinge_drilling_v1.dxf` (drilling template for the frame leaves, hole centres at least 5 mm inside the leaf outline)
- `layout_template_v1.dxf` (1:1 table layout)
- `gen_cad.py` (regenerates the files above)

## License

MIT, see `LICENSE`. `arm_watch.py` is adapted from an MIT-licensed task in this repository, as stated in its header.

The external **YAM robot asset is MIT licensed** (I2RT YAM). It is not redistributed here.
