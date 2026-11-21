# Slide a high-chair tray in to fit a seated toddler

Everyday childcare task: the tray of a high chair is locked in its outermost position. Hold the
tray's front release up while pushing the tray toward the toddler, stop at the closest of its four
positions that does not touch the manikin, and let both locking fingers click into their armrest slots.
Built for the Robocurve YAM / Isaac Lab-Arena stack.

## Why this task

Graco's [Made2Grow owner's manual](https://s7d9.scene7.com/is/content/NewellRubbermaid/DASH/S7_int/2020/NWL0000997155C%203AD.pdf)
describes exactly this adjustment: "Press button on front of tray and push towards seat to adjust tray
positions. There are four adjustment options." Its warnings add "DO NOT FORCE TRAY against child. Use
only the four adjustment positions. Be sure both adjustment fingers are engaged in armrest slots."

What is distinct here:

- The tray is already mounted and locked. It only moves while the release is held up, so one gripper
  has to keep a button pressed while it pushes the tray 60 mm.
- The stopping point is not named. It is set by the child: the closest position whose rear rim
  clears its belly (20 mm at position 3; position 4 would press 10 mm into it).
- A tray that is only pushed, never released, stays between slots, and contact with the manikin at any
  moment ends the trial at zero.

## Geometry and physics

Full-size reconstruction proportioned after the Graco Made2Grow; not manufacturer CAD. All values are
in `spec.py` and `scene.py`.

- Chair: seat top 560 mm above the floor, 280 x 320 mm seat, backrest to 1,080 mm, footrest at
  380 mm, four splayed 28 mm legs. Seat front at x = 660 mm.
- Armrest rails: 32 mm wide, 30 mm tall, top at 720 mm, centred at y = ±170 mm, x = 460-800 mm.
  Each has four 18 mm long, 8 mm deep slots in its underside at x = 620, 650, 680 and 710 mm and
  moulded tick marks on its outer face.
- Tray: 300 x 440 mm, 12 mm plate, 20 mm rim with a 120 mm thumb gap at the front centre, 0.65 kg,
  riding on the rails through two underside channels with 3 mm side clearance. Tray-on-rail friction
  0.35 static, 0.30 dynamic.
- Release bail (one rigid 60 g part, native revolute joint 43 mm under the plate): an 80 mm wide
  blue button sticking 22 mm out under the front edge, two arms and two 12 x 20 x 16 mm locking
  fingers. Pressing the button up to the 6° stop lowers the fingers 15 mm, clear of the slots; a
  return spring (0.25 N·m per degree) lifts them back.
- Toddler manikin (12-18-month size), rigid and fixed in the seat, built from capsules and
  spheres: 170 mm wide, 120 mm deep torso, 150 mm head, thighs, shins, feet and arms. The front of
  its belly is at x = 750 mm.
- Tray positions (centre x): 520, 550, 580, 610 mm. Rear-rim clearance to the belly: 80, 50, 20 and
  -10 mm. The tray starts locked in position 1.
- Grippers: 75 mm aperture, 40 N per finger. Physics 240 Hz, control 20 Hz.

## Score ladder

A finger is locked in a slot when it is within 4 mm of the slot's centre along the rail and its top
is at least 3 mm up inside the slot. Both fingers must be in the same slot.

- **0.25** the tray has moved at least 10 mm in from position 1.
- **0.5** both fingers locked in position 2 or 3.
- **1.0** both fingers locked in position 3, both grippers at least 5 mm clear of the tray and bail,
  tray and bail under 10 mm/s and 2°/s for ten consecutive 20 Hz frames.
- If the tray, the bail or a gripper ever comes within 1 mm of the toddler, the tray drops below
  600 mm, leaves the published workspace, or fixed scenery moves more than 3 mm, the trial scores 0
  for good.

Physical judging: the tray's side pointers against the armrest ticks give the position; a light pull
on the tray confirms both fingers are engaged; a paper strip or video shows any contact with the
manikin. Real trials use a toddler-size manikin or weighted doll (about 80 cm tall, 9-12 kg),
never a child, harnessed so that a 20 mm spacer block just fits between its belly and the rear
rim with the tray locked in position 3.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 340 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 1200 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 1.5 minutes for the demo (full score
at step 252 with the reset check) and about 27 minutes for a 1,200-step Dream-1 run. Seed 0,
object jitter 0. Demo observations are three RGB cameras only; Dream also receives standard
joint/gripper readings. No object poses or scorer state reach the policy.
