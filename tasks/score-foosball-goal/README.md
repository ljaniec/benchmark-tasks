# Score a foosball goal past the keeper

Table football: with the handle of its three-man rod, the robot slides the rod to line a man up
behind the ball and turns the rod quickly through the ball to shoot it into the opponents' goal,
past their keeper, without spinning the rod. Built for the Robocurve YAM / Isaac Lab-Arena stack.

## Why this task

Foosball tables stand in bars, student unions, offices and family rooms. A shot is made only
through the rod, which tests things pick-and-place does not:

- The gripper has to hold a 32 mm handle with its tool axis on the rod's axis, so that the wrist
  roll turns the rod instead of fighting its bearings.
- The ball is not in front of any man: the rod has to slide 50 mm first, then the men's feet have to
  swing through the ball fast enough to carry it 0.37 m into a 203 mm goal mouth.
- The rules forbid spinning (turning the rod more than 360 degrees), so the strike has to be a
  controlled wind-up and swing.

## Geometry and physics

All values are in `spec.py` and `scene.py`. Origin on the floor midway between the two pedestals'
centres; x forward, y to the left, z up. The table's long axis runs along y and the robot plays from
its near long side, like a person.

- Table, sized like a Tornado Sport: green 1200 x 750 mm playfield 830 mm above the floor (about
  40 mm higher than the real table, so the handle is within the arm's reach) on a wooden cabinet
  and legs; 20 mm wooden walls, 120 mm high at the sides and 100 mm at the ends, with 203 x 75 mm
  goal mouths and closed goal boxes. The near side wall's inner face is at x = 600 mm; the table is
  centred at y = -80 mm, so the opponents' goal line is at y = -680 mm (the robot's right).
- Rods: eight 15.9 mm steel rods 150 mm apart, axes 70 mm above the playfield. Men are 22 mm wide
  (along the rod) with 16 mm bodies and 25 x 20 x 22 mm feet; their toes clear the playfield by
  3 mm. The robot's team is red, with its handles (32 mm black capsules, 110 mm long) on the near
  side; the opponents are blue.
- The robot's three-man rod (men 184 mm apart, 0.64 kg with handle and men) is at y = -305 mm, in
  front of the right arm. It slides up to 95 mm each way from centred and turns freely; its handle's
  centre is 150 mm outside the near wall (x = 450 mm) when the rod is centred. It starts centred with
  its men hanging straight down.
- Opponents: the two-man rod (men 241 mm apart, centred) at y = -455 mm and the keeper rod at
  y = -605 mm, its one man 60 mm nearer the robot than the table's centre line. These, and the five
  other rods, are held still with their men hanging straight down.
- Ball: 35 mm hard plastic foosball, 20 g, white, resting on the playfield at x = 1025 mm (50 mm
  beyond the middle man), y = -336.5 mm (just in front of the men's feet, towards the goal).
- Plastic friction 0.30 static, 0.25 dynamic; ball restitution 0.5.
- Two YAM arms on 750 mm pedestals at y = ±305 mm. Grippers: 75 mm aperture, 40 N per finger.
  Physics 240 Hz, control 20 Hz.

## Score ladder

- **0.25** the ball has moved 50 mm or more towards the opponents' goal.
- **0.5** the ball has got past the opponents' keeper rod.
- **1.0** the ball has gone into the opponents' goal (its centre through the mouth and behind the goal
  line).
- The trial scores 0 for good if the three-man rod ever turns more than 360 degrees from its
  starting angle either way (spinning), a robot part touches the ball, any robot part enters the
  space over the playfield (between the walls, up to 250 mm above the playfield), the ball leaves
  the table other than into the goal, or a fixture moves more than 3 mm.

Scores are latched except the invalidating conditions. The rod's turn is accumulated at every
physics step, and the space over the playfield is checked with an overlap query against the
robot's collision shapes.

Physical judging: a camera above the table shows the ball's travel (tape marks at 50 mm and at the
keeper rod) and whether it drops into the goal; a mark on the handle, filmed from the end, shows the
rod's turn. Real trials are safe: the ball weighs 20 g and the robot only holds a handle outside the
table.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 240 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 400 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 1 minute for the demo (goal at step
196, plus the reset check), under 1 minute for the hold check and about 1.5 minutes for a 400-step
Dream-1 run. Seed 0, no object jitter. Demo observations are three RGB cameras only; Dream also
receives standard joint/gripper readings. No object poses or scorer state reach the policy.

The scripted demo slides its open fingers over the end of the handle along the rod's axis, closes
on it (fingers left and right of the handle), pushes the rod in 50 mm so the middle man is behind
the ball, rolls the wrist back 70 degrees and then forward to 60 degrees past vertical in one
command; `demo.py` replays it open loop.
