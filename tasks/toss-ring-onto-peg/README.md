# Toss a ring onto the centre peg of a ring toss set from behind a line

Fairground and backyard game: pick up a plastic ring from a side table and toss it, from behind a
taped line, onto the centre peg of a toddler's ring toss set lying on the floor. The peg is out of
reach of both arms, so the ring has to be thrown. Built for the Robocurve YAM / Isaac Lab-Arena stack.

## Why this task

Ring toss is played at fairs, parties and in back gardens from toddler age. Landing a ringer needs
things that pick-and-place does not test:

- The target is 0.82 m in front of the arm bases (the arms reach about 0.6 m) and 0.37 m beyond the
  throw line, so the ring can only get there in free flight.
- The release has to set the ring's speed, direction and attitude at once: it has to come down over
  the peg roughly flat, with its centre within 41 mm of the peg's axis, or it bounces off.
- The arm has to accelerate hard and then stop behind the line.

## Geometry and physics

All values are in `spec.py` and `scene.py`. Origin on the floor midway between the two pedestals'
centres; x forward, y to the left, z up.

- Ring toss set: Kiddie Play Ring Toss Game (Amazon ASIN B0CMR4MDYF; 16 x 16 in cross base, five
  pegs, 7 in tall assembled, five 5 in rings, 9.6 oz in all). Modelled as one loose plastic body
  lying on the floor: a cross of two 406 x 40 x 22 mm arms, and five 25 mm round-tipped pegs whose
  tips are 178 mm above the floor, one in the centre (red) and one 180 mm out along each arm. The
  set without its rings weighs 192 g. The centre peg stands at x = 820 mm, y = -305 mm (straight
  ahead of the right arm), with the cross's arms along x and y. The peg diameter and the arms'
  section are not published and are estimated.
- Ring: 127 mm outer diameter, 10 mm round tube (107 mm hole), 16 g, pink plastic. Modelled as 16
  overlapping capsules forming one rigid hoop.
- Side table: 200 x 240 mm wooden top, 840 mm high, centred at x = 330 mm, y = -600 mm (to the
  right of the right arm), so its back edge (towards the robot) is at x = 230 mm. The ring lies
  flat on it, centred at x = 260 mm, y = -600 mm, so its back 33.5 mm hangs over the table's back
  edge.
- Throw line: 50 mm white tape across the floor, near edge at x = 450 mm.
- Plastic on plastic and on the floor: friction 0.35 static, 0.30 dynamic, restitution 0.3.
- Two YAM arms on 750 mm pedestals at y = ±305 mm. Grippers: 75 mm aperture, 40 N per finger.
  Physics 240 Hz, control 20 Hz.

## Score ladder

- **0.25** the ring touches any part of the set (base or a peg).
- **0.5** the ring touches the centre peg.
- **1.0** the ring ends up around the centre peg (the peg's axis passes through the ring's hole,
  below the peg's tip) and stays there for 20 consecutive control steps (1 s).
- The trial scores 0 for good if any part of the robot is past the throw line's near edge
  (x > 450 mm) at any time, a robot part touches the ring after the ring has touched the floor or
  the set, the ring or the set leaves x -0.4 to 2.2 m, y -1.4 to 1.0 m, z up to 1.8 m, or a
  fixture moves more than 3 mm.

Scores are latched except the invalidating conditions. In simulation a touch is a gap of 1 mm or
less between the ring's surface and the set's, checked at every physics step. Robot parts past the
line are found with an overlap query against the space beyond the line, using the robot's
collision shapes.

Physical judging: a camera at floor level beside the set shows which parts the ring touches and
whether it ends around the centre peg (lift the set by its base afterwards if in doubt: a ringer
comes up with it); a camera along the throw line shows whether any robot part crosses it. Real
trials are safe: the ring weighs 16 g and the throw stays below 3 m/s; keep people out of the
2 m in front of the robot.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 360 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 600 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 1 minute for the demo (ring around
the centre peg at step 211, full score at step 231, plus the reset check), under 1 minute for the
hold check and about 2 minutes for a 600-step Dream-1 run. Seed 0, no object jitter. Demo
observations are three RGB cameras only; Dream also receives standard joint/gripper readings. No
object poses or scorer state reach the policy.

The scripted demo pinches the ring's overhanging back rim from above (fingers closing front to
back), lifts it, carries it in front of the right shoulder and throws it forward and 25° up along
a straight line, commanding 2.4 m/s; the fingers open 0.05 s after the commanded peak speed and the
hand stops within 0.06 s. The throw was tuned in this scene (commanded speeds from 2.2 to 2.6 m/s
and openings 0.03 to 0.05 s after peak all ring the peg); `demo.py` replays it open loop.
