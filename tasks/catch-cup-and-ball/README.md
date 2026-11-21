# Catch the ball of a wooden cup-and-ball toy in its cup

Classic dexterity toy: take a wooden cup-and-ball out of its stand by the handle, swing the ball
on its string up and over the cup, and catch it in the cup. Built for the Robocurve YAM / Isaac
Lab-Arena stack.

## Why this task

Ball-in-a-cup has been a children's toy for centuries (bilboquet, kendama's ancestor) and is a
standard benchmark for learning dynamic motor skills on robot arms (Kober and Peters, "Policy
search for motor primitives in robotics", 2009/2011). It needs things that quasi-static
pick-and-place does not test:

- The ball is only reachable through the string: the arm has to put energy into a 0.55 m pendulum
  by moving the handle at the right rhythm (the swing period is about 1.5 s).
- At the top of the swing the string goes slack and the ball flies freely; the cup has to be under
  it when it comes down, with the toy upright.
- Touching the ball with the gripper, or letting it hit the stand or the floor, ends the trial.

## Geometry and physics

All values are in `spec.py` and `scene.py`. The toy follows a commercial handmade wooden
cup-and-ball (7.5 in long, 3.5 in cup, 1.25 in ball, 21 in string, 120 g); wall, depth and handle
thickness are not published and are estimated.

- Cup: 88.9 mm outer and 76 mm inner diameter, 50 mm tall, 40 mm deep (10 mm floor), red lacquered
  wood. Modelled as a 20-sided wall and a floor disc.
- Handle: 24 mm turned wood dowel, 140.5 mm below the cup (190.5 mm overall). Toy without the
  ball: 109 g.
- Ball: 31.75 mm yellow lacquered wooden sphere, 11 g.
- String: 533 mm cotton cord, knotted to the handle's right side 10 mm under the cup, other end at
  the ball's centre. Modelled as an inextensible distance limit (slack allowed). PhysX distance
  joints engage 25 mm beyond their limit (measured), so the authored limit is shortened by that
  amount; in the recorded runs the knot-to-centre distance never exceeds the string plus the ball's
  radius (548.9 mm) by more than 0.3 mm.
- Wood on wood: friction 0.45 static, 0.40 dynamic, restitution 0.4.
- Stand (pine, fixed): 300 x 300 x 20 mm base board centred at x = 610 mm, a 40 x 40 mm post
  rising to 880 mm at x = 660 mm, a 24 x 20 mm bracket level with the post's top, and a vertical
  sleeve with a 27 mm bore, 6 mm wall and 40 mm height whose floor is 860 mm above the floor,
  with a 30° slot on its right side (-y) for the string. The sleeve's axis is at x = 500 mm,
  y = -450 mm, the same y as the post and base.
- Start: the toy stands upright in the sleeve, handle down, cup up; the ball hangs still on its
  full string, its bottom 0.43 m above the floor.
- Two YAM arms on 750 mm pedestals at y = ±305 mm. Grippers: 75 mm aperture, 40 N per finger.
  Physics 240 Hz, control 20 Hz.

## Score ladder

Measured on the toy's own frame: the ball is in the cup when its centre is inside the cup's inner
wall (within 22 mm of the axis, 2 mm tolerance) and between the cup's floor and its rim. Upright
means the handle within 30° of vertical.

- **0.25** the handle's end has been lifted 10 mm or more above the sleeve's rim.
- **0.5** with the toy lifted and upright, the ball's centre has risen above the centre of the
  cup's rim.
- **1.0** with the toy lifted and upright, the ball has stayed in the cup for 20 consecutive
  control steps (1 s).
- If a robot part touches the ball (or, in real trials, the string), the ball comes within 1 mm
  of the floor, the stand or a pedestal (so it can't be set down and scooped up), the toy falls
  below 0.30 m, the toy or ball leaves x -0.4 to 1.2 m, y -1.2 to 0.8 m, z up to 1.6 m, or a
  fixture moves more than 3 mm, the trial scores 0 for good.

Scores are latched except the invalidating conditions. PhysX gives a sphere no rolling
resistance, so a caught ball can keep circling the cup's floor; the 1 s criterion only asks that
it stay inside.

Physical judging: a camera beside the stand gives the ball's height relative to the rim and
whether it stays in the cup; a slow-motion clip shows any gripper contact with the ball or the
string. Real trials are safe: the toy weighs 120 g and the ball 11 g; keep people outside the
0.6 m swing radius.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 600 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 600 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 1.3 minutes for the demo (ball in
the cup at step 290, full score at step 310, plus the reset check), under 1 minute for the hold
check and about 2 minutes for a 600-step Dream-1 run. Seed 0, no object jitter. Demo observations
are three RGB cameras only; Dream also receives standard joint/gripper readings. No object poses
or scorer state reach the policy.

The scripted demo grasps the handle from the side, lifts the toy out of the sleeve, carries it
to a play position with a two-period raised-cosine profile so the ball does not swing, then
pumps the handle horizontally and vertically and follows a short rehearsed path to the catch.
The swing was searched offline on a pendulum model of the arm and string and confirmed in this
scene; `demo.py` replays it open loop.
