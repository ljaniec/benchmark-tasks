# Catch the ball of a wooden cup-and-ball toy in its cup

Classic dexterity toy: take a wooden cup-and-ball out of its stand by the handle, swing the ball
on its string up and over the cup, and catch it in the cup. Built for the Robocurve YAM / Isaac
Lab-Arena stack.

## Why this task

Ball-in-a-cup has been a children's toy for centuries (bilboquet, kendama's ancestor) and is a
standard benchmark for learning dynamic motor skills on robot arms (Kober and Peters, "Policy
search for motor primitives in robotics", 2009/2011). It needs things that quasi-static
pick-and-place does not test:

- The arm moves the handle rhythmically to put energy into the 0.55 m pendulum.
  Its swing period is about 1.5 s.
- At the top of the swing the string goes slack and the ball flies freely; the cup has to be under
  it when it comes down, with the toy upright.
- Touching the ball with the gripper, or letting it hit the stand or the floor, ends the trial.

## Geometry and physics

All scene dimensions are in `spec.py` and `scene.py`. Prepare the toy with a digital scale,
calipers and a ruler. Use ordinary wood stock, a turned dowel, an unfinished wooden craft ball
and cotton cord. Finish the cup and ball with smooth, dry lacquer in contrasting colors.

1. Turn the cup blank to 88.9 ± 1 mm diameter and 50 ± 1 mm height. Bore a centered
   76 ± 1 mm opening to a depth of 40 ± 1 mm. Keep the bottom flat and smooth the rim and bore.
2. Bond a 24 ± 0.5 mm wood dowel concentrically below the cup with 140.5 ± 1 mm of dowel exposed.
   Select wood stock that gives 109 ± 8 g for the finished cup and handle together.
3. Select a 31.75 ± 0.5 mm wooden ball weighing 11 ± 1 g. Fix a 2.4 ± 0.2 mm cotton cord into
   a shallow hole at its surface. Secure the other end against the right side of the handle
   10 ± 1 mm below the cup. Under a 1 N pull, adjust the exposed cord to 533 ± 2 mm from its
   exit at the handle knot to its exit on the ball; this measurement excludes the knots and
   any cord inside the wood. Mark the two exits so the gauge check can be repeated.
4. Measure the finished parts after the finish dries. Stand the handle in the sleeve with the
   cord through the side slot and let the ball hang freely. The ball center should settle at
   about 442 mm above the floor. Repeat the same fit and hanging check at reset.

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
- The trial scores zero for good if a robot touches the ball (or touches the string in a
  physical trial), the ball visibly hits the floor, stand or a pedestal, the toy falls below
  0.30 m, the toy or ball leaves x −0.4 to 1.2 m, y −1.2 to 0.8 m, or z above 1.6 m,
  or a fixture moves at least 3 mm. Contact between the ball and its own cup is permitted.

Scores are latched except the invalidating conditions. PhysX gives a sphere no rolling
resistance, so a caught ball can keep circling the cup's floor; the 1 s criterion only asks that
it stay inside.

Film from beside the stand to judge ball height relative to the rim and the one-second hold.
Use a second oblique view and slow motion at 60 fps or faster to check visible contact with the
robot or fixtures. A bounce, deflection, or visible contact surface is enough. Close passes earn
the score they otherwise achieve. In simulation, a physical contact impulse at any physics
substep supplies the fixture-contact flag. Keep people outside the swing area.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

The revised evaluation uses Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2,
Isaac Lab-Arena 0.3.0 at `c7edf3a9d5` and Inspect Robots 0.60.0. Set
`ISAAC_PYTHON`, `ARENA_ROOT` and `INSPECT_OVERLAY` for the installed runtime and
the dependencies listed in `DEPENDENCIES.json`.

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

The revised demo scored 1.0 in 309 steps, with the ball held in the cup for 20 consecutive
steps. It took 102.1 seconds through the end of the episode. A separate ball drop
registered contact with the floor and invalidated the trial even though the ball had
already bounced above the floor by the next control observation. The repeat reset restored
the body poses, zero velocities, score counters and starting RGB frames exactly.
Dream-1 beta scored 0.0 in 600 steps and took 192.6 seconds. Use seed 0 and no object jitter.
Demo observations
are three RGB cameras only; Dream also receives standard joint/gripper readings. No object poses
or scorer state reach the policy.

The scripted demo grasps the handle from the side, lifts the toy out of the sleeve, carries it
to a play position with a two-period raised-cosine profile so the ball does not swing, then
pumps the handle horizontally and vertically and follows a short rehearsed path to the catch.
The swing was searched offline on a pendulum model of the arm and string and confirmed in this
scene; `demo.py` replays it open loop.
