# Start a Newton's cradle

Desk toy: lift the ball nearest the robot out along its swing with its strings taut, and let it go so that
only the ball at the far end swings out. Built for the Robocurve YAM / Isaac Lab-Arena stack.

## Why this task

Newton's cradles are the classic executive desk toy and a standard classroom physics demonstration, and
starting one is the only thing anyone does with it. Doing it so that a single ball flies out at the far end
tests things a pick-and-place does not:

- Ball 1 hangs on two strings in a V, so it can only move along a 117.5 mm arc. The hand has to follow
  that arc: pull too little and the strings go slack, pull sideways and the ball twists.
- Its neighbour hangs 1 mm away. The grasp and the lift must not touch it, and the other balls must
  still be hanging still when ball 1 is let go.
- The release has to be from rest: a push, a flick or a twisted ball arriving off-line spreads the impact
  over several balls instead of passing it cleanly to the last one.

## Geometry and physics

All values are in `spec.py` and `scene.py`. Origin on the floor midway between the two pedestals'
centres; x forward, y to the left, z up.

- Table: wooden, 500 x 600 mm top, 760 mm high, centred at x = 520 mm, y = 0.
- Cradle: 0.8 kg in all, standing loose on the table (it is not fixed down). Real large desk cradles are
  this size: 25 mm (1 inch) balls on a 9 x 9 x 7 inch frame; common 18 cm cradles use 19-22 mm balls.
- Plinth: black wood, 260 x 180 x 20 mm, centred on the table, long side along x.
- Frame: chrome-plated 6 mm steel rods. End frames at x = 398 and 642 mm, each with two legs rising from
  y = +/-90 mm at the plinth to y = +/-45 mm at a crossbar 940 mm above the floor; two rails along x at
  y = +/-45 mm join them.
- Balls: five 25 mm chrome steel balls, 64 g, centres at x = 468, 494, 520, 546 and 572 mm (1 mm apart),
  y = 0, 822.5 mm above the floor (30 mm clear of the plinth). Each hangs by two 0.6 mm nylon strings,
  114 mm long, from the rail points beside its centre to the top of the ball, so it swings only along
  x, on a 117.5 mm radius about the line between its two anchors. Ball 1 is nearest the robot.
- Steel on steel: restitution 0.97, friction 0.1. With touching balls, PhysX solves the row as one block
  (ball 1 rebounds and several balls swing together; the GAUGE physics benchmark reports the same for Isaac
  Sim). The 1 mm gaps are wider than the balls' contact distance, so PhysX passes each impact ball to
  ball: from a 35 degree release ball 5 swings out to about 33 degrees, about 93% of ball 1's speed,
  as GAUGE measured on a real cradle.
- Two YAM arms on 750 mm pedestals at y = +/-305 mm. Grippers: 75 mm aperture, 40 N per finger.
  Physics 960 Hz (a ball moving at 0.65 m/s travels under 1 mm per step), control 20 Hz.

## Score ladder

Angles are a ball's swing from hanging straight down, along the row; ball 1 positive towards the robot,
ball 5 positive away from it.

- **0.25** while the robot holds ball 1, ball 1 is at least 20 degrees out with both strings taut and
  balls 2-5 within 3 degrees of hanging. Taut: neither string sags 1 cm or more (in simulation, ball 1's
  centre within 2.5 mm of its swing circle; a 114 mm string 2.5 mm short of straight sags about 1 cm).
- **Release.** The robot stops touching ball 1, and at its last touch ball 1 was at least 20 degrees out,
  strings taut, balls 2-5 within 3 degrees, and ball 1 moving under 0.1 m/s (about 3 mm between frames
  at 30 frames/s). Only ball 5's first swing
  out after the release counts; it ends when ball 5 falls 1 degree back from its peak. Touching ball 1
  before then cancels the release; a later release can still score.
- **0.5** that swing reaches at least 10 degrees.
- **1.0** that swing reaches at least 15 degrees and balls 2-4 stay within 6 degrees from the release
  until it ends. A clean release from 20 degrees sends ball 5 to about 18.6 degrees.
- The trial scores 0 for good if the robot touches any of balls 2-5, any plinth corner or rail end moves
  more than 3 mm from its start, the table or pedestals move more than 3 mm, or anything leaves
  x -0.4 to 1.4 m, y -1.4 to 1.0 m, z 0 to 1.6 m.

The highest level reached counts. In simulation a touch is any robot contact reported by PhysX, checked
at every physics step.

Physical judging: a camera level with the balls, looking along y, with a protractor printed on a card
behind the cradle, shows every angle; slow-motion playback shows whether the strings were taut and the ball
still at the release. Tape marks at the plinth corners show whether the cradle moved. Real trials are
safe: the cradle weighs under 1 kg.

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

Runtime per trial on one GPU, including scene start-up: about 70 seconds for the demo (full score at
step 181, plus the reset check), about 1.5 minutes for the hold check and about 3.5 minutes for a 600-step
Dream-1 run. Seed 0, no object jitter. Demo observations are three RGB cameras only; Dream also receives standard
joint/gripper readings. No object poses or scorer state reach the policy.

The scripted demo brings the right hand in level from the robot's side, fingers pointing along the row
and closing side to side on ball 1's equator. It lifts the ball 35 degrees along its swing circle (aimed 6 mm
outside it: the simulated arm settles a few mm short of its targets, and the strings must stay taut; the
ball turns about the pinch axis), holds it for 1 s, opens the
fingers and backs away 80 mm. `demo.py` replays it open loop. Setting `CRADLE_TEST_LIFT_DEG` starts ball 1
already swung out, for checking the physics with the hold policy; it is unset for scored trials.

## Sources

- Familiarity: https://en.wikipedia.org/wiki/Newton%27s_cradle (its most common use is as a desktop
  executive toy); https://www.phys.vt.edu/outreach/projects-and-demos/demonstrations-wiki/mechanics/newtons-cradle.html
  ("Most people have seen it"); https://www.theguardian.com/artanddesign/2024/oct/20/newtons-cradle-executive-toy-ballrace
- Sizes: https://www.xump.com/science/newtonscradle.cfm (7 x 7 x 4.75 in, 22 mm balls, 0.9 kg);
  https://www.analyticalsci.com/c/products/the-famous-newtons-cradle-handcrafted-wood-the-original (1 inch balls)
- Physics fidelity: GAUGE, https://arxiv.org/html/2608.05948 (real cradle momentum transfer 93%; default
  Isaac Sim solves the row as a block)
