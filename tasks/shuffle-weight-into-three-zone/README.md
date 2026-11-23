# Table shuffleboard: one shot into the 3-point zone

A YAM robot stands at the near end of a 9-ft table shuffleboard. One red-capped weight stands on the board
in front of its right hand. The robot must slide the weight down the board with one push, so that it stops
in the 3-point zone at the far end: completely past the 3-line (6 in from the end) and still on the board.

## Why this task

Table shuffleboard is a fixture of bars, clubs and game rooms. The 9-ft size is sold for apartments and
small bars. Each shot is a single release. On a waxed board the weight's travel grows with the square of
its speed, so to stop in a 6-inch zone after 2 m the hand must let go at the right speed (within about
±1.5%) and straight down the board. It must also let go cleanly, with the weight flat and on the board. The
skill is open-loop speed control at the moment of release, with no chance to correct afterwards.

The closest comparators are different skills:
- Embodied Olympics' curling has a humanoid deliver a 20 kg stone from the hack on ice, with spin and sweeping
  rules, in a match.
- The air-hockey task on the board strikes an incoming puck with a held mallet.
- Bowling tasks roll a ball into pins.
- The prize-wheel task flicks a wheel on a fixed axle so it coasts round to a segment.

Here a 345 g weight is held in the fingers, slid along a board by an arm, and released.

## Scene

- Board: a 9-ft table's playfield, 7 ft 8 in × 20 in (2.337 × 0.508 m) of waxed maple. Its top is 30 in
  (0.762 m) above the floor. The far end is 2.62 m from the robot's base line.
- Cradle: 31 in wide and 9 ft long overall, with gutters 100 mm wide along the sides and 200 mm long at
  each end, 65 mm below the board. A weight that goes off the board drops into them.
- Lines: painted strips 6 mm wide, drawn only (they have no thickness the weight can feel):
  - the single foul line across the middle of the board, as on 9, 12 and 14 ft tables;
  - at each end, the 3-line 6 in and the 2-line 12 in from the end.
  The far zones carry three, two and one red dots for their points.
- Weight: a chrome-plated steel puck 2 5/16 in across (58.7 mm) and 0.75 in (19.05 mm) high, with a red
  plastic cap 55 mm across and 13 mm high. It weighs 345 g, with its centre of mass 11.6 mm above its base.
  It starts at x = 0.315 m on the board's centre line (y = −0.305 m), straight ahead of the right arm,
  35 mm from the near end of the playfield.
- Robot: two unmodified YAM arms on 750 mm pedestals at x = 0, y = ±0.305 m, with 40 N parallel grippers.
  The cradle's near end is 5 mm in front of the pedestals.

## Physics

- PhysX at 960 Hz, control at 20 Hz.
- Waxed board against anything: friction 0.02 (the board's material combines by the minimum). A weight
  let go at 0.92 m/s slides 2.16 m, decelerating at 0.20 m/s².
- Steel puck against the gripper pads: friction 0.75, restitution 0.
- Shuffleboard friction depends on the wax (sold in speeds, such as 1 to 4) and on humidity. Published
  examples range from 0.012 (a puck slowing from 1.0 to 0.2 m/s over 4 m) to 0.080 (a textbook pub table).
  We use 0.02, a fast board, which suits a short 9-ft table.
- Approximations:
  - the board is perfectly flat, without the slight crown that real climate adjusters set;
  - the wax is a uniform friction, not loose beads;
  - the puck is a straight cylinder, without its slight taper;
  - there is no air drag;
  - the gutters are plain wood, not padded.

## Score

The trial scores **0** if:
- the weight is ever more than 5 mm above the board with nothing touching it (a throw, not a slide);
- the robot touches the weight again after it has left the hand sliding at 0.1 m/s or faster (after the
  shot);
- the weight leaves the board after the shot (into a gutter);
- or the table moves or the weight leaves the workspace.

Otherwise the score is set by where the weight comes to rest after the shot. It must be flat (tilted less
than 10°), untouched, and slower than 5 mm/s for 1 s. As in the game, a weight touching a line counts the
lower zone.

| Score | Weight at rest | Points in the game |
|---|---|---|
| 1.0 | completely past the 3-line, including a "hanger" over the end | 3 (4) |
| 0.5 | completely past the 2-line | 2 |
| 0.25 | completely past the foul line | 1 |
| 0 | short of that | dead weight |

On a real table every condition can be checked with the board's painted lines and a side video: whether
the weight left the board, whether it was in the air, and whether the hand touched it after the release.

## Reset and board speed

Between trials, stand the weight back on the centre line 35 mm from the near end and brush the playfield
smooth. Before each set of trials, check the board's speed so it matches the simulated friction of 0.02:
1. Push a weight down the board by hand and film it at 60 fps beside a tape measure on the board.
2. Read its speed from its positions 1 s apart. It must slow by 0.18 to 0.22 m/s each second.
3. Sprinkle on silicone shuffleboard wax to make the board faster, or wipe some off to make it slower, and
   check again.

## Run

```bash
export ISAAC_PYTHON=/path/to/IsaacLab/.venv/bin/python INSPECT_OVERLAY=/path/to/inspect-overlay ARENA_ROOT=/path/to/IsaacLab-Arena
./run_inspect.sh --output runs/demo --policy demo --steps 600 --check-reset
./run_inspect.sh --output runs/hold --policy hold --steps 200
./run_inspect.sh --output runs/dream --policy dream --steps 600 --budget-ledger runs/dream-budget.jsonl
python audit_run.py runs/demo --require-full
PYTHONPATH=. python -m unittest -q test_contract test_dream_budget
```

One trial is one episode, seed 0, with no randomisation. A 600-step trial takes about 3 minutes on a single
GB10 GPU.

## Demonstration

`demo.py` is an RGB-only scripted rehearsal planned with offline IK. It never reads object poses.
1. The right hand comes down over the weight and takes it by its sides (fingers pointing down, closing across
   the board), with the pad tips 2 mm above the board.
2. Keeping the weight on the board, the hand slides it straight down the centre line. It is commanded to
   1.225 m/s over 90 mm; the arm lags, and the weight actually reaches 0.92 m/s.
3. The fingers open at that speed (control step 123), the hand stops 0.1 m further on and lifts away, and
   the weight slides on alone. It stops at x = 2.562 m, 55 mm from the far end, with its back edge 65 mm
   past the 3-line.
4. The left arm stays parked beside the table.

The release speed and timing were rehearsed in this scene.

## Sources

- Shuffleboard Federation, official table shuffleboard rules (weight: 2.3125 in across, 0.75 in high,
  310–355 g in all): https://www.shuffleboardfederation.com/rules.html
- Shuffleboard Federation FAQ (lines at 6 and 12 in; a single foul line in the middle of 9, 12 and 14 ft
  tables; weights must be completely past a line; hangers): https://www.shuffleboardfederation.com/faq.html
- Champion 9-ft table (playfield 7 ft 8 in × 20 in, 30 in to the top of the playfield, cradle 31 in wide):
  https://www.shuffleboardfederation.com/9-foot-championship-line-shuffleboard-table.html
- McClure regulation weights (chrome-plated, 344–345 g with plastic caps):
  https://mccluretables.com/products/weights-2-516-custom-pucks-plastic-caps
- Table sizes (9 ft for apartments and small bars; plays fast):
  https://www.apollostemplates.com/sports/table-games/shuffleboard/dimensions/
- Table shuffleboard (silicone wax lowers friction; scoring): https://en.wikipedia.org/wiki/Table_shuffleboard
- Friction examples: https://www.physicsforums.com/threads/coefficient-of-kinetic-friction-from-shuffleboard-puck-deceleration.199948/
  and https://www.pearson.com/channels/physics/asset/5d26ff16/tabletop-shuffleboard-is-a-popular-pub-game-where-players-slide-pucks-across-a-t
