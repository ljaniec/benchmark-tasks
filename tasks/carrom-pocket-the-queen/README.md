# Carrom: pocket the queen with one strike of the striker

A YAM robot sits at one side of a full-size carrom board. The striker lies on the robot's base lines and the red
queen stands 6 cm short of the far left pocket, in line with the striker. The robot must strike the striker
with its fingers, from the base lines, so that it slides 0.63 m across the powdered board and knocks the queen
into the pocket, without the striker following it in.

## Why this task

Carrom is one of the most played table games in South Asia, found in homes, clubs and cafés, with
international rules and championships under the International Carrom Federation (ICF). Every shot is the same
act: the player places the striker on their base lines and flicks it with a finger. The striker must be struck,
not pushed, and nothing may touch it once it has left the base lines. To pocket a piece, the hand must send a
15 g disc along a line to within about 1 cm at 0.63 m, and at a speed in a narrow band: too slow and the
queen stops short; too fast and the striker follows it into the pocket (a "due", a foul in the game). The skill
is an open-loop strike with the bare hand: a short, fast contact that sets both the direction and the speed of a
free sliding disc, with no tool to guide it and no chance to correct.

The closest comparators are different skills:
- Pool tasks (such as the pending 8-ball shot and the rejected 1-ball pot) stroke a cue held in one hand and
  bridged by the other into a rolling ball. Here there is no cue and no bridge: the fingers hit the striker
  directly, and it must start from a fixed strip on the board.
- Table shuffleboard slides a held weight and lets go; the hand controls the weight until release.
- Air hockey strikes a moving puck with a held mallet; curling (Embodied Olympics) delivers a held stone.
- Prize-wheel and spinner tasks flick an object turning on a fixed axle.

## Scene

- Board: a full-size tournament carrom board (as in Precise's or Surco's ICF-approved boards): a plywood playing
  surface 74 cm square and 10 mm thick, dusted with carrom powder, in a rosewood frame 7 cm broad that stands
  2.2 cm above the surface. The surface's top is 0.762 m above the floor, from x = 0.220 to 0.960 m and y = −0.370
  to +0.370 m, on a four-legged stand.
- Pockets: four round holes 4.45 cm across in the corners, their centres 24.25 mm in from both sides of the frame,
  with a net (here a rigid box 55 mm deep) under each.
- Markings, drawn only (no thickness): on each side two base lines 47 cm long, the outer one 10.15 cm from the
  frame and 6 mm thick, the inner one 3.18 cm from the outer one's outer edge, closed at both ends by base
  circles 3.18 cm across with red centres; four arrows from the base lines towards the pockets; a centre circle
  3.18 cm across and an outer circle 17 cm across.
- Striker: a smooth disc 41.3 mm across, 9 mm thick and 15 g (the largest the ICF laws allow), dark blue,
  with bevelled edges.
  It lies flat on the robot's base lines, touching both, its centre at x = 0.3405 m, y = −0.0682 m.
- Queen: a red wooden carrom man 31.5 mm across, 8 mm thick and 5.5 g, its centre at x = 0.8865 m, y = 0.3115 m:
  6 cm short of the far left pocket's centre (0.9358, 0.3458), on the straight line from the striker's centre
  through the pocket's. No other carrom men are on the board.
- Robot: two unmodified YAM arms on 750 mm pedestals at x = 0, y = ±0.305 m, with 40 N parallel grippers. The
  line from the striker to the queen points straight away from the right arm's base.

## Equipment and cost

- An ICF-approved tournament board with its striker, carrom men and powder: for example the Surco Champion
  (Speedo) board, $184.99 from a US retailer, or the Precise Champion 16 mm board, $249 on Amazon (links under
  Sources). Any board meeting the ICF laws gives the same layout; check its speed before each set of trials (below).
- A stand or sturdy table that puts the playing surface 0.762 m above the floor, beside the robot's pedestals.
- A phone that films at 60 and 240 fps, and a tape measure.

## Physics

- PhysX at 960 Hz, control at 20 Hz.
- Powdered board against anything: friction 0.04 (it combines by the minimum); published values for a powdered
  board are 0.02 to 0.05. A disc sliding on it slows by 0.39 m/s each second.
- Striker on queen: restitution 0.82; a disc rebounds from the frame at about 60% of its speed. Board surface:
  no bounce.
- Approximations:
  - the discs are 48-sided prisms with 0.8 mm bevelled edges;
  - the corner squares that carry the pockets (6 cm) and the strips between them are 0.2 mm lower than the
    middle of the board, so a sliding disc steps down onto them instead of catching on the seam between two
    collision shapes;
  - the powder is a uniform friction, not loose grains;
  - the nets are rigid boxes;
  - the board is perfectly flat;
  - there is no air drag;
  - the closed gripper fingers strike in place of a fingernail.

## Score

The trial scores **0** (a foul) if:
- the robot touches the queen;
- the robot is still touching the striker when the striker's centre is more than 1 cm off the base-line strip
  (a push, not a strike);
- the robot touches the striker again after the stroke, that is, once the striker, untouched, has moved more
  than 1 cm off the strip;
- the striker or the queen is ever more than 5 mm above the board with nothing touching it, or comes to lie on
  the frame or off the board;
- or the board moves, or a disc leaves the workspace.

The base-line strip is where the striker's centre can be while it touches both base lines and lies between the
centres of the base circles. Before the stroke the robot may adjust the striker, as long as its centre stays
within 1 cm of the strip.

Otherwise the score is set when both discs are untouched and still (under 5 mm/s for 1 s) after the stroke:

| Score | Outcome |
|---|---|
| 1.0 | the queen has dropped through a pocket and the striker has not |
| 0.5 | both have dropped (the striker followed the queen in) |
| 0.25 | the queen has moved at least 1 cm from its spot, neither has dropped |
| 0 | a miss, no stroke, or the striker dropped without the queen |

On a real board every condition can be checked with the board's markings and two videos: an overhead one for
where the discs end, and a 240 fps one beside the base lines, in which the striker moves about 3 mm per frame at
0.75 m/s, for whether a finger was still on the striker 1 cm past the strip.

## Reset and board speed

Between trials, take the queen out of the net and set both discs back on their pencil-marked spots; brush any
powder lumps off the board. Before each set of trials, check the board's speed so it matches the simulated
friction of 0.04:
1. Flick the striker across the board by hand and film it from above at 60 fps beside a tape measure.
2. Read its speed from its positions 0.5 s apart. It must slow by 0.35 to 0.43 m/s each second.
3. Dust on more carrom powder to make the board faster, or wipe some off to make it slower, and check again.

## Run

```bash
export ISAAC_PYTHON=/path/to/IsaacLab/.venv/bin/python INSPECT_OVERLAY=/path/to/inspect-overlay ARENA_ROOT=/path/to/IsaacLab-Arena
./run_inspect.sh --output runs/demo --policy demo --steps 400 --check-reset
./run_inspect.sh --output runs/hold --policy hold --steps 200
./run_inspect.sh --output runs/dream --policy dream --steps 400 --budget-ledger runs/dream-budget.jsonl
python audit_run.py runs/demo --require-full
PYTHONPATH=. python -m unittest -q test_contract test_dream_budget
```

One trial is one episode, seed 0, with no randomisation. A 400-step trial takes about 2 minutes on a single
GB10 GPU.

## Demonstration

`demo.py` is an RGB-only scripted rehearsal planned with offline IK. It never reads object poses.
1. The right hand closes its fingers and comes down behind the striker, fingers pointing down with their tips
   2 mm above the board, the closed pair facing along the line from the striker through the queen to the far left
   pocket (34.8° left of straight ahead).
2. At 5 s it strikes. It is commanded to 0.545 m/s over 5 cm and held at that speed; the arm overshoots the
   command and meets the striker 5 cm later with the outer face of its front finger. There is one contact
   (control step 106), and the striker leaves at 0.735 m/s, heading 34.3°.
3. The hand stops within 0.08 s and lifts away. The striker slides 0.63 m and hits the queen at 0.29 m/s (step
   130), slightly off-centre. The queen slides into the far left pocket and drops into the net (step 134); the
   striker stops on the board 5.5 cm short of the pocket.
4. The left arm stays parked beside the board.

The strike's speed and direction were rehearsed in this scene.

## Robot asset and licences

- The YAM arms (USD, kinematics and wrist cameras) come from the external adapter
  https://github.com/kairosiann/yam-isaac-lab-arena (revision 0059d02a1a91bc6834c1d0846022891df0c78730), which uses
  I2RT Robotics' YAM model (upstream commit 120c3c81400171174604e503943f8d1ebc891058). That asset is under the MIT
  License, Copyright (c) I2RT Robotics; it is not redistributed here, and its upstream `LICENSE.i2rt` must be kept
  with any copy. The arms are used unmodified. Pinned versions are in `DEPENDENCIES.json`.
- This task's own code is under the MIT License (`LICENSE`).

## Sources

- ICF laws of carrom (board, frame, pockets, base lines, circles and arrows; striker at most 4.13 cm and 15 g;
  carrom men 3.02–3.18 cm, 7–9 mm, 5.0–5.5 g; the striker is struck, not pushed; the stroke and the due):
  https://www.carrom.co.uk/laws-of-carrom/ and https://iakc.org/wp-content/uploads/2024/02/Carrom-Official-Rules.pdf
- Laws of carrom (Indian Carrom): https://www.indiancarrom.co.in/laws-of-carrom/
- Carrom's popularity and play: https://en.wikipedia.org/wiki/Carrom
- Powdered board friction 0.02–0.05: https://huggingface.co/blog/bpHigh/carrom-rl-env
- Striker-on-piece restitution 0.82 and frame rebound: https://fizzbloom.in/play/carrom/
- Board prices: https://upgrade.desiclik.com/product.php?productid=24177 (Surco Champion Speedo, $184.99) and
  https://www.amazon.com/dp/B079VNYPDJ (Precise Champion 16 mm, $249)
