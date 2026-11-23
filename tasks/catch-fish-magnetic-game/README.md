# Catch the red fish in a magnetic fishing game

Children's game: pick up the fishing rod, lower its dangling magnet onto the red fish's steel washer,
and lift the fish out of the pond without picking up any other fish. Built for the Robocurve YAM /
Isaac Lab-Arena stack.

## Why this task

Magnetic fishing games have been sold as children's and parlour games since about 1900 and are still a
standard preschool toy; most adults have played one, as children or with them. Catching a fish needs things that direct pick-and-place does not test:

- The robot controls the magnet only through a rod and a slack cord, so the magnet swings like a
  pendulum whenever the rod moves. The robot has to let the swing die down, or move so it doesn't
  start, before it can put the magnet on one particular 20 mm washer.
- The other fish lie 105 mm away, and the magnet pulls any washer within 25 mm. Land it off target
  and it may come up with the wrong fish, or two.
- The robot may touch only the rod: not the fish, the magnet or the cord. Holding the cord would
  take the swing out of the task.

## Geometry and physics

All values are in `spec.py` and `scene.py`. Origin on the floor midway between the two pedestals'
centres; x forward, y to the left, z up.

- Children's table: wooden, 600 x 800 mm top, 550 mm high, centred at x = 500 mm, y = -300 mm.
- Pond: a round wooden tray, 300 mm inside diameter, 8 mm walls rising 40 mm above its 6 mm floor
  (painted blue), centred at x = 560 mm, y = -230 mm.
- Five flat painted wooden fish, modelled on games such as Melissa & Doug Catch & Count: an
  80 x 40 x 12 mm body with a 25 x 32 mm tail (101 mm long overall), 23 g, and a 20 mm steel washer
  glued on the back over the balance point (9 mm behind the body's centre). They lie flat in the
  pond: the red fish in the centre, nose pointing 20 degrees left of straight ahead, and the blue,
  green, yellow and orange fish around it, 105 mm out (positions in `spec.FISH`), at least 12 mm
  from each other and 15 mm from the wall.
- Game box: closed, 200 x 120 x 60 mm, centred at x = 370 mm, y = -560 mm.
- Rod: a flat wooden stick, 280 x 15 x 8 mm, 20 g, its last 80 mm painted red (the handle). It lies
  flat along x on the box, handle end at x = 225 mm (45 mm past the box's back edge), tip at
  x = 505 mm. A 200 mm cord runs from the tip to a 16 mm x 8 mm disc magnet (6 g), which lies on the
  table at x = 555 mm, y = -560 mm.
- Magnet pull on each washer: 2 N at contact, falling as (1 + gap / 4 mm)^-3, none beyond 25 mm; it
  holds a fish with about 9 times its weight.
- Two YAM arms on 750 mm pedestals at y = +/-305 mm. Grippers: 75 mm aperture, 40 N per finger.
  Physics 240 Hz, control 20 Hz.

## Score ladder

- **0.25** the magnet touches the red fish's washer.
- **0.5** the red fish is lifted on the magnet with its lowest point at least 20 mm off the pond's
  floor.
- **1.0** the red fish hangs on the magnet with its lowest point at least 50 mm above the pond's rim,
  with no other fish on the magnet, for 20 consecutive control steps (1 s).
- The trial scores 0 for good if the robot touches any fish, the magnet or its cord, a fish falls off the
  table (more than 50 mm below its top), or anything leaves x -0.4 to 1.4 m, y -1.4 to 1.0 m,
  z 0 to 1.6 m.

Scores are latched except the invalidating conditions. In simulation a touch is a gap of 1 mm or
less (the cord has no collider; any robot collider within 1 mm of the drawn cord's surface, found by
sphere queries every 2 mm along it, touches it), and a fish is on the magnet when its washer is within 2 mm of the magnet's face; both are
checked at every physics step.

Physical judging: a side camera level with the pond's rim, with a scale behind it, shows the fish's
height; whether the magnet touched the red washer and whether another fish came up with it are
visible on the same video. Real trials are safe: the toy weighs under 200 g in all.

## External robot asset

The external YAM robot asset is MIT licensed (I2RT YAM via the pinned YAM Isaac Lab-Arena adapter).
It is not redistributed here. See `DEPENDENCIES.json` and retain upstream `LICENSE.i2rt`.

## Run

```bash
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy hold --steps 200 --check-reset --output runs/hold
bash run_inspect.sh --policy demo --steps 400 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

Dream (only after a full-score demo; shared GPU lock; ledger required):

```bash
bash run_inspect.sh --policy dream --steps 600 --output runs/dream \
  --budget-ledger /path/to/persistent/dream-budget.jsonl
```

Runtime per trial on one GPU, including scene start-up: about 1 minute for the demo (full score at
step 330, plus the reset check), under 1 minute for the hold check and about 2 minutes for a
600-step Dream-1 run. Seed 0, no object jitter. Demo observations are three RGB cameras only; Dream
also receives standard joint/gripper readings. No object poses or scorer state reach the policy.

The scripted demo pinches the rod's overhanging handle from above (fingers closing side to side),
lifts it level, carries the dangling magnet over the red fish's washer and holds still for 2 s while
the swing dies down, lowers the rod until the magnet rests on the washer, waits 1 s and lifts the
rod tip to 920 mm. `demo.py` replays it open loop.

## Sources

- History and familiarity: V&A, "The Popular Game of Magnetic Fishing", Chad Valley, about 1910,
  https://collections.vam.ac.uk/item/O35745/the-popular-game-of-magnetic-table-game-chad-valley-company/ ;
  J.W. Spear & Sons, Magnetic Fish Pond, about 1905 ("Versions of this game continue to be developed today"),
  https://ehive.com/collections/203194/objects/1651676 ;
  https://wetaskiwinmuseum.com/2019/12/07/the-history-of-the-magnetic-fish-pond-game/
- Current wooden sets with flat painted fish and washers: Melissa & Doug Catch & Count Fishing Game.
