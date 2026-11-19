# Counter-hold a loaded trailer hand winch crank while flipping its ratchet pawl

Instruction given to the policy: `Hold the red winch crank with one hand, flip the blue ratchet pawl into the magenta pocket with the other, then turn the crank to let the strap out until the black strap mark sits between the red tabs, without touching or coming within 5 mm of the orange sled or yellow strap, and finish with both hands clear of the winch.`

Files in this folder: `task.json`, `task.py` (mechkit winch articulation, success, partial credit, latched violations, open-loop demo), `demokit.py` (matrix TCP grips), `mechkit.py`, `arm_watch.py` (per-step finger/TCP read), `cad/` (BUILD.md plus STEP/STL/PDF), `LICENSE`, `README.md`.

## Offline check, no GPU

From the root of this repository, with the yam-arena package importable (on the program's Brev launchable, after `source ~/IsaacLab-Arena/yam.env`):

```bash
python tasks/trailer-hand-winch-pawl-flip/task.py
```

It asserts the scoring rules on hand-built states (20 consecutive clear-rest steps turn 0.75 into score 1.0 and success, every ladder rung, prior 0.50 latch before payout, payout-then-flip cheat, engaged UP refuse-payout couple, continuous counter-hold before pawl flip, runaway drum motion, sled cumulative tamper, open-pad strap halo, surface-extent clear-distance checks, every unintended strategy from the risk list, objects held in the air or moving, swapped or flipped parts, latch persistence, demo-coupled kinematic payout to success on both proxy states and MechState-only dicts, 20 Hz interpolated ArmPlan crank-hold straddle and URDF link-hull collision checks against winch colliders and blue paddle) and replays both arms' plans through the kinematics at 20 Hz, every joint inside the URDF limits with an IK error under 0.5 mm. A geometry check at the start pose, full payout and along the crank arc fails if any two rigid colliders that are not joined by a joint overlap by more than 0.5 mm, if a collider sits below the table, or if a joint anchor is outside its moving body or the fixed frame; the only allowed overlaps are the four joint-adjacent pairs (fixed frame with pinion, drum, pawl, sled), listed with reasons in `GEOM_ALLOWED`.

## Run and score one Dream-1 trial

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=tasks/trailer-hand-winch-pawl-flip/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the red winch crank with one hand, flip the blue ratchet pawl into the magenta pocket with the other, then turn the crank to let the strap out until the black strap mark sits between the red tabs, without touching or coming within 5 mm of the orange sled or yellow strap, and finish with both hands clear of the winch." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/winch-dream
```

Scoring is automatic (SCORE_OVER=end, SUCCESS_HOLD_S=0.05). `score()` counts 20 consecutive clear-rest steps (1.0 s) itself and returns 1.0 on the 20th; `success()` is true from then on, so the episode ends with score 1.0. Until then a clear frame after `L_payout` is 0.75 (`min(tier, end_cap)`). Instantaneous mid-episode 0.25 is debug-only; after that clamp, any result below 0.50 becomes 0.0 so timeouts never ship 0.25 (including after a brief magenta seat then pawl slip).

The CLI prints `task_score: <value>` and the log keeps it at `samples[0].epochs[0].task_score`. The drum angle conditions in `task.json` are read in a real trial from the crank angle (4.1:1) and the black stripe position; the two black spokes on the +y flange sit behind the cheek and are only a hand reset aid.

## Run and score the full-score demo

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/trailer-hand-winch-pawl-flip/task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Hold the red winch crank with one hand, flip the blue ratchet pawl into the magenta pocket with the other, then turn the crank to let the strap out until the black strap mark sits between the red tabs, without touching or coming within 5 mm of the orange sled or yellow strap, and finish with both hands clear of the winch." \
  --scorer task_score --max-steps 2400 --seed 0 --no-prompt --store-frames --log-dir ~/yam-eval/winch-demo
```

The demo is `demo()` in `task.py`, open loop and deterministic (no `demo_observations`, no scorer branching). Left arm closes 7 mm narrower than the 26 mm sleeve (19 mm command, jaws stop on the sleeve), grips 7 mm inboard of the sleeve centre (y = 0.098 m) to keep the forearm off the +y cheek, tracks the full `r = 0.178 m` crank arc from +40 to -82.3 deg with the TCP on the sleeve axis, then opens after a rest, lifts clear and withdraws; right arm approaches straight above `p_up` (2.0 s final approach) with a 32 mm partial opening 8 mm above the blue paddle centre, closes 7 mm narrower than the 12 mm paddle, flips it into the magenta mouth, and lifts straight up clear of the paddle before opening and retreating, then both withdraw and rest. Mechkit on_reset seeds the pretensioned sled (`x = 0.4550 m`) before the initial settle and frame 0; on_step gear/strap coupling moves drum, stripe, and sled with the crank; green UP engaged refuses payout-direction couple (wind-in still allowed). Pawl uses an over-center drive target (UP below 20 deg, magenta pocket past the ridge). Unbraced under load with pawl past the ratchet, the sled spring back-drives the 4.1:1 train (runaway latch). Red sleeve is fixed to the crank link (no free-spin bearing); sleeve and paddle use the default friction mu 1.0. While the crank is counter-held under load the pinion drive adds a 0.30 N*m preload toward payout (true Fulton value 0.53 N*m, reduced for the YAM wrist); at the chock the pinned joints report zero rate so the 1.0 s rest hold is not defeated by gravity droop. Closing counts as a hold only when the pad opening is 20 to 27.5 mm (no closing on air at 28 to 30 mm). Hands-clear keepouts and `< 5 mm` strap/sled clearance check full TCP and finger-pad/tip surfaces against all winch, crank, pawl, strap, stripe, and sled extents.

## Run limit, runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s of simulated time), `--max-steps 2400`.
- Runtime per trial on an RTX 4080 SUPER: Dream-1 about 7.6 min wall clock for the full 2,400-step trial (Dream round trips pause the simulator); the demo about 3.4 min for 725 steps; the first run after boot adds a few minutes of Isaac Sim warm-up.
- Seeds: `--seed 0` with `-E object_jitter=0` for the open-loop demo. Dream-1 trials are not bitwise reproducible.
- Demo grip pitch: yaw -90 with pitch blended about -45 to 0 deg on the left (straight +45 unreachable at start height); right pawl approach at pitch -45 with hover h=0.32 m.

## Submitted videos and scores

Task file SHA-256, first 12 hex: 8d27506fc7a0 (`sha256sum task.py | cut -c1-12`; equals `eval.environment_revision` in both logs). inspect-robots 1.0.0, Dream checkpoint `dreamlabs/dream-1-beta`.

- Videos: top camera above, left and right wrist cameras below, each camera at the 320 x 176 the policy sees.
- Dream-1 run: trial `scene-0-e0`, `task_score = 0.0`, ended by `max_steps` after 2400 steps. The left arm reaches to within about 6 cm of the red crank sleeve and hovers beside it, the right arm moves toward the winch, but neither gripper grasps the crank or the pawl; both arms then withdraw toward their bases with closed grippers.
- Demo: trial `scene-0-e0`, `task_score = 1.0`, ended by `success` after 725 steps.
- Setup screenshot: frame 0 of the top camera, from the demo command with `-E cam_width=640 -E cam_height=360 --max-steps 1`.

## Reset between trials

Wind the red crank CCW until the 0 deg spoke aligns with the red pointer, stripe at x = 0.3400 m, sled at x_sled = 0.4550 m; flip the blue pawl into green UP; verify the ratchet holds hands-free; inspect the strap wrap. Replace the black vinyl stripe if worn.
