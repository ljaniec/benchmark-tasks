# Refit a motor brush and restore its spring contact

Validated on the local DGX: Dream-1 0.0 in 1,600 steps; fixed-action demo 1.0 in 453 steps, with the required ten-step release dwell. Both Inspect evaluations completed successfully. This demonstrates feasibility in the fixed layout, not hardware validation or policy generalization.

This task covers mechanical refitting of a pre-bedded carbon brush after inspection and cleaning. A motor repair technician must place it in its guide and restore pressure against the commutator. This is a useful maintenance stage before reconnecting the electrical shunt, closing the cover and testing the motor. The installed holder, collector and surrounding motor structure are present. It does not claim to complete electrical repair or prove motor operation.

## Running and scoring

`DEPENDENCIES.json` pins the simulator, robot assets, Arena and Inspect versions. Set `ISAAC_PYTHON` to the installed IsaacLab environment's Python, `INSPECT_OVERLAY` to an isolated package directory, and `ARENA_ROOT` to an isolated Arena checkout. The tested backend uses native PhysX on CPU and RTX RGB rendering. Run these commands from this task folder:

```bash
git clone https://github.com/isaac-sim/IsaacLab-Arena "$ARENA_ROOT"
git -C "$ARENA_ROOT" checkout c7edf3a9d5d8276565208f72422f8f8a0806d6cf
uv pip install --python "$ISAAC_PYTHON" --target "$INSPECT_OVERLAY" --no-deps \
  inspect-robots==0.60.0 inspect-robots-yam==0.37.0 dreamlabs-client==1.0.14 \
  isaaclab-teleop==0.11.3 \
  'yam-arena @ git+https://github.com/kairosiann/yam-isaac-lab-arena@0059d02a1a91bc6834c1d0846022891df0c78730' \
  'inspect-robots-dream @ git+https://github.com/kairosiann/inspect-robots-dream@b5e936a201cec66c54c297e522c920d98445e339'
python prepare_arena_compat.py "$ARENA_ROOT"
PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" list embodiments
PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" list policies
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py'
bash run_inspect.sh --policy demo --steps 200 --output runs/demo-smoke
bash run_inspect.sh --policy demo --steps 1600 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

The overlay installation preserves the simulator's scientific dependencies with `--no-deps`. A compatible SciPy installation is needed for offline constrained IK. FFmpeg is needed for video export. The compatibility script changes only the isolated Arena checkout. Exact versions and revisions are recorded in `DEPENDENCIES.json`.

After local feasibility and camera checks pass, supply `DREAM_API_KEY` privately in the environment. Keep one persistent budget ledger across all paid attempts:

```bash
bash run_inspect.sh --policy dream --steps 200 --output runs/dream-smoke \
  --budget-ledger runs/dream-budget.jsonl
bash run_inspect.sh --policy dream --steps 1600 --output runs/dream \
  --budget-ledger runs/dream-budget.jsonl
PYTHONPATH="$INSPECT_OVERLAY" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" video \
  runs/dream/inspect/LOG.json --out runs/dream-video --fps 20
python export_media.py runs/dream/evidence/episode-00 runs/model-media --name model_run
python export_media.py runs/demo/evidence/episode-00 runs/demo-media --name demo
```

Replace `LOG.json` with the generated Inspect log. Every run enables frame storage, uses an exclusive local GPU lock, refuses an existing output directory and has a 1,200-second wall timeout. The trial limit is 1,600 control steps at 20 Hz, including model image-history warm-up. Physics runs at 480 Hz. Seed is 0 and object jitter is 0. The final demo took 22.65 simulated seconds and 125.0 seconds of episode wall time; the Dream trial took 80.00 simulated seconds and 479.8 seconds of episode wall time on the local GB10. End-to-end process times were 136.7 and 482.6 seconds, respectively, including startup/cleanup and the demo reset audit.

The Dream guard checks the published model price and conservatively reserves USD 0.02 per adapter attempt, covering one possible transport retry, up to USD 2.00 cumulatively per ledger. Reservations are upper bounds, not invoices; include other account spending when setting a budget. Never replace a ledger to restart its allowance.

## Mechanism and setup

All dimensions use metres unless stated otherwise. Coordinates: x forward from the arm-base line, y left of its midpoint, z up. The table surface is z=0.7475; YAM bases are at (0, ±0.305, 0.75). Maximum aperture is 75 mm and each finger is capped at 40 N, with a native equal-travel mimic constraint representing the physical rack. `spec.py` and `scene.py` define all procedural geometry. No external task mesh is required.

The motor is isolated and bolted to a stand; its service cover is already removed. A 210 mm diameter, 145 mm long rear motor body, two end-shield cheeks, a bearing boss and the mounting foot surround the brush gear. The stationary copper commutator is 120 mm diameter and 130 mm long, axis y, top at (0.32, -0.035, 0.9125). The brass box sits 2.5 mm above it, with a 25.2 x 32.2 mm opening, 3 mm walls and 30 mm height. These dimensions preserve real brush-box clearance, rather than an enlarged insertion slot.

The graphite brush is nominally 25 x 32 x 50 mm, 75 g. Its curved underside is pre-bedded to the collector radius, represented by ten convex strips. It starts upright beside the motor at (0.47, -0.12, 0.74882), with x tangential, y axial and z radial. Bedding, cleaning and shunt disconnection precede this task. The flexible electrical lead is omitted because it is not manipulated in this mechanical stage; hardware trials must route it freely outside the grip and box path before starting.

A 35 g steel pressure finger pivots about y at (0.26, -0.035, 0.9925). It has a 60 mm lever, 14 mm width and a 30 mm downturned toe. Its native revolute joint is limited to -85..22 degrees; initial angle 16.09 degrees down, resting against the empty holder. A passive torsion spring uses 0.70 N m/rad stiffness, 0.020 N m s/rad damping and an 80-degree unloaded target. Around the installed position its nominal torque is approximately 1 N m, giving about 16 N pressure at a 60 mm lever arm. This is approximately 2 N/cm² over the 8 cm² brush face, within the manufacturer's typical range. The USD drive stores stiffness and damping per degree, so the SI values are multiplied by pi/180 when authored. Native contact, not a controller event, stops it on the brush. No state-triggered attachment, latch, snap or object pose command exists.

The hinge post joins the holder through a 60 x 22 x 14 mm spring bracket; its bearing axle is represented by the native joint and visual cylinder, avoiding duplicate internal axle collision. The housing, holder dimensions, mass, pressure-finger proportions and spring rate are representative estimates, not manufacturer CAD. The task preserves the conventional brush-box/clock-spring mechanism. It omits powered operation, arcing, wear, flexible shunt motion and electrical connection. Hardware calibration has not been performed. Native PhysX TGS runs at 480 Hz; workpiece friction is 0.25, zero restitution. Gripper-pad static/dynamic friction is 1.2/1.0 with native average combination and a 2 mm torsional patch radius. Friction is an estimate, not a fitted result.

## Observations and demonstration

`overhead_cam`, `left_wrist_cam`, and `right_wrist_cam` correspond to Inspect's `top_cam`, `left_cam`, and `right_cam`. Native RGB is 640 x 360, bilinearly resized to 320 x 176 at 20 Hz; the official Dream adapter uses JPEG quality 85. Dream also receives standard robot joint/gripper readings. Object transforms, physics queries and scoring state are evaluator-only.

The demonstration accepts the three RGB frames and executes a documented fixed offline action sequence for this zero-jitter layout. It uses no live object-state feedback and is not a general visual-servo policy. Constrained offline inverse kinematics checks every 0.25-second Cartesian waypoint against hardware limits. The brush uses a side approach above the end-shield wall. The other arm grips the pressure lever and follows its hinge arc. Offline joint-target biases based on the specified spring torque, brush weight, robot gains and planned joint velocities compensate controller deflection and damping lag. They are computed before execution, with no measured object state; hardware joint and force limits remain enforced. The largest clipped velocity bias is recorded in the demo transcript. Fixed subdegree approach and submillimetre placement biases come from rehearsal of this one published layout and remain constant during execution. The complete native and policy-delivered frame sequences are stored for audit.

The initial overhead view establishes the motor layout but the motor hides the separate brush; the right wrist sees the brush at pickup. The wrist views expose the lever, guide and final pressure-finger placement as the arms move, with temporary occlusion during contact. Native frames and actual resized policy frames, including representative JPEG-quality-85 samples, were inspected. A submillimetre seating gap is judged with the stated physical feeler check, not inferred from a low-resolution image alone.

## Physical judging and score

Use a ruler, 0.5 mm feeler gauge, protractor and continuous video. The open box-to-collector gap permits a feeler check of the brush face from the side. The result is the highest current valid tier; earlier successful seating does not survive removal.

| Score | Physical condition |
|---|---|
| 0.0 | No valid insertion, or any permanent invalid event below. |
| 0.25 | Brush inside the guide, at least 10 mm inserted from the top rim, tilt below 4 degrees and centre less than 1 mm of the box centre. |
| 0.5 | Also seated against the commutator: at most 0.5 mm gap at both axial ends and across the curved face; tilt below 1 degree. |
| 1.0 | Also pressure-finger toe fully over the brush top and resting on it, both hands clear of brush and pressure finger, continuously for 10 control steps (0.5 s). |

Finger placement is checked in the brush's coordinates: its entire 13 x 16 mm toe footprint lies within the 25 x 32 mm top, lower edge gap at most 0.5 mm and toe tilt below 5 degrees. The known passive spring supplies pressure. On physical hardware, confirm that the finger is free, spring-loaded and moves back toward the brush when lifted before the trial; a photo alone must not validate a broken or disconnected spring.

Release uses a 35 x 42 x 63 mm inspection envelope centred 25 mm above the brush's radial origin and a 76 x 26 x 44 mm envelope centred at (34,0,-15) mm in the pressure finger frame. No robot collision geometry may enter either envelope. These correspond to at least approximately 5 mm clearance around the relevant component; the stated envelopes are authoritative at corners. Physical judging can use an overlaid dimensioned template or ruler and close video.

Permanently zero for the brush leaving x=[-0.19,0.91], y=[-0.65,0.65], z=[0.7375,1.30] (whole-object bounds); the housing tipping more than 15 degrees; a fixed table, housing or holder moving more than 2 mm; a pressure-finger pivot separating more than 1 mm, a finger below -86 or above 24 degrees (solver tolerance around its -85..22 degree stops), or another destructive shortcut. Simulated penetrations greater than 0.5 mm cannot earn seating or pressure-contact credit. These are evidence-validity limits, not extra skills.

Reset reconstructs the scene, native joints, physics views and cameras; restores authored body poses, robot root/joint states and zero velocities; clears score, dwell and invalid history; and resets all camera timestamps/buffers. The local reset audit compares initial transforms and camera hashes and runs a 20-step zero-score hold. `scorer.py` replays grades; `audit_run.py` checks source snapshots, native gripper limits, frame completeness and correspondence, reset velocities and grades.

## Recorded validation

The ten CPU tests passed. Both final runs passed complete frame-sequence, source-snapshot, native 40 N finger-cap, 75 mm aperture, coupled-jaw and score-replay checks. An independent reconstruction from exported geometry and raw transforms agreed on 10,265 geometric checks across both runs. The completed-demo reset restored poses, velocities, joints, counters, camera timestamps and initial image hashes; a subsequent 20-step hold scored zero. `local-validation.json` records measured results and evaluated source hashes.

The Dream trial completed its allotted steps with score 0.0; this result is reported without attributing a cause. The official adapter handled retryable session-expiry responses, and every request attempt was included in the persistent budget ledger. The smoke and full model trials together reserved at most USD 1.44; that is a conservative reservation, not an invoice. No cloud compute was started.

## Sources and distinctness

[Morgan's carbon-brush handbook](https://www.morganelectricalmaterials.com/media/ry5jzzqg/technicalhandbookglobalproof_0.pdf) documents fitting, bedding, free brush movement, brush-holder clearance and spring pressure. [Mersen's dimensional guide](https://us.mersen.com/sites/default/files/files_imported/2017-ptt-tds-04-t-a-r-dimensions-mersen.pdf) lists standard brush dimensions and guide tolerances. [Helwig's brush-holder range](https://helwigcarbon.com/brush-holders/) shows the conventional clock spring and pivoting pressure finger, and its use in motor maintenance.

A board search on 20 November 2026 covered 3,876 unique task IDs and found no direct motor-brush operation. The nearest spring phone-holder task clamps an object laterally; the rail-carriage task traverses a detent; the M.2 task holds a sprung board under a fastener; the wiper task changes a hook-mounted blade. This task restores a free sliding electrical contact in a close guide under its pressure finger. Shared elementary spring-handling skills do not establish identical work products. The common benchmark catalogs were also screened. Reviewers determine final distinctness and importance.

Original code is MIT licensed. Robot and simulator dependencies retain upstream licenses and are not redistributed.
