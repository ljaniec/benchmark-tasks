# Restore a spindle collet's retaining fit

The fixed-layout RGB-only demo reaches **1.0 in 863 control steps (43.15 simulated seconds)**. Dream-1 (`dreamlabs/dream-1-beta`) scores **0.0 after 1,600 steps (80 simulated seconds)**. Both recordings use seed 0 and zero object jitter. Eleven CPU tests pass.

This is service of a detached ER collet and retaining nut before reinstallation in a machining spindle. Restoring the eccentric lip's engagement with the collet groove lets the nut extract the collet from the spindle taper during subsequent tool changes. The same interface is used in CNC and robot-mounted machining spindles. The task starts with the removed components, as required by the assembly procedure; it does not claim to reproduce access inside an installed spindle.

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

Replace `LOG.json` with the generated Inspect log. Every run enables frame storage, uses an exclusive local GPU lock, refuses an existing output directory and has a 1,200-second wall timeout. The trial limit is 1,600 control steps at 20 Hz, including model image-history warm-up. Physics runs at 480 Hz. Seed is 0 and object jitter is 0. The selected demo took 244.85 wall seconds for the episode, or 256.79 seconds including process startup and the full reset audit. The full Dream episode took 493.24 wall seconds, or 497.27 seconds including startup. A 200-step Dream smoke test took 77.94 wall seconds.

The Dream guard checks the published model price and conservatively reserves USD 0.02 per adapter attempt, covering one possible transport retry, up to USD 2.00 cumulatively per ledger. Reservations are upper bounds, not invoices; include other account spending when setting a budget. Never replace a ledger to restart its allowance.

Runs preserve source snapshots and hashes, Inspect actions and delivered RGB frames, native RGB frames, evaluator traces, reset state and exit receipts. `scorer.py` checks recorded scores against the stored evidence; `audit_run.py` also verifies frame completeness, resizing, source hashes and reset velocities. Independent CPU reconstruction from exported mesh bounds, raw actor transforms and native overlap-hit lists also agreed on all 6,904 demo and 12,800 model geometry checks. `local-validation.json` records the results. These consistency checks complement physical judging rather than replacing it. Exported review media combines the three native views without identifying overlays; official Inspect videos use policy-delivered frames.

## Setup and mechanics

Coordinates are metres: x away from the robot bases, y left, z up. Tabletop height is 0.7475 m. The two YAM bases are at (0, ±0.305, 0.75). Maximum gripper aperture is 75 mm and finger force is capped at 40 N. A native mimic constraint couples equal finger travel, representing the single-motor rack mechanism; both finger joints retain their limits and force caps. All initial velocities are zero. `spec.py`, `scene.py` and the initial offline IK waypoint define the complete reset.

| Object | Geometry, material and starting placement |
|---|---|
| Workbench | Fixed wood table, 1,100 × 1,300 × 60 mm; centre (0.36, 0, 0.7175). |
| Retaining nut | Blackened steel, estimated 50 mm across flats × 23 mm axial height, 160 g. Small-opening-face origin (0.27, 0.16, 0.77055); local axis points −z and the wide-opening face rests on the table. Clear main bore radius 20 mm; eccentric lip centred at local z=10.3 mm, thickness 1.2 mm, opening radius 16.2 mm offset +1 mm along local x. Lower opening radius 13.5 mm for the first 3 mm; conical seat opens to radius 16.675 mm at z=8.5 mm. |
| Collet | Silver spring steel, ER32 envelope: maximum diameter 33 mm, length 40 mm, bore 19 mm, mass 100 g. Inverted nose origin (0.30, −0.02, 0.86755), axis −z; the narrow tail rests on the support block. Nose length 5.5 mm with 30° taper; groove diameter 29.2 mm × 3.6 mm axial length; upper body diameter 32 mm at z=9.1 mm, tapering inward by 8° to its top. Eight longitudinal slot gaps of 1.5° each. |
| Return pad | Fixed green rubber rectangle, 90 × 90 × 2 mm; centre (0.36, 0.23, 0.7485). |
| Support block | Fixed steel base, 40 × 40 × 80 mm; centre (0.30, −0.02, 0.7875), floor at 0.8275 m. A 10 mm-deep open locating cup surrounds the collet tail: inner radius 11.91 mm at the floor, increasing at 8° to 13.31 mm; outer radius 20 mm, rim at 0.8375 m. Radial clearance is 0.25 mm. It carries lateral assembly loads and has no latch or attachment. |

The collet envelope follows a published ER32 drawing. Nut internals, mass distribution and clearances are representative estimates, not manufacturer CAD. The nut is one dynamic body with separate convex annular wedges, preserving its open bore and asymmetric lip. Its unused screw threads are omitted because this task ends before spindle reinstallation.

The collet's upper tapered body is rigid. Eight separate nose/groove sectors each have a native radial prismatic constraint, inward travel limited to 0.55 mm and outward travel to 0.05 mm. Each spring uses 10,000 N/m stiffness and 10 N s/m damping, with a zero-displacement rest position. This is a reduced model of small radial compliance, not full spring-steel deformation or tool clamping. The model has no proximity-triggered snap, attachment, captured pose or scripted object motion.

Workpiece friction is 0.20 with zero restitution. Finger pads retain rubber-like static/dynamic values of 1.2/1.0, combined with contacting materials by the native average rule, and a 2 mm torsional contact-patch radius. These are estimates, not hardware calibration. The native TGS solver runs with zero velocity iterations. All grasping, insertion, retention and placement must result from robot contact and dynamics.

## Observations and demo

The three permitted views are `overhead_cam`, `left_wrist_cam` and `right_wrist_cam`, named `top_cam`, `left_cam` and `right_cam` by Inspect. Cameras render 640 × 360 RGB and deliver bilinearly resized 320 × 176 frames at 20 Hz; the official Dream adapter encodes these as JPEG at quality 85. Dream also receives standard robot joint/gripper readings. Object poses, contact queries and scoring state are confined to the evaluator.

The scripted demo accepts RGB only and follows a fixed offline IK rehearsal for the published zero-jitter layout. Cartesian waypoints are sampled every 0.25 s to keep approach paths clear of the support. It does not query live object state, and is not a general visual-servo policy. Offline IK residuals must be below 3 mm and 0.02 rad at every waypoint. The moving hand grasps the nut’s hexagonal exterior while the other initially stabilizes the supported collet, then withdraws before pressing. Grasp depths and positions are selected offline; they are never refreshed from live object poses. Camera frames and input shapes are recorded for audit. During the retention check, the unused wrist follows a precomputed inspection path to keep the exposed collet and its separation from the support in view. The overhead view establishes layout; wrist views show the collet before entry, its exposed tail during retention, and placement on the pad. The active gripper partially occludes the nut during insertion, as it does physically, so retention is judged from the released collet staying with the nut. Native frames, delivered frames and official-adapter JPEG samples were inspected for contrast, clipping and z-fighting. The small parts are most legible in wrist views; overhead alone does not resolve the groove.

## Physical judging

Use video, a ruler or calipers, a protractor and a timed close view. The nut axis points from its small-opening face toward its wide opening. Visible collet protrusion beyond the wide-opening rim and axis alignment give the fit measurements, regardless of assembly orientation. No force sensor or simulator state is needed.

- **0.25:** the collet has entered the nut: nose plane lies between −5 and +20 mm along the nut axis from its small-opening face, axis offset at most 6 mm and axis angle at most 35°.
- **0.5:** it is seated and released: axis offset at most 1.5 mm, axis angle at most 5°, and exposed collet length beyond the wide-opening rim is 18.5–21.8 mm. No robot part intersects the collet inspection envelope, a 43 × 43 × 50 mm box centred 20 mm along its axis from the nose root.
- **1.0:** first maintain the seated, released collet for 10 consecutive steps while the whole assembly is at least 50 mm above the support block top (140 mm above the table) and the nut axis is tilted at least 110° from upright. Then put the whole assembly inside the return pad's 90 × 90 mm boundary, lowest point within 3 mm of its top, with both parts released for 10 consecutive steps. Nut clearance uses a 68 × 68 × 33 mm inspection envelope centred halfway along its axis. Ten steps at 20 Hz is 0.5 s.
- **0.0:** no current valid stage, or permanently after any dynamic component leaves the workspace, a collet nose sector separates by at least 2 mm from its body, a fixed table, pad or support block moves by more than 2 mm, or the workbench tilts more than 15°. Invalid shortcuts cannot retain partial credit.

Workspace bounds are x=[−0.19,0.91], y=[−0.65,0.65], z=[0.7375,1.25] m, checked against whole-object bounds. Unseating or grasping the collet again clears the retention proof. Interrupted proof or final dwell must restart. The collet cannot earn full credit by merely resting loose in an upright nut: the independent tipped handling check must have been observed. Judge the terminal state, preserving only this required history.

Reset rebuilds the stage, constraints, physics views and cameras; restores all authored rigid poses, robot joint targets and zero velocities; and clears proof, dwell, invalid-event and camera state. The reset check compares initial prop/robot states and camera hashes, checks timestamps and counters, and executes a 20-step zero-score hold.

## Sources and distinction

[Carbide3D's ER collet guide](https://carbide3d.com/hub/docs/er-collet-basics/) explains the eccentric lip, extraction function and angled assembly before spindle or tool installation. The [DARMET ER32 drawing and instructions](https://www.e-darmet.com/en_US/p/ER32-collet-19-mm-DARMET/1371) provide collet dimensions and the assembly procedure. [Teknomotor's robotics applications](https://www.teknomotor.com/en/applications/robotics) list robot-mounted spindle families with ER32 toolholding.

Toolholding installation and maintenance are documented duties of [machinists](https://www.onetonline.org/link/details/51-4041.00) and [CNC tool operators](https://www.onetonline.org/link/details/51-9161.00). The manufacturer’s [ER assembly instructions](https://us.rego-fix.com/en/products/collets/standard?system=ER) describe the same groove-and-eccentric-ring engagement. The task addresses this service step across machining equipment, including robot-mounted spindles.

The useful outcome is restored collet-to-nut extraction retention. A circlip task expands a separate ring with pliers over a shaft; a keyed chuck task clamps a tool by tightening; loading a toolholder uses a different spindle retention interface. A plumbing push-fit release collet disengages a pipe-gripping ring; it does not assemble a spindle extraction interface. This operation instead requires engaging a circumferential collet groove with an eccentric internal lip and preserving retention after release. The board screen checked 3,855 unique task IDs on 19 November 2026, including accepted, pending and revision entries, without finding this operation. Related public benchmark catalogs were also screened. Screening cannot guarantee reviewer acceptance.

Original code uses the accompanying MIT license. The external **YAM robot USD and mesh assets** are supplied by [yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena/tree/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam), revision `0059d02a1a91bc6834c1d0846022891df0c78730`. These robot assets are **MIT licensed**; retain their upstream [LICENSE.i2rt](https://github.com/kairosiann/yam-isaac-lab-arena/blob/0059d02a1a91bc6834c1d0846022891df0c78730/yam_arena/assets/yam/LICENSE.i2rt). The assets are loaded externally and are not redistributed in this task. Simulator dependencies retain their respective upstream terms.
