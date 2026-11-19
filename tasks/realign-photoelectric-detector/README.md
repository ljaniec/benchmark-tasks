# Realign and secure a robot conveyor's photoelectric detector

Local demonstration: **1.0 in 1,096 control steps (54.8 simulated seconds)**, with ten final dwell steps. The recorded trace, source hashes and all camera frames passed audit; the full reset regression and 12 CPU tests passed. Dream-1 scored **0.0 in 2,000 steps (100 simulated seconds)**, with no invalid-state event; the Inspect evaluation completed successfully.

This task represents mechanical commissioning of a retroreflective object-presence detector in a robot conveyor assembly. The useful result is an optical setting that survives fastening and release, verified by an actual blocked/clear output cycle. The conveyor remains stationary throughout. Electrical connection, sensitivity setup and machine isolation are preconditions.

## Running and scoring

`DEPENDENCIES.json` pins the tested simulator, robot, Arena and Inspect versions. Install Isaac Sim 6.2.0.0 and IsaacLab 47.1.0 with Python 3.12.13. Set `ISAAC_PYTHON` to that environment's Python, `INSPECT_OVERLAY` to an isolated package directory, and `ARENA_ROOT` to an isolated Arena checkout. The tested backend is native PhysX on CPU with RTX RGB rendering. Robot assets remain external under their upstream licenses.

Run installation and trial commands from this task folder:

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
"$ISAAC_PYTHON" -m unittest discover -s . -p 'test_*.py' -v
bash run_inspect.sh --policy hold --steps 200 --output runs/hold
bash run_inspect.sh --policy demo --steps 200 --output runs/demo-smoke
bash run_inspect.sh --policy demo --steps 1200 --check-reset --output runs/demo
python scorer.py runs/demo/evidence/episode-00/trace.jsonl
"$ISAAC_PYTHON" audit_run.py runs/demo --require-full
```

These overlay commands assume the installed simulator supplies its scientific and rendering dependencies; `--no-deps` preserves them. Supporting versions include NumPy 2.5.1, Pillow 12.3.0, websockets 14.2 and Torch 2.11.0+cu130. FFmpeg is required for video export. The exact Arena Git revision is authoritative; its source metadata declares version 0.3.0. The compatibility script changes only the isolated Arena checkout.

After the local feasibility and observation checks pass, supply `DREAM_API_KEY` privately in the environment and retain one budget ledger across all paid attempts:

```bash
bash run_inspect.sh --policy dream --steps 200 --output runs/dream-smoke \
  --budget-ledger runs/dream-budget.jsonl
bash run_inspect.sh --policy dream --steps 2000 --output runs/dream \
  --budget-ledger runs/dream-budget.jsonl
PYTHONPATH="$INSPECT_OVERLAY" "$ISAAC_PYTHON" "$INSPECT_OVERLAY/bin/inspect-robots" video \
  runs/dream/inspect/LOG.json --out runs/dream-video --fps 20
python export_media.py runs/dream/evidence/episode-00 runs/model-media --name model_run
python export_media.py runs/demo/evidence/episode-00 runs/demo-media --name demo
```

Replace `LOG.json` with the generated Inspect log. The launcher uses `store_frames=True`, takes an exclusive local GPU lock, refuses existing output directories and imposes a 1,200-second wall timeout. The task limit is 2,000 control steps at 20 Hz, including the Dream adapter's initial image-history warm-up. Physics runs at 480 Hz. Evaluation seed is 0 and object jitter is 0. A 200-step local smoke run takes about one minute; a 1,200-step scripted trial with reset takes about 5 minutes on the tested installation. The measured full model trial took 610.8 wall seconds; the successful demo plus reset took 302.5 wall seconds. Service latency adds time to model trials.

The persistent spending guard checks the current published model price and reserves USD 0.02 per adapter attempt, including a possible transport retry, with a USD 2.00 cumulative cap across invocations using the ledger. Reservations are upper bounds, not invoices. Account-wide spending outside this ledger must also be counted. Never replace the ledger to restart its allowance.

Each run preserves the executable source snapshot and hashes, Inspect actions and policy frames, native camera frames, evaluator evidence, reset state and process result. `scorer.py` replays every recorded grade and dwell count; that checks telemetry consistency and is not independent physical judging. `audit_run.py` additionally checks frame completeness, native-to-policy image resizing, source hashes and reset velocities. The exported review video stacks the three native views without identifying overlays. Official Inspect videos use the delivered policy frames. All MP4s use H.264, yuv420p and faststart, below 50 MB; `setup.png` is the first overhead frame, below 10 MB.

## Reconstructing the setup

Coordinates are metres, x away from the robot bases, y to the left and z upward. Tabletop height is 0.7475 m. Robot bases are at (0, ±0.305, 0.75). All initial velocities are zero. `spec.py`, `scene.py` and the first IK waypoint specify the complete reset layout.

| Object | Geometry, material and initial placement |
|---|---|
| Table | Fixed wood surface, 1,100 × 1,300 × 60 mm, centre (0.36, 0, 0.7175). |
| Conveyor section | Fixed metal frame 560 × 360 × 45 mm, centre (0.620, 0.09, 0.7700); dark belt 510 × 340 × 3 mm, centre (0.635, 0.09, 0.7940). Travel is along y; the beam crosses its width along x. |
| Detector pedestal | Fixed metal block 65 × 45 × 59 mm, centre (0.33, 0, 0.7770). |
| Sensor and bracket | Blue metal sensor envelope 42 mm deep × 15.6 mm wide × 48.5 mm high. A steel foot is 30 × 23.5 × 2 mm; upright is 2 × 19 × 46.5 mm. Combined mass 140 g. Pivot (0.330, 0, 0.8075), initial yaw −14°. Sensor centre relative to pivot (0.031, 0, 0.02525). The received-light indicator is a yellow 5 mm square on top. |
| Clamp | M5 × 0.8 thread, 8 mm hex head, backed off one quarter-turn at reset. Axis (0.312, 0, 0.8075), 18 mm behind the pivot. Fixed captive washer 10 mm diameter × 1 mm thick; head 3.5 mm high, 4 g. A black radial witness line marks the head angle. The scoring threshold is 95°, five degrees beyond initial contact. |
| Reflector | Fixed red-backed reflector 4 × 18 × 60 mm, centre (0.900, 0, 0.83275), facing −x. Fixed post 12 × 18 × 59 mm at (0.904, 0, 0.7770). Nominal aligned optical distance is 517.5 mm. |
| Socket driver | Free 100 g tool: black rectangular handle 20 × 20 × 60 mm, steel shaft 6 mm diameter × 65 mm, 8.1 mm internal hex socket 12 mm deep, with a 0.3 mm radial entry bevel over the first 1 mm. Socket origin (0.23, −0.15, 0.75755), tool axis initially +x; handle centre 110 mm along that axis. |
| Test block and pad | Free opaque orange plastic block 40 × 40 × 60 mm, 45 g, centre (0.38, −0.18, 0.7785). Fixed green pad 75 × 75 × 1 mm, centre (0.38, −0.18, 0.7480). |

The sensor is rigidly attached to the upright. A revolute constraint represents the pivot and ±30° arcuate slot; the slot, mounting-hole details and electrical lead geometry are omitted. This is representative procedural geometry, not manufacturer CAD. The real sensor envelope and compatible bracket are documented below; bracket thickness and detailed socket construction are estimates. The rectangular grip is a simple replaceable flat-sided handle. Commercial [nutdrivers with shaped grips and anti-roll features](https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05029464001.pdf) establish the tool type; the modeled dimensions and rectangular grip are not that product’s CAD.

The screw uses a reduced thread-preload law: physical hex-head rotation corresponds to 0.8 mm advance per turn. Beyond the initial 0.2 mm gap, estimated preload is 10 MN/m times additional advance, capped at 600 N. A native zero-velocity rotational drive resists motion with a torque cap of preload × 0.25 × 18 mm. Its stiffness is zero: it has no target or captured angle, and torque above its cap can make the mount slip. Its damping regularizes near-zero velocity at 1,000 N m s/rad. The captive washer is fixed; axial washer motion, thread geometry and contact-pressure distribution are omitted. Its zero-friction collider avoids counting that interface twice. Bolt resistance is 0.002 N m plus preload times a 0.20 nut factor and 5 mm diameter. These are explicit mechanical approximations, not calibrated OEM torque or fast-transient models. Rigid workpiece contacts use an estimated coefficient of friction 0.25 and zero restitution. Finger pads use the upstream YAM rubber-like estimates of 1.2 static and 1.0 dynamic friction; PhysX combines the two contacting materials by their average. A native 2 mm torsional contact-patch radius approximates finite rubber-pad contact; it does not attach the tool to the fingers. The TGS solver runs with zero velocity iterations to avoid its documented interaction with stiff joint drives. These coefficients are not measurements of a particular physical installation. An external passive torque represents cable bending with 0.05 N m/rad stiffness and 0.001 N m s/rad damping, capped at 0.03 N m. The lead itself is not manipulated.

The optical model samples nine rays across a narrow cone, using the documented 25 mm spot at 1.5 m as its nominal divergence. It is an approximate geometric return model, not the full manufacturer's optical transfer function. Seven returned rays turn on the physical receive indicator. Native scene queries test occlusion by all colliding objects, including the robot. Reflector incidence is limited to 30°. Transparent-object attenuation, reflectivity variation, teach-in electronics and electrical faults are omitted.

The YAM arms retain native joint limits and effort limits, with a 75 mm maximum aperture and 40 N per finger. All tool, block and mount motion during execution comes from native dynamics and robot contact. No object is attached to a gripper or repositioned to complete a stage.

## Observations and demonstration

Policies receive only `overhead_cam`, `left_wrist_cam` and `right_wrist_cam`, through Inspect names `top_cam`, `left_cam` and `right_cam`. Calibrated cameras render 640 × 360 RGB; actual policy frames are bilinearly resized to 320 × 176 at 20 Hz. Dream also receives standard robot joint/gripper readings. Object poses, contacts, optical-ray diagnostics and score remain evaluator-only data.

The scripted demonstration accepts RGB only and uses a fixed offline IK rehearsal for this zero-jitter layout. It contains a fixed tool-to-gripper calibration measured during development; it does not query object poses during execution and is not a general visual servo policy. Yellow-pixel observations are recorded for audit, not used as hidden phase feedback. Every IK waypoint must meet a 3 mm position and 0.02 rad orientation residual gate before execution.

The left wrist resolves the indicator during alignment and remains at an inspection pose after release. This view shows the sensor, clamp and LED during the test-block cycle; the right wrist follows the tool and block, and the overhead view shows the whole arrangement. The socket naturally covers the hex head while seated. The actual delivered frames and their official JPEG-quality-85 encoding were inspected at setup, grasp, insertion, turning, release, beam blockage, return and completion. The yellow indicator remains visible through alignment and the blocked/clear cycle; no z-fighting was observed. The head witness is too small for a precise 95° reading at policy resolution. Physical judging therefore uses a close before/after photograph and protractor; an unresolved mark alone does not establish the clamp condition. This fixed-layout demo is not evidence of a general visual-servo solution.

## Physical judging and score

Use the three videos, a ruler, 5 mm gap gauge, protractor and timed close view. Start with the head witness mark at its published zero orientation and the clamp one quarter-turn loose. Judge the current terminal state, preserving only the required test history. No force sensor or simulator telemetry is needed for physical judging.

- **0.25:** the detector's received-light indicator is on.
- **0.5:** additionally, the head witness has turned at least 95° clockwise from reset, all robot parts are at least 5 mm from the detector/bracket, and the tool is at least 5 mm clear. The setting must receive light without being held.
- **1.0:** at that released, fastened setting, observe reception for 0.5 s, then hold the opaque test block in the beam so the indicator is off for 0.5 s. Remove and return the block wholly inside its green pad, with its lowest point within 3 mm of the pad top. Release the block with 5 mm robot clearance. Reception and all terminal conditions must then hold for ten consecutive 20 Hz steps, or 0.5 s.
- **0.0:** no current valid stage, or permanently after frame tilt exceeds 15°, a dynamic object leaves the workspace, the bracket separates from its pivot by 2 mm, or a fixed frame/reflector moves more than 2 mm. Moving or damaging the fixtures cannot earn partial credit.

Workspace bounds are x=[−0.19,0.91], y=[−0.65,0.65], z=[0.7375,1.25] m, checked against whole-object bounds. Use table-edge reference marks and a ruler in a physical reconstruction. Re-gripping the detector, touching it with the tool, loosening the clamp, or changing the bracket setting by more than 1° clears the earlier blocked/clear proof. Blocking with a hand or tool is not an object-detection test. During the legitimate blocked interval the current score is zero; earlier alignment alone is not a terminal success. The final dwell is consecutive and loss of a terminal condition removes full credit.

The evaluator measures the same conditions using native geometry queries, bolt orientation, optical output and state history. Internal force estimates are diagnostics only. A valid physical judge must be able to resolve the witness, LED state, clearance and block return from the views or ordinary static measurements. This judging procedure has not been tested on physical hardware.

Reset rebuilds the stage, constraints, physics views and cameras; restores authored prop and robot poses, joint targets and zero velocities; resets the loose clamp; and clears all scorer and camera buffers. The regression compares prop/robot reset states and initial camera hashes, verifies cleared counters/timestamps, and runs a 20-step hold with score zero and a loose clamp.

## Sources and distinctness

The [SICK operating instructions](https://www.sick.com/media/docs/4/34/634/operating_instructions_wl12g_3_en_de_fr_pt_it_es_ja_zh_im0035634.pdf) describe alignment using received-light indication and the blocked/clear test. The [WL12G-3B2531T01 datasheet](https://www.sick.com/media/pdf/9/89/389/dataSheet_WL12G-3B2531T01_1041458_en.pdf) supplies the envelope, mass, nominal beam spot and compatible bracket listing. The [BEF-WG-W12 drawing](https://cdn.sick.com/media/pdf/2/32/432/dataSheet_BEF-WG-W12_2013942_en.pdf) supplies the stainless bracket envelope and slotted-base reference.

A [documented robot-unloading application](https://www.bannerengineering.com/br/pt/applications/other-applications/positioning-pallets-for-unloading-by-robot-arms.html) uses retroreflective detectors along conveyor rails to trigger robot unloading. It supports the robot-cell integration context, not a claim that this application uses the modeled sensor.

The distinctive outcome is restoring optical detection and retaining it through fastening and an output test. Clearing cartons from a fixed photo-eye beam, connecting an M12 sensor cable, and setting laser focus height are nearby operations. Holding and turning a fastener alone would not establish distinctness. The live board was screened against 3,760 task IDs on 19 November 2026, including accepted and pending entries. LIBERO, SimplerEnv, RoboCasa, BEHAVIOR-1K, RoboDojo and DataCenterBench were also screened. RoboDojo’s screw-fastening task is a nearby constituent skill; optical alignment retained through fastening and an output test is the additional operational requirement. This is a screening judgment, not a guarantee of reviewer acceptance.

Original task code uses the accompanying MIT license. External robot and simulator assets retain their upstream terms and are not redistributed here.
