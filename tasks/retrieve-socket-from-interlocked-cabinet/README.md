# Retrieve a socket from an interlocked tool cabinet

Retrieve the usable socket from a workshop cabinet and place it on the green mat. Access is coupled: an open drawer prevents its neighbor from opening, and a misplaced screwdriver prevents that drawer from closing. The instruction specifies the outcome without prescribing how to clear the obstruction.

The scripted RGB-only demonstration reaches 1.0 in 976 steps (48.8 simulated seconds), with a 10-step released dwell. A fresh reset reproduces all recorded poses, velocities, joints and initial camera bytes; its 20-step hold scores 0.0. Dream-1 beta reaches 0.0 in one bounded 1,200-step trial (60 simulated seconds), without invalid-state termination.

## Application and distinctness

Tool retrieval is part of ordinary workshop repair. The [US Bureau of Labor Statistics](https://www.bls.gov/ooh/installation-maintenance-and-repair/automotive-service-technicians-and-mechanics.htm) describes sockets and ratchets among the common hand tools used by automotive service technicians and mechanics, an occupation with 825,800 jobs in its 2025 estimate. The occupation count establishes the scale of the work setting; it is not a measured frequency for this particular obstruction.

[Bott's cubio catalog](https://www.bott.de/en/content/download/1581/file/cubio_Hauptkatalog_2025_EN_web.pdf) documents single-drawer blocking and a 525 x 525 x 400 mm cabinet with two 150 mm drawer fronts (40010005). This task uses that full-size envelope. Stacked blocking elements and wedges are an established [drawer-interlock mechanism](https://patents.google.com/patent/US8696074). The internal geometry here is a generic, fully specified implementation, not manufacturer CAD. The cabinet is bolted to a bench; this is access recovery, without a cabinet-tipping claim.

The meaningful difference from generic drawer opening, external-bar removal and pick-and-place is that clearing one obstruction releases a separate, physically interlocked drawer. Pulling harder on the target drawer does not release it. The socket has functional hex and square openings, rather than being represented by an arbitrary block. No pending or accepted direct match was found in the board snapshot checked on 2026-11-20.

## Reproduce the setup

All coordinates are metres in a right-handed frame: x points away from the robot bases, y to their left, z up. All initial orientations are identity. `spec.py` and `scene.py` are authoritative, deterministic construction data. There is no object jitter.

| Object | Dimensions/material | Initial location and constraint |
| --- | --- | --- |
| Workbench | 1.10 x 1.30 m timber top, 60 mm thick, four steel legs | Top z=0.500; center x=0.630, y=0; fixed |
| Cabinet | 525 x 525 x 400 mm steel envelope; simplified 4 mm walls | Front x=0.450, back x=0.975, y +/-0.2625, bottom z=0.500; bolted to bench |
| Lower drawer | 400 x 400 mm sheet-steel tray; 4 mm collision bottom and walls, 95 mm wall height; 500 x 145 x 8 mm folded-front envelope; 2.5 kg | Floor-body origin (0.250,0,0.535); 200 mm open |
| Upper drawer | Same tray, front and mass | Origin (0.450,0,0.685); closed |
| Drawer handles | Rectangular polymer grip 120 x 20 x 20 mm; two steel standoffs | Grip center 50 mm in front of each drawer origin, 75 mm above floor-body origin |
| Interlock | Two steel bars, 10 x 12 mm cross section, heights 150/20 mm, masses 0.12/0.04 kg; 6 mm cam rise over 20 mm | Bar x=0.520,y=0.235; lower base z=0.550, upper base z=0.7008; reset both raised 6.1 mm; vertical travel 6.5 mm |
| Screwdriver | 207 mm total; 6 mm steel shaft with flat tip; octagonal polymer grip 32 mm across circumscribed diameter and 100 mm long; 0.120 kg | Tip at (0.310,0.070,0.539); upright in an 8 mm square shaft bore, 50 mm high, attached to lower tray |
| Socket | Chrome-vanadium steel approximation, 26 mm exterior diameter, 37 mm long, 0.080 kg; 17 mm hex opening 20 mm deep, 12.7 mm square drive 14 mm deep, 3 mm web with 6 mm passage | Bottom at (0.540,-0.070,0.690), upright in upper tray; gravity settles it onto the tray |
| Green destination mat | 200 x 200 x 2 mm polymer | Center (0.230,-0.420,0.501), fixed |
| Tool parking pad and cup | 250 x 250 x 2 mm pad; open 36 mm square cup with 4 mm walls, 120 mm high | Pad center (0.250,0.430,0.501); cup center x=0.300,y=0.430, floor z=0.502; fixed |
| Two YAM arms | Standard link dimensions, 75 mm parallel-jaw aperture, 40 N per finger | Base positions (0,+/-0.305,0.750), fixed on steel pedestals |

The socket size follows the [17 x 37 mm, 1/2-inch drive manufacturer example](https://hybris-media.wera.de/download/pdfgenerator-datasheets/de/05003608001.pdf). Its circular exterior and bore depths are simplified; mass distribution, knurling and drive detent are not modeled. The collision mesh consists of separate convex annular sectors so the holes remain physically open. No branded markings or logos are rendered.

Panel collision solids are deliberately thicker than nominal thin folded sheet metal. The specified 2.5 kg unloaded drawer mass is an approximation, not a density integral of those collision solids or a manufacturer measurement. Floor top surfaces and exterior clearances use the published construction coordinates.

The drawer guides are native prismatic joints with 400 mm travel, force-mode damping 30 N s/m capped at 3 N, and no position motor. The closed cam tip has 3 mm clearance from the follower edge. Cam contact raises the stacked bars; a fixed ceiling limits their shared displacement. Bars use native guides and 0.2 N s/m damping capped at 0.2 N. There is no software drawer-lock flag, grasp attachment, object teleport or phase-triggered unlock in a trial.

## Physical judging and scoring

Judge the terminal state, taking the highest applicable tier. Use a ruler at the drawer-front datum, the visible mat edges, a 3 mm feeler gauge and the synchronized camera video.

| Score/condition | Physical check | Simulator implementation |
| --- | --- | --- |
| 0.25 | Lower drawer front within 5 mm of its closed datum | Native drawer displacement |
| 0.5 | Upper drawer front at least 200 mm outward from its closed datum | Native drawer displacement |
| 1.0 | Socket wholly within the mat outline, lowest point within 3 mm of its top; all grippers at least 5 mm clear of drawers and tools; all movable objects stable within 2 mm and 2 degrees for ten successive control samples | Bounds, gripper clearance and a rolling pose-stability window plus consecutive completion counter |
| Otherwise 0.0 | No qualifying outcome | No partial state satisfied |
| Permanent 0.0 | Object leaves the workspace; cabinet tips over 15 degrees; tools, fixture or mechanism are damaged/bypassed | Full geometry workspace bounds; fixture displacement; native guide separation/rotation; impossible simultaneous drawer extension |

Workspace: x in [-0.150,1.200], y in [-0.650,0.650], z in [0.490,1.350]; all movable geometry must remain inside. Guide integrity allows no more than 2 mm transverse separation or 2 degrees rotation; displacement beyond travel by more than 2 mm is damage. Both drawers more than 30 mm open is a bypassed interlock. Fixed objects moving more than 2 mm is a fixture fault. These are evidence-validity checks, not extra manipulation skills. A dropped socket on the bench does not earn placement credit; a gripper holding it over the mat does not qualify. Physical judging checks support, clearance and motion from close views, not hidden force telemetry.

No mark or partial score persists after the corresponding physical condition is lost. Invalid events remain latched until reset. Ten completed control intervals correspond to 0.5 seconds at 20 Hz. The native simulation uses 480 Hz PhysX with TGS, 96 position iterations for the props and zero velocity iterations. Collision/rest offsets are 0.1/0 mm for props and 0.2/0 mm for robot geometry. Prop friction is 0.25; rubber pads use static/dynamic friction 1.2/1.0 with a 2 mm torsional patch. Sleeping is disabled for the small mechanism bodies so direct calibration forces are processed consistently. No increased finger force or enlarged tool is used to obtain a grasp.

## Policy inputs and reset

Both runs share the same scene, control rate, action limits, seed 0 and object_jitter=0. The three permitted RGB cameras are `overhead_cam`, `left_wrist_cam` and `right_wrist_cam`. Native frames are 640 x 360; delivered observations are bilinearly resized to 320 x 176 at 20 Hz. The fixed camera is at (0.350,0,1.850), aimed at (0.480,0,0.600). The two standard wrist-camera mounts and intrinsics come from the YAM adapter. The socket is naturally hidden in its closed drawer and becomes observable when opened.

Dream-1 receives the RGB streams and the standard robot joint/gripper readings. It receives no object poses, contacts, score or evaluator history. The demo receives only the RGB streams and executes a deterministic offline-IK rehearsal of the fixed layout. It does not claim visual generalization; it is nonprivileged feasibility evidence. `demo_observations=true` is not used.

Reset closes and rebuilds the environment, all rigid bodies and guide joints. It restores authored poses, zeroes linear/angular and joint velocities, restores robot joints, resets scoring and stability history, renders after restoration, and explicitly clears camera buffers. The saved initial camera bytes and every rigid/robot reset value are compared after the completed demonstration. There are no consumed parts.

## Run and verify

Install the pinned dependencies listed in `DEPENDENCIES.json`, including Isaac Sim/Lab, Arena, the YAM adapter and Inspect Robots. Set `ISAAC_PYTHON`, `INSPECT_OVERLAY` and `ARENA_ROOT` to the installed runtime locations. For the documented Arena revision and Lab version, run `python prepare_arena_compat.py "$ARENA_ROOT"` on an isolated Arena checkout. Keep credentials in environment variables only.

```bash
export PATH="$INSPECT_OVERLAY/bin:$PATH"
export PYTHONPATH="$INSPECT_OVERLAY:$ARENA_ROOT"
export OMNI_KIT_ACCEPT_EULA=YES ACCEPT_EULA=Y
# On Linux ARM64, also set LD_PRELOAD=/lib/aarch64-linux-gnu/libgomp.so.1
inspect-robots list embodiments
inspect-robots list policies
bash run_inspect.sh --policy hold --steps 200 --output RUN_SMOKE
bash run_inspect.sh --policy demo --steps 1100 --check-reset --output RUN_DEMO
DREAM_API_KEY=... bash run_inspect.sh --policy dream --steps 1200 --budget-ledger REQUEST_LEDGER.jsonl --output RUN_DREAM
python scorer.py RUN_DEMO/evidence/episode-00/trace.jsonl
python audit_run.py RUN_DEMO --require-full
python audit_run.py RUN_DREAM
python -m unittest test_scorer test_dream_budget
"$ISAAC_PYTHON" calibrate_interlock.py --output MECHANISM_CHECK
inspect-robots video RUN_DEMO/inspect/LOG.json --out VIDEO_DEMO --fps 20
python export_media.py RUN_DEMO/evidence/episode-00 VIDEO_DEMO_H264 --name demo
python export_media.py RUN_DREAM/evidence/episode-00 VIDEO_MODEL_H264 --name model_run
```

Submission metadata also passed the supplied task validator. `run_inspect.py` invokes official Inspect evaluation with `store_frames=True`, seed 0 and one episode. Each action contains six absolute joint angles and normalized gripper opening per arm. The task limit is 2,000 steps (100 simulated seconds); the bounded model trial may stop earlier as recorded. `dream_budget.py` reserves up to $1.20 across at most 60 attempts, including the client's retry, in a persistent ledger and checks the current model price before calling it. These are conservative reservations, not an invoice. The GPU lock prevents overlapping local runs. Each process has a 2,400-second watchdog.

The calibration is separate from policy evaluation. It applies known forces to the drawer bodies, removes the obstructing screwdriver between cases, and checks both directions of the interlock; it cannot establish a robot feasibility score.

The local force calibration passes both interlock directions: a 40 N pull permits only 3.0 mm upper-drawer play while the lower is open and 4.4 mm lower-drawer play while the upper is open. With the screwdriver removed, a 20 N push closes the lower drawer, and a 40 N pull opens the upper by 371 mm in the test interval. The screwdriver otherwise blocks closure with about 71 mm remaining. These are simulation calibration measurements. See `mechanism-validation.json`. The cams are continuous convex solids to avoid internal collision seams.

Camera review used actual 320 x 176 policy frames as well as native views. The screwdriver and drawer handles are resolved in the corresponding wrist views. The socket is small overhead and naturally hidden before the drawer opens; its bore and released placement are clear in the right wrist view. Fingers partly occlude it during grasp. No target flickering or z-fighting was observed in sampled approach, grasp, opening, transfer and release frames.

The H.264 exporter produces yuv420p MP4 with faststart; maximum 50 MB per video. `setup.png` is exactly the first native overhead frame, maximum 10 MB. Policy frames, native frames, source hashes, action/score logs and reset receipts are retained with the local evidence. The final receipts are in `local-validation.json`: demo episode wall time 306.58 s (319.51 s for the complete process including reset audit); Dream episode wall time 463.05 s. Nine CPU tests and the score, source, reset and delivered-frame audits pass. Dream used 25 action chunks and 51 reserved attempts, with 24 recovered session expirations; the persistent ledger reserved $1.02, below the $1.20 cap. Reservations are conservative bounds, not billed charges. Maximum/mean adapter tracking error was 0.2151/0.0443 rad. Both runs used the same executable source.

## Dependencies and licenses

The external **YAM robot asset is MIT-licensed**, including its USD/URDF geometry and materials; retain its upstream `LICENSE.i2rt`. It is supplied through the [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena) at revision `0059d02a1a91bc6834c1d0846022891df0c78730` and is not redistributed in this task. Procedural task geometry and task code use the included MIT `LICENSE`. Manufacturer documents and patents are references only; their text or CAD are not redistributed.

Verified runtime pins: Isaac Sim 6.2.0.0; Isaac Lab 47.1.0 (`892354c283df78fabf2ff4661902a3a1504b3c82`); Arena `c7edf3a9d5d8276565208f72422f8f8a0806d6cf`; Inspect Robots 0.60.0; Inspect YAM 0.37.0; Dream client 1.0.14; Dream adapter `b5e936a201cec66c54c297e522c920d98445e339`; Python 3.12.13; NumPy 2.5.1; Pillow 12.3.0; Torch 2.11.0+cu130. Simulation runs on native CPU PhysX with local GPU RTX rendering.
