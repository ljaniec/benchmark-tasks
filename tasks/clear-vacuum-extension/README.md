# Clear a blockage from a vacuum extension tube

Clear the blockage from the vacuum tube. Leave the debris in the tray and return the clearing rod to its rest.

Clearing a lodged blockage restores airflow in a vacuum used for everyday cleaning. The extension is the full-size stainless tube from a canister vacuum. Its manufacturer directs owners to inspect the wand after unplugging the appliance and describes pushing a stubborn blockage free. This task starts with the affected straight extension removed and uses a blunt rod to push the debris into a tray. Open saddles support the tube and leave it free to slide if the operator fails to steady it. [Extension dimensions](https://cdn.numatic.com/wp-content/uploads/EXP-001101-MED900A-Exploded-Drawing.pdf) · [Operating manual](https://cdn.numatic.com/wp-content/uploads/911967-HVR160-UK-Manual-1-1.pdf) · [Blockage procedure](https://henryhub.myhenry.com/how-to-unblock-a-henry-vacuum/)

## Setup and reset

- Use the standard YAM station with its desk 747.5 mm high. Coordinates use x away from the arms and y to their left. Unplug and detach the affected extension before preparing the bench.
- Place the stainless tube parallel to y at x = 280 mm and z = 850.5 mm, with mouths at y = −38 mm and y = 384 mm. The tube is 422 mm long and 32 mm in outer diameter. The modeled bore is 30 mm and the mass is 329 g. Measure the bore on the selected physical tube.
- The two tube saddles have hardwood bases 60 × 30 × 87 mm at y = 40 mm and y = 289 mm. Each saddle has a 36 mm opening between two 12 mm wide sides that rise 19 mm above its base. The saddles only support the tube from below and the sides.
- The clearing tool has a blunt wooden shaft 484 mm long and 10 mm thick and a 32 × 35 × 30 mm rectangular wood handle. Rest it parallel to y with the handle centre at x = 332 mm, y = −553.5 mm and z = 850.5 mm and the tip pointing toward positive y. The two rod saddles at y = −422 mm and y = −154 mm have 48 × 30 × 98 mm hardwood bases and a 24 mm opening between sides that rise 8 mm above their bases.
- Place an empty plastic catch tray at x = 280 mm and y = 470 mm. It has a 180 × 190 mm interior, an 8 mm high floor and a 29 mm rim above that floor.
- Start with one 23 mm long, approximately 32.8 mm diameter orange plug of compressible packing foam. Compress it into the catch-side mouth so its centre is 50 mm inside the tube. The reset must hold it in the horizontal tube under gravity. Check that the blunt rod passes freely through the empty tube before preparing the obstruction. Use the same plug, starting orientation and marked bench positions for all resets.

## How to judge

Use the final 0.5 s to judge the following visible conditions. Check each scored condition over all 10 controls. Award 0.25 for the lodged plug protruding at least 5 mm past either mouth. Award 0.5 for the intact plug completely out of the tube and resting inside the tray. For captured foam, keep both grippers at least 35 mm clear and judge its centre below the rim and at least 8 mm inside each wall. An overhead or close side view with a ruler can verify both distances.

Award 1.0 if the intact plug stays in the tray, the tube and rod settle into their own pairs of saddles and each gripper stays at least 35 mm clear of the tube, rod and plug for the full interval. Check that both shafts span their saddle centre lines by at least 15 mm beyond the first and last support. The tube and rod centre lines stay within 3 mm of their resting height at each support and within 3 mm and 8 mm laterally, respectively, of the support centre. You can mark the centre lines on the saddle bases. Use an end view to inspect the clear bore and a view of the bench to check the tray and supports.

Award 0.0 for other outcomes, damage or leaving the work area. To mark the work area, use x = 10 to 720 mm, y = −640 to 640 mm and z = 742.5 to 1135 mm. Check the tube and rod centre-line endpoints and the plug centre against those limits. The simulator uses these same geometry checks. Its stillness tolerance for full score is 25 mm/s for the tube, rod and plug.

## Simulation and run commands

The steel tube uses 20 collision facets and stands on the same open saddles used for the physical setup. Four spring-loaded contact pads approximate radial compression of the foam. The bore diameter, mass and foam contact model are estimates. In an isolated simulator test at 480 Hz, the foam stayed seated without a rod. An aligned rod contacted the foam with a 0.25 N force limit and moved it less than 1 mm. An aligned push with an 8 N limit ejected it, and a laterally offset push did not move it. These limits describe the simulation contact controls.

Use three 320 × 176 RGB images and standard joint and gripper readings at 20 Hz. The native RGB views are 640 × 360. The scripted route uses fixed nominal waypoints. After releasing the tube, the left arm moves its standard wrist camera above the tray for the final inspection. The evaluator uses the same object geometry a judge can see in the scene, outside the policy observations.

```sh
instruction="Clear the blockage from the vacuum tube. Leave the debris in the tray and return the clearing rod to its rest."
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 420 \
  --store-frames --log-dir ./demo-log --no-prompt --grader none \
  --scorer task_score --instruction "$instruction"
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90 \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 420 \
  --store-frames --log-dir ./model-log --no-prompt --grader none \
  --scorer task_score --instruction "$instruction"
jq -r '.results.metrics.task_score' ./demo-log/adhoc_*.json ./model-log/adhoc_*.json
```

Use 230 controls for a smoke test of the grasp and clearing stroke. The 420-control run represents 21 s of simulated execution at 20 Hz. Run `python -m pytest -q test_scorer.py` for the CPU checks. On reset the simulation restores poses, velocities, spring-pad joints, scoring counters and camera buffers.

The full demonstration scored 1.0 in 420 controls with seed 0, reaching full score at control 398 and maintaining it through the end. The run took 175.7 seconds on an NVIDIA L40S with Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and PyTorch 2.11.0 with CUDA 12.8. The reset audit passed for poses, velocities, joints, counters, timing and cameras. To repeat the audit, set `VACUUM_AUDIT_DIR` to a fresh writable directory and substitute `python -B audit_vacuum.py run` for `inspect-robots run` in the demo command.

Dream-1 scored 0.0 in 420 controls with seed 0 on the same scene and software versions. The model run took 201.1 seconds. Both recordings show the three policy views at 20 fps. The setup image is the first overhead frame.

Install the YAM Isaac Lab-Arena adapter separately. The external I2RT YAM robot asset includes the arms, grippers, meshes and collision hulls under the MIT license. The local `mechkit.py` adapts the MIT helper from the benchmark's brake-pad task. [YAM adapter revision](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1) · [I2RT MIT license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt) · [Brake-pad task](https://github.com/robocurve/benchmark-tasks/tree/main/tasks/brake-pads-caliper)
