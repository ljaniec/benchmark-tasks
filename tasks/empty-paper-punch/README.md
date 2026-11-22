# Empty the scraps from a paper punch

Empty the paper scraps into the waste tray. Close the punch's bottom cover while holding it, then leave it upright on its starting pad.

The task exercises holding an object steady while releasing its waste cover, controlling small loose pieces during the drop, and securing the cover for the next use. The scene is an unbranded model based on a 270 g compact two-hole punch with a hinged plastic waste cover. It uses the reference product's 100 × 70 × 81 mm class of body, with a small modeled release grip. The original design holds one part of the waste receptacle on the body while the other folds down. [Product specification](https://www.leitz.com/en-gb/products/stapling--punching/desktop-hole-punches/leitz-nexxt-wow-mini-hole-punch/) · [Leitz closure design](https://patents.google.com/patent/EP0318650B1/en)

## Setup and reset

- Use the standard YAM station. Coordinates are in metres, with x away from the arm bases, y to the left and the desk at z = 0.7475.
- Place the punch upright at x = 0.285 and y = 0.120 with the closed cover on the desk. Its rear hinge is along the y axis. The body is steel with a plastic lever cover and a hinged plastic waste cover. Tape a thin outline around the resting punch to mark its starting pad.
- Put a shallow plastic waste tray at x = 0.350 and y = −0.040. Its inside opening is 220 × 190 mm. The floor is 6 mm above the desk, and the rim is 26 mm above its floor.
- Reset with 48 cardstock disks, each 5.5 mm across, 0.38 mm thick and about 6 mg, loaded inside the closed cover. Use the same disk count and clear the desk and tray each time. The simulated order is two fixed 4 × 6 layers, with seed 0 and zero layout jitter.

## How to judge

Count the original disks in the tray after the scene has settled. An original disk resting against the inside wall counts when its centre is inside the walls and below the rim. Partial and full credit require all 48 accounted for in the tray or the punch, no damage, all parts within the station and both grippers 35 mm clear for 0.5 s. Award 0.25 for at least 24 disks in the tray and 0.5 for all 48.

For 1.0, watch the free edge of the cover in the side view as the punch is raised at least 45 mm above the desk. The base faces down, within 15° of horizontal. For at least 0.5 s the cover must stay in its normal closed position, within 2° of shut. During the lift the holding hand grips only the housing, with at least 6 mm between that hand and the cover. The other hand stays at least 35 mm from the cover. Use a ruler for lift height and hand clearance and compare the cover edge to the initial closed profile in the side view. Then check that the punch is upright within 5° on the starting pad, with the base centre within 18 mm of the starting centre. Use the overhead view for the tray and pad. If a pile hides scraps, spread the finished tray contents on a sheet to confirm the count.

## Simulation and evidence

The waste cover has an unpowered revolute hinge and a spring-loaded bead that encounters a fixed keeper. The modeled release grip has a flat 16 × 9.5 mm contact face. Press the clip inward and guide the cover down so it clears the keeper. The grip and internal clip dimensions and spring rate are model estimates. The lever stays in its resting shape. The tray has the same upper collision surface as its visible floor, with a deeper collision backing below that surface. The discrete cardstock disks are rigid.

The trial uses 480 Hz physics, 20 Hz actions, 320 × 176 delivered RGB, and standard robot joint and gripper readings. The three native RGB views are 640 × 360. The scripted demonstration uses fixed waypoints. The right wrist camera moves to a clear side view during the lift that checks the cover. Per-step object geometry used by the evaluator stays outside the policy inputs.

```sh
python -B audit_punch_demo.py run \
  --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 520 \
  --store-frames --log-dir ./demo-log --no-prompt --grader none --scorer task_score \
  --instruction "Empty the paper scraps into the waste tray. Close the punch's bottom cover while holding it, then leave it upright on its starting pad."
```

Set `PUNCH_AUDIT_DIR` to a fresh writable directory and `PUNCH_TRACE` to a path in that directory before the run. The audit saves matched-seed initial and reset states, timestamps, score counters and all three RGB views. The reset returns the punch, its joints and each scrap to their initial poses with zero velocity and reinitializes the three cameras. Use 300 controls to smoke-test the opening and collection phases.

The demonstration scored 1.0 in 520 controls with seed 0. It took 243.3 seconds of wall time on an NVIDIA L40S with Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and PyTorch 2.11.0 with CUDA 12.8. The first full score was at control 502. An independent replay of the recorded trials under the cover-retention rule matched every step and both terminal scores.

Dream-1 scored 0.0 in 520 controls with seed 0 on the same scene. The run took 269.4 seconds. Both recordings show the actual three policy views at 20 frames per second.

```sh
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90 \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 520 \
  --store-frames --log-dir ./model-log --no-prompt --grader none --scorer task_score \
  --instruction "Empty the paper scraps into the waste tray. Close the punch's bottom cover while holding it, then leave it upright on its starting pad."
jq -r '.results.metrics.task_score' ./demo-log/adhoc_*.json ./model-log/adhoc_*.json
```

Install the [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena) separately. The external YAM robot asset includes the I2RT arms, parallel grippers, meshes and collision hulls, distributed under the [I2RT MIT license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt). The procedural scene and task code use the included MIT license. The local `mechkit.py` adapts the MIT helper from the benchmark's [brake-pad task](https://github.com/robocurve/benchmark-tasks/tree/main/tasks/brake-pads-caliper).
