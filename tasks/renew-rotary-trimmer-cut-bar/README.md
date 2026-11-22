# Turn a rotary trimmer’s cutting bar to a fresh face

The trimmer is already open for maintenance. The old face has a visible wear groove. The bar rests with its far end in the track and its raised near end resting on a plastic service lever. Lift out the bar, turn it to an unused face, seat it flush along the track, and close the guide. The guarded blade carriage stays parked throughout the repair. The source specifies exact dimensions and reset poses. [Reference trimmer](https://cuix-prod.fiskars.com/en-us/crafting-and-sewing/products/paper-trimmers/portable-rotary-paper-trimmer-12-199080-1007) · [Cutting-bar service instructions](https://www.xacto.com/support/faqs)

## Set up and judge the repair

Place the 348 × 242 mm rigid deck flat on the YAM worktop with the long edge across the table and the cutting track 335 mm from the arm baseline. Use a 312 × 8 × 8 mm polymer cutting bar, a 322 mm hinged plastic guide, the parked blade carriage and a 70 × 16 mm plastic service lever. Leave the guide open at 98°. Slide the grooved bar 48 mm toward the lever so its raised end rests on the lever and the other end remains in the track. The bar starts with the groove facing up.

Watch the bar leave its track and turn to a fresh face. Measure its seating at both ends against the deck with a straightedge and a feeler gauge. Full score requires both ends within 0.9 mm of the track depth, the guide within 5° of shut and the hands at least 35 mm from the bar for 0.5 s. The guarded carriage stays parked. The model estimates the bar section, wear groove, track clearance and hinge bracket dimensions. The scene starts with one end already pried up.

The external I2RT YAM robot asset, including the arms, parallel grippers and meshes, uses the MIT license. YAM Isaac Lab-Arena supplies the simulator integration. The procedural scene uses the included MIT license and the mechanical asset authoring helpers adapt the benchmark's MIT-licensed examples. [I2RT YAM license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt) · [YAM Isaac Lab-Arena](https://github.com/kairosiann/yam-isaac-lab-arena)

## Run and score

Activate the installed IsaacLab-Arena environment. Use seed 0 and zero object jitter. A full trial has 380 controls at 20 Hz, or 19 seconds of simulated motion. The scripted controller follows fixed poses. Dream-1 receives the three 320 × 176 RGB views and the robot's joint and gripper readings. The native views are 640 × 360. Fixture and bar geometry are available to the scorer.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 380 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Turn the trimmer's worn cutting bar to a fresh face. Seat it flush and close the cutting guide." \
  --log-dir rollout
```

Run the model with the same setup and seed.

```sh
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90 \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 380 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Turn the trimmer's worn cutting bar to a fresh face. Seat it flush and close the cutting guide." \
  --log-dir model-rollout
jq -r '.results.metrics.task_score' rollout/adhoc_*.json model-rollout/adhoc_*.json
```

The scripted demonstration reaches full score at control 368 and finishes at 1.0. It took 116.7 seconds on an NVIDIA L40S using Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and PyTorch 2.11.0 with CUDA 12.8. Dream-1 finished its 380 controls in 127.8 seconds and scored 0.0.

Pass the scripted arguments to `python audit_trimmer.py run` with `TRIMMER_AUDIT_DIR` set to an existing directory for a repeated reset check using all three policy-sized and native views. The check compares physical state, velocities, rates, counters, timestamps and camera poses. Image checks compare the restored scene with the initial views and verify that distinct mid-episode views fail the comparison.
