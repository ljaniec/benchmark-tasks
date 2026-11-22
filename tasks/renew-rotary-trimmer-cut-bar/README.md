# Turn a rotary trimmer’s cutting bar to a fresh face

The trimmer is already open for maintenance. The old face has a visible wear groove. The bar rests with its far end in the track and its raised near end resting on a plastic service lever. Lift out the bar, turn it to an unused face, seat it flush along the track, and close the guide. The guarded blade carriage stays parked throughout the repair. The source specifies exact dimensions and reset poses. [Reference trimmer](https://cuix-prod.fiskars.com/en-us/crafting-and-sewing/products/paper-trimmers/portable-rotary-paper-trimmer-12-199080-1007) · [Cutting-bar service instructions](https://www.xacto.com/support/faqs)

The complete deck is 348 mm long. The 312 × 8 × 8 mm polymer bar is free to move, and the guide runs on a physical hinge. The bar section and groove, track clearance and hinge bracket dimensions are estimates. The reset starts after the near end has been pried up. Judge the repair by the exposed face, the gap at both ends and the closed guide. The code checks these poses during the episode and requires a released full state for ten control steps.

The external [I2RT YAM robot asset](https://github.com/i2rt-robotics/i2rt) is MIT-licensed. The robot meshes and controller are supplied by [YAM Isaac Lab-Arena](https://github.com/kairosiann/yam-isaac-lab-arena). Mechanical asset authoring helpers are derived from the MIT-licensed benchmark examples.

## Run and score

Activate the installed IsaacLab-Arena environment. The fixed seed is 0. A full trial has 380 controls at 20 Hz, or 19 seconds of simulated motion. The scripted controller follows fixed poses and does not use simulator object observations.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 380 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Turn the trimmer's worn cutting bar to a fresh face. Seat it flush and close the cutting guide." \
  --log-dir rollout
```

Use the terminal `task_score` in Inspect’s results. The same command can be passed through `python audit_trimmer.py run` with `TRIMMER_AUDIT_DIR` set to an existing directory to capture the three policy-sized and native views and check a same-seed reset. The scoring code reads fixture and bar geometry for judging.
