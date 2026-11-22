# Clear a blockage from an attached vacuum floor nozzle

Hold the raised floor nozzle steady. Pull out the wedged paperboard, put it in the tray, and lower the connected nozzle. The complete vacuum is unplugged. The 280 mm floor tool, 32 mm wand, hose and canister remain connected for the trial. The blockage procedure calls for checking the assembled cleaning equipment when a vacuum stops picking up. [Floor tool specification](https://numatic.com/products/280mm-proflo-combi-floor-tool/) · [Blockage procedure](https://cdn.numatic.com/wp-content/uploads/910074-HENRY-HETTY-620W-EURO.pdf)

The head starts at 67° with the suction mouth visible. The worktop is 747.5 mm high, with the head pivot at x = 420 mm and y = −45 mm, 52 mm above the worktop. The tray's clear area is 188 × 158 mm, centered at x = 250 mm and y = −285 mm. The 280 × 122 mm plastic floor tool has a rigid shroud, soft leading and trailing strips, and an 88 mm wide central channel. Put the 300 mm diameter plastic canister at x = 652 mm and y = −404 mm, with the flexible hose running from the canister to the 32 mm metal wand behind the head. The bench supports the assembly at its reset marks.

Make the obstruction from 350 g/m² paperboard. Cut the stock 62 mm wide and crease across its width, forming a flattened diamond profile 58 mm long with a 23 mm ridge halfway along. The two flat tips are 8–10 mm thick. Mark 26 mm from one tip and insert to the mark, leaving 32 mm visible. The fold should stay in place with the nozzle at 67°. Use the same uncrushed shape or remake it to the same dimensions for every trial.

At the side of the nozzle, use a 10 mm witness mark and an angle gauge to check extraction along the mouth at a head angle of at least 50°. Check that the whole original fold rests inside the tray. For full credit, use a side view to verify the sole is within 5° of flat, the nearest finger is at least 25 mm from the shell, and both hands are clear of the fold for 0.5 s. Inspect the original board and assembly for damage. The spring-backed strips approximate the elastic grip of the real nozzle's skirt on the folded paperboard. The folded shape is rigid and the model has no powered airflow. The bench holds the canister and integrated wand in place.

The external I2RT YAM robot asset includes the arms and grippers under the MIT license. The simulation uses the stock YAM Isaac Lab-Arena adapter. The procedural scene and the adapted benchmark asset helper use the included MIT license. [External YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

Activate the installed IsaacLab-Arena environment. Use seed 0, zero object jitter and 320 controls at 20 Hz. The scripted demonstration uses fixed poses. Read the terminal `task_score` from Inspect.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 320 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction 'Hold the raised floor nozzle steady. Pull out the wedged paperboard, put it in the tray, and lower the connected nozzle.' \
  --log-dir rollout
```

Set `NOZZLE_AUDIT_DIR` to an existing directory and use the scripted arguments with `python audit_nozzle.py run` to save three policy and native camera views and verify the reset.
