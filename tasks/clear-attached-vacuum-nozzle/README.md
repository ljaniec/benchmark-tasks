# Clear a blockage from an attached vacuum floor nozzle

Hold the raised floor nozzle steady. Pull out the wedged paperboard, put it in the tray, and lower the connected nozzle. The complete vacuum is unplugged. The 280 mm floor tool, 32 mm wand, hose and canister remain connected for the trial. The blockage procedure calls for checking the assembled cleaning equipment when a vacuum stops picking up. [Floor tool specification](https://numatic.com/products/280mm-proflo-combi-floor-tool/) · [Blockage procedure](https://cdn.numatic.com/wp-content/uploads/910074-HENRY-HETTY-620W-EURO.pdf)

The head starts at 67° with the suction mouth visible. The worktop is 747.5 mm high, with the head pivot at x = 420 mm and y = −45 mm, 52 mm above the worktop. The tray's clear area is 188 × 158 mm, centered at x = 250 mm and y = −218 mm. The floor tool has a rigid shroud, soft leading and trailing strips, and an 88 mm wide central channel. Use 350 g/m² paperboard folded to 62 mm wide, 58 mm long and 23 mm at the thickest part. Insert it about 26 mm, leaving the original fold end exposed. Return the assembly and sheet to their marked starting positions for every run.

Watch the sheet move at least 10 mm along the nozzle mouth while the head is at least 50° from flat. Verify the complete original sheet stays inside the tray. The result finishes with the head within 5° of sole-down and both arms clear for at least 0.5 s. The spring-backed strips approximate the elastic grip of the real nozzle's skirt on the folded paperboard. The folded shape is rigid and the model has no powered airflow. The bench holds the canister and integrated wand in place.

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
