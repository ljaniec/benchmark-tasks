# Restore a sagging desk fan's tilt

Support the loose fan head, aim it straight ahead, and tighten its tilt knob. Set the test saddle on top of the grille to check the clamp, then take both hands away. A correctly tightened pivot holds the full guarded head at the intended aim. A loose one visibly sags. The Urbanjet manual describes holding the head, selecting the tilt and tightening the side knob. [Manufacturer manual](https://fanimation.com/wp-content/uploads/2019/12/Urbanjet_FP7958_OM.pdf) · [Product specifications](https://fanimation.com/wp-content/uploads/2019/12/FP7958_Urbanjet__Spec_08_15_19.pdf)

## Setup and physical scoring

Use the assembled 2.30 kg compact fan with its steel grille and plastic side knob. The modeled height is approximately 305 mm and the grille spans 178 mm. Its round 190 mm base starts at (400, −18) mm on the 747.5 mm high worktop. The face points toward negative x. The pitch axis is 132 mm above the top. The bench holds a 175 g steel or weighted aluminum saddle, its 52 × 35 mm crown supported on two feet at (308, −313) mm. Use rubber facing where the saddle meets the guard. Center the saddle on the top centerline of the grille with its weight line 70 mm in front of the pitch axis. For physical reproduction, confirm that it rests on the guard without contact with the neck or an external fixture. Its extra moment is about 0.12 N·m.

Keep the fan unplugged. Back off the tilt knob until the assembled head can return to its downward stop under its own weight. Mark and repeat that setup. The simulated head starts at −22° and the knob starts 45° before first modeled clamp contact. Place the proof saddle upright on the marked bench position. The unweighted mechanical reference is straight ahead, with the grille face vertical.

Use a square or digital angle gauge beside the grille. Score 0.25 between 12° downward and 6° upward and 0.5 within 2° of vertical. For 1.0 the grille stays within 2°, supporting the released saddle for at least 0.5 s with both grippers clear. Use the close image and a ruler to check clearances, 28 mm from the guard, 46 mm from the adjustment knob center and 47 mm from the saddle center. Give zero if any part is damaged, the base tilts over 5°, moves more than 26 mm, or the saddle leaves x = 120–650 mm and y = −500 to 500 mm. Reset the base, resting head, loose knob and weight after each run.

The simulation uses a reduced static/dynamic friction model of a threaded pitch clamp. The estimated screw diameter is 6 mm, collar radius 12 mm, seating stiffness 0.65 N·m/rad and static friction coefficient 0.25. After the knob reaches first contact, turning against seating resistance raises the pitch joint's breakaway moment. A matched-load test releases the same head at level with the clamp loose and tightened, then applies a 0.12 N·m proof moment. The simulation keeps the guard assembled and represents its openings with a continuous contact envelope. The rectangular grip is a simplified profile and the fan is unpowered.

The arms and grippers are the external I2RT YAM asset, licensed under MIT. The procedural scene and helper code are licensed under the included MIT terms. [External YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

Choose seed 0 at 20 Hz with zero jitter. The scripted demonstration uses fixed target paths and normal robot commands. Inspect reports the released terminal `task_score`.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 355 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Support the loose fan head, aim it straight ahead, and tighten its tilt knob. Set the test saddle on top of the grille to check the clamp, then take both hands away." --log-dir rollout
```

## Evaluation results

Both runs use seed 0, zero jitter, three RGB views at 320 × 176, and a maximum of 355 controls at 20 Hz. The Dream-1 adapter also receives ordinary joint and gripper readings. Physics runs at 480 Hz. Recorded software versions are Inspect Robots 1.0.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu130.

| Run | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 355 | 127.2 s |
| Dream-1 | 0.00 | 355 | 118.8 s |

For the model run, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` in place of `--policy yam_scripted`.

The head settled at -1.425° with the 175 g weight released. In the native controls, the loose head dropped to -12.34°. The tightened head moved at most 0.157° with the 0.12 N·m proof applied. The released demonstration passed the same-seed reset audit.
