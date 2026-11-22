# Refit the cutter assembly on a hand-crank classroom sharpener

Seat the replacement cutter with its original crank in the sharpener’s mounted housing. Turn the retaining ring clockwise, then turn the crank to verify that the cutter stays in place and the shaft turns. The original sharpener guard and shavings drawer stay in place. [Manufacturer replacement procedure](https://cdn.shopify.com/s/files/1/0148/9147/files/How-to-Replace-the-Blade-2.jpg?v=1613149072) · [Replacement assembly](https://www.classroomfriendlysupplies.com/products/replacement-blade) · [Manufacturer table mount](https://www.classroomfriendlysupplies.com/collections/replacement-parts/products/sharpener-mount)

The plastic housing stays fixed on its 130 × 83 mm desk mount, with the bore center at x = 408 mm and y = 36 mm, 91 mm above the worktop. The sharpener body is about 118 × 77 × 150 mm. Start the steel replacement cutter with its nose supported in the bore, 43 mm out from the seated position. Keep its 44 × 54 mm plastic collar aligned with the entry slots and set the crank 59° counterclockwise from downward when viewed from the rear of the sharpener. Put witness marks on the collar, crank and case. At setup the seat witness line is visible and the cutter can slide into the bore without turning the ring. Return all three joints to these marks and clear the robot and cameras between episodes.

Use a side view and a 1.5 mm feeler gauge to check the seated gap. The installed collar mark is at a nominal 67° with an 8° tolerance on either side. After securing the collar, check that the marked crank turns at least 45° clockwise while the sleeve stays seated. Both grippers must release the mechanism for at least 0.5 s and the secured state must persist. The model uses two profiled tracks as a contact approximation of the original retaining ring. Beveled flats approximate the original ring grip. The track and clearances are estimates. A fixed tabletop mount represents the workshop clamp.

In a matched simulation check under a 5 N axial pull, the open collar allowed 41.0 mm of cutter withdrawal. With the collar engaged, withdrawal was 1.04 mm and the independent crank remained free to turn.

The external I2RT YAM arm and gripper assets are MIT licensed. The task runs through the stock Isaac Lab-Arena YAM adapter. The procedural assets and helper use the included MIT license. [External YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

Use seed 0, zero object jitter, and 320 controls at 20 Hz. The demo follows a fixed geometry trajectory. The reported outcome is Inspect’s terminal `task_score`.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 320 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction 'Seat the replacement cutter, turn its retaining ring clockwise, and turn the crank to check that the cutter stays in the sharpener.' \
  --log-dir rollout
```

## Evaluation results

Both runs used seed 0, zero jitter, 320 × 176 delivered RGB streams, and 320 controls at 20 Hz. Dream-1 also received the robot's joint and gripper readings. The physics ran at 480 Hz. The recorded versions were Inspect Robots 0.60.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu128.

| Run | Terminal score | Wall time |
| --- | ---: | ---: |
| Scripted demo | 1.0 | 114.1 s |
| Dream-1 | 0.00 | 95.6 s |

For the model run use the same embodiment, scene, rates, seed, jitter and instruction with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90`. The score is Inspect's terminal `task_score`.
