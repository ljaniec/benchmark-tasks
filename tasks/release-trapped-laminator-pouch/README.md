# Release a trapped pouch from a desktop laminator

Hold the laminator’s rear release lever down, grip the folded edge and draw the trapped pouch straight out through the entry slot. Lay it in the marked area. Let the lever return. The assembled laminator starts cool and disconnected. The Fellowes M5-95 manual shows withdrawal through the original entry with the rear release held. [Manufacturer procedure and specification](https://assets.fellowes.com/manuals/M5-95_407894B_NA_A5_092614.pdf)

## Setup and reset

Place the 356 × 158 × 67 mm plastic housing on a 148 × 340 × 43 mm rigid polymer riser with its center at (410, 10) mm on a 747.5 mm high bench. The entry faces negative y. Leave both elastomer pressure rollers and the 240 mm entry assembled. The 34 × 26 mm rear paddle center projects to (290, 105) mm. Begin with the machine switched off and the mains plug visibly disconnected. Let the housing cool before loading the specimen.

Use a 164 × 117 mm polyester pouch holding a 148 × 105 mm paper insert. Seat its sealed end 61 mm into the mouth, centered. Bend a 9 × 75 mm strip near the sealed end to form the leading crease. The modeled ridge is 2.6 mm high. Crease the center 35 mm of the exposed unsealed edge into the folded grip. Its model envelope is 24 × 35 × 19 mm. Load the jam cold by holding the original rear release during placement and then letting it return. Two 205 × 8 mm plastic ledges support the exposed tail. They are centered 116 mm in front of the entry and 51 mm to either side of the feed centerline, with their top surfaces 74.6 mm above the worktop. Mark a 162 × 224 mm rest area centered at (449, −385) mm. Use the same layout, specimen, start depth, and paddle rest mark at every reset.

Use a 4 N spring scale to check the repeatability of the test specimen before a physical trial. With the paddle untouched, axial travel should stay below 5 mm. With it depressed, the specimen should release through the slot. Those are setup acceptance checks. The task uses estimated roller dimensions and spring rates. Production film is thinner than the 2 mm rigid simulation envelope, and the two creased regions represent initial deformations of the scrap pouch.

## Judge the trial

With a ruler and camera or close side view, compare withdrawal to the entry and lever position to the rest mark. The first tier requires 10 mm of straight motion at the slot with the paddle depressed by at least 11 mm from the start of the pull. Judge straight by the pouch’s long edge within 16° of the feed direction and its centerline within 24 mm of the slot center. For 0.5 the far edge of the pouch must clear the mouth by 10 mm before the lever is released. For 1.0 the main face lies flat, its outline and folded end are inside the marks, the lever returns within 2° of rest, and both hands remain 20 mm clear of the pouch, 25 mm clear of the fold center and 40 mm clear of the lever center for 0.5 s. Mark the workspace x = 135–735 mm and y = −515 to 515 mm in the same bench coordinates. Give zero for a damaged part, a lever or housing failure, or a pouch leaving the marked workspace.

The external I2RT YAM arms and parallel grippers are provided by the stock Isaac Lab-Arena YAM adapter under MIT. Procedural task assets and helper code use the included MIT license. [External YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

The demonstration uses a fixed target path and normal robot commands. Inspect reports the terminal `task_score`.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 240 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction 'Hold the laminator’s rear release lever down, grip the folded edge and draw the trapped pouch straight out through the entry slot. Lay it in the marked area. Let the lever return.' \
  --log-dir rollout
```

## Evaluation results

Both runs use seed 0, zero jitter, three RGB views at 320 × 176, and a maximum of 240 controls at 20 Hz. The Dream-1 adapter also receives ordinary joint and gripper readings. Physics runs at 480 Hz. Recorded software versions are Inspect Robots 1.0.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu130.

| Run | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 240 | 117.7 s |
| Dream-1 | 0.00 | 240 | 118.9 s |

For the model run, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` in place of `--policy yam_scripted`.

Under the same 4 N pull limit, the untouched pouch moved 4.44 mm and the released pouch moved 92.70 mm. The mounted run extracted and laid down the same pouch, and the reset audit passed.
