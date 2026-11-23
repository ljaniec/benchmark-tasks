# Stop a feeder from sending two carton blanks at once

Adjust the separator so the rollers take only the bottom blank, then run the two original blanks through the feeder. Operators set the separator when changing stock or correcting a double feed in packaging and document equipment. The lower blank must pass while the upper blank waits at the throat. A released test cycle gives an immediate check of the setting. [Packaging feeder setup](https://www.superior-phs.com/_files/ugd/3e9481_a73376a0426046098fab9d92faf3fdac.pdf) · [Example sheet feeder adjustment](https://www.pitneybowes.com/content/dam/support/product-documentation/di880-di875-f600/en/pulse-input-output-mod-op-guide-en.pdf#page=63)

## Physical setup

Use the dimensions in `feeder_geometry.py` and `mounted_feeder_fixture.py` for the compact steel feeder. The station uses a 1,100 × 1,300 × 60 mm worktop at 747.5 mm height and evaluating YAM bases at x = 0, y = ±305 mm. The feeder's origin is x = 300 mm, y = 0, 17 mm above the worktop. Its infeed faces the evaluating robot. Five 36 mm diameter, 84 mm wide rubber rollers turn about parallel horizontal axes. They are centered at local x coordinates −57, −14, 30, 74 and 116 mm. The deck is 28 mm above the origin. The steel base sits under the rollers, and a clear polycarbonate guard covers the feed path. The test button and winged separator screw remain outside the guard, at the locations in the scene file.

Use a low-voltage geared drive and common timing belt to turn all five rollers on the fixed board path at approximately 38 rpm, with their top surfaces moving toward the exit. Set a 6 s one-shot timer. A normally open momentary button starts one cycle and the timer switches the drive off after it expires. Keep the drive and timing circuit enclosed and the guard fitted. The simulation uses the same fixed-duration button cycle and limits the effort on each roller to 0.08 N·m.

Set the visible screw to the 9.9 mm wide reset opening. Use two reusable chipboard blanks, each 95 × 85 × 4.2 mm and 20 g. Mark the lower blank edge cyan and the upper edge cream. Place both flat against the same infeed stop with aligned front edges. Stop the rollers. After each trial restore the initial opening, both aligned blanks, the button, timed cycle and stop state. The simulation also resets poses, velocities, score state and each RGB camera buffer.

## Judging a trial

Record the feed path through the guard with a fixed side view at 20 Hz or faster. The exit marks are 49 mm downstream from the center of the separator. Award 0.25 when the lower front has crossed the exit marks and the upper front has not reached the line 14 mm downstream of the separator center when the lower rear reaches the witness line 10 mm upstream of its center. Award 0.5 when the upper front crosses the same exit marks at least 0.60 s after the lower front. For full credit both blanks remain clear of the separator for 0.5 s after the cycle stops. Moving the blanks by hand through the feeder, a double feed, leaving the marked station or tipping the machine above 15° invalidates the result. Use frame counts and the visible witness lines.

The five modeled roller surfaces physically transfer the stock, and the wing screw drives the sliding gate through a native coupling. The compact frame, drive gearing and rigid stock are approximations. The YAM embodiment uses the external I2RT YAM arm and parallel-gripper assets under the MIT license. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

Run from the task folder in Isaac Lab-Arena. Use seed 0, 20 Hz, zero object jitter, and the three 320 × 176 RGB policy streams.

```sh
python -m pytest -q test_feed_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/mounted_feeder.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 330 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction 'Adjust the separator to feed one blank at a time. Press the test button and confirm that both blanks feed through separately.' \
  --log-dir rollout
```

Read the terminal `task_score`. For Dream-1, use `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90`. The camera streams and robot proprioception use the standard model adapter. The scripted path is fixed.

## Verified runs

Both trials used seed 0, zero jitter, RGB at 320 × 176, 20 Hz control and 480 Hz physics. The runtime used Inspect Robots 0.60.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu128.

| Policy | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 330 | 132.5 s |
| Dream-1 | 0.00 | 330 | 123.4 s |

In three native trials, a 9.90 mm gap sent both marked blanks through together, a 5.49 mm gap separated them, and a 3.11 mm gap stopped both. The mounted demo independently scored both releases at a 1.40 s interval. The final score includes 0.5 s after the timed drive stops. The matched-seed audit checked the simulator state, episode counters, camera buffers and image timestamps.
