# Park a bicycle on its kickstand

Park the bicycle so that its tires and the kickstand support it after the robot lets go. The maneuver transfers the bicycle's weight from a padded support to its own stand. The 26-inch steel cruiser reference has a 1,195 mm wheelbase and a 16.78 kg mass. The adjustable spring stand reference establishes the range of usable leg lengths and load. [Bicycle specifications](https://electra.trekbikes.com/us/en_US/bikes/electra-bikes/cruiser-bikes/cruiser-1-step-over/a/24420/) · [Kickstand product catalog](https://www.pletscher.ch/images/content/downloads/en/Catalogue.pdf)

## Reproduce the setup

Use meters with x away from the arms, positive y to the left and z up. The two arm bases sit on 240 × 240 × 600 mm rigid pedestals centered at x = −0.050 m and y = ±0.305 m. The deck is a 920 × 1,990 × 40 mm slab with its top at z = 0.44 m. Its visible parking rectangle has corners x = 0.10 and 0.74 m, y = ±0.90 m. The wheelbase midpoint starts at x = 0.39 m, y = 0, measured directly above the wheel line. Both 668 × 54 mm tires rest on the deck. The frame leans 5° away from the stand against a 65 × 235 × 68 mm rubber bumper. The bumper is centered at y = 0.102 m and z = 1.171 m with its near face at approximately x = 0.472 m.

Use a 300 mm side stand with its folded and deployed stops intact. The simulation's stand mount is 52 mm toward the arms and 245 mm toward the rear wheel from the wheelbase midpoint, 291 mm above the platform. The fork starts centered, the stand folded, and the arms in their ready positions. The demo holds the top tube while it swings the stand past the spring midpoint, then transfers load to the deployed stand and releases the frame. The spring torque approximation and tire geometry are generated in `task.py` and `bicycle.py`.

## Judge the trial

Film the bicycle and floor marks at 20 Hz or faster. A level on the frame or a camera aligned with an angle scale gives its tilt. Use the painted floor rectangle and a metric height reference next to the wheel line.

- Credit 0.25 when the upper tube is at least 10 mm clear of the starting bumper. Credit 0.5 if the stand also reaches its deployed stop.
- Credit 1.0 when the frame is at least 30 mm clear of the bumper, both tires and the stand shoe touch the deck, and the frame leans 8–14° toward the stand. All fingertips are at least 12 mm clear of the bicycle and have withdrawn to x < 200 mm. The bicycle stays below 12 mm/s for ten consecutive 20 Hz frames. The video must show three supporting contacts and no contact with another object.
- Give zero credit if either tire contact patch leaves the marked area, rises over 15 mm above the deck or the frame tips beyond 17° from vertical at any point.

## Run and reset

Use Python 3.12.16, Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and Isaac Lab-Arena 0.3.0 at `c7edf3a9d5`. The [YAM USD assets and meshes](https://github.com/kairosiann/yam-isaac-lab-arena) use the MIT license. The bicycle, supports and markings are procedural. The three views are the wider overhead RGB view and the two standard wrist RGB views. The overhead optical center is at (−0.46, 0, 2.45) m, aimed at (0.39, 0, 0.76) m. The official model adapter also receives robot joint and gripper readings. The fixed demo issues scripted robot motions without reading bicycle state.

```bash
source "$ARENA_ENV"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 500 \
  --store-frames --log-dir demo --scorer task_score \
  --instruction 'Park the bicycle on its kickstand and leave it standing independently.'

python -B preflight.py
python -B camera_view.py
```

The limit is 500 steps or 25 seconds at 20 Hz. The demo scores 1.0 at step 333 and took 76.5 seconds to run at seed 0. Dream-1 beta scores 0.0 in 400 steps at seed 0 and took 78.4 seconds. To repeat it, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false --max-steps 400 --log-dir model_run`. Maximum tire lift was 6.8 mm, and the minimum measured hand clearance during the released dwell was 103 mm. Scene recreation restored the starting poses, velocities, joints, score state and camera buffers. Replaying controls reproduced measured states at steps 1, 20 and 50.

The two wheel bearings, steering fork and side stand have native joints. The convex tire envelope and spring torque are approximations. The simulated contact check allows up to 9 mm of numerical tire compression at the deck. Its support test measures the lowest tire surface and the stand shoe. The lateral lean and three weight-bearing ground contacts remain physical. Cosmetics and the drivetrain do not drive the bicycle during the trial.
