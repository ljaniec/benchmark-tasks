# Stop a monitor arm from sinking

Rebalance a mounted display whose arm sinks after release. The arm should support the display at two different working heights. A commercial desk mount for 13–32 inch displays uses a 6 mm hex key to adjust its counterbalance while the display is supported. The scene preserves the installed desk mount, screen weight, lift pivot and rear tool access. [Manufacturer specifications](https://vivo-us.com/products/stand-v001q) · [Assembly instructions](https://www.kbauthority.com/files/PDF/VIVO/STAND-V001Q%20Assembly%20Instruction.pdf)

## Set up the station

Use a rigid 1,100 × 1,300 × 60 mm tabletop. Its top is at z = 747.5 mm in the simulator. The x-axis points from the robots into the work area, y to the robots' left and z up. Center the desk at x = 360 mm, y = 0. Place the bases of the parallel-gripper arms at x = 0, y = ±305 mm and z = 750 mm.

Use a desk grommet and steel post with a 300 mm aluminum lift link. Put its pivot 287.5 mm above the tabletop at x = 410 mm and y = −64 mm. Rotate the post assembly −35° about vertical from a link pointing along positive y. Carry a 4.2 kg unpowered display with a 535 × 305 × 35 mm plastic-bezel housing, glass face and steel VESA plate. Its center is 300 mm from the lift pivot. The link weighs about 0.6 kg and the screen stays upright while the arm moves. Set the arm to its 21° lower stop with the counterbalance too weak to hold the display at either test height. The simulation starts with a 3.33% preload deficit at horizontal.

Stand a 6 mm steel hex tool tip-down in a fixed ABS holder at x = 260 mm, y = −223 mm. Use a 24 × 30 × 25 mm rectangular grip. The tool tip starts about 44 mm above the tabletop. The holder has a 36 × 36 mm footprint and a tapered mouth around its bottom seat. Keep the tabletop clear of other support objects. Before each trial return the arm, key and both robots to their start states and let the scene settle at the lower stop.

## Judge the trial

Measure the vertical position of the same point on the screen bezel at each hold. The horizontal arm reference is the center height of the screen with the lift link level. Use a ruler and video sampled at 20 Hz or faster. The two test bands are 62 ± 12 mm below and 62 ± 12 mm above that reference. During a hold the range from lowest to highest reading may be at most 5 mm.

The robot must withdraw both hands to x < 285 mm and leave the key resting tip-down in its holder. The screen and arm may touch only their installed mount. Score 0.25 for a 0.5-second hold in either band, 0.5 for a 2-second hold, and 1.0 for a 2-second hold at each height. The final hold must remain valid when the trial ends. Give no credit if the tool leaves the work area or the hardware is damaged. Match the software limit by rejecting the key center outside x = −120 to 700 mm, y = −610 to 610 mm or more than 40 mm below the desk surface.

## Run and score

Use Python 3.12.16, Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and Isaac Lab-Arena 0.3.0 at `c7edf3a9d5`. The external YAM robot USD assets and meshes use the MIT license. The mechanism, key and holder are procedural. [YAM simulation assets and license](https://github.com/kairosiann/yam-isaac-lab-arena)

```bash
source "$ARENA_ENV"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/scene.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 720 \
  --store-frames --log-dir demo --scorer task_score \
  --instruction 'Adjust the sinking monitor arm so it stays put after release at two working heights.'

python -B camera_view.py
```

The limit is 720 steps or 36 seconds at 20 Hz. The scripted robot uses the fixed fixture dimensions and authored waypoints. It does not read object state. The Dream-1 adapter receives the overhead and two wrist RGB streams, alongside robot joint and gripper readings. The overhead mount is at (−0.40, 0, 2.12) m aimed at (0.40, 0, 0.95) m. Run the model with the same scene, seed and observation flags using `--policy dream -P model=dreamlabs/dream-1-beta -P record=false`.

The demo scores 1.0 at step 627 and took 124.6 seconds to execute. The hex key drove the adjuster about 74°, then returned to its holder. The unsupported display held −59.8 mm and +68.1 mm for two seconds each. Dream-1 beta scores 0.0 in 720 steps and took 122.6 seconds. Both runs use seed 0. The right wrist view shows the key and socket during insertion and turning. The monitor fills much of the left wrist while held. The overhead camera shows the workspace and both released heights.

The key and adjuster have physical collision surfaces. The 6.32 mm socket and tapered entry allow clearance for the beveled 6 mm bit. The gas spring is an applied joint torque based on the lift geometry and the actual adjuster rotation. Its preload slope, friction and thread pitch are simulation approximations. Cables and unused swivel axes are omitted. The same-process reset audit compares poses, velocities, joints, score state, camera timestamps, pixel buffers and replayed controls.

Run the reset audit with the same simulator command and replace `inspect-robots run` with `python audit_native.py run` in a fresh output directory. The arm and fixture states reproduced exactly on reset and after the first replayed control. Renderer variations accounted for at most 1.31% mean absolute RGB change at reset. On the archived images the camera check accepted all repeated views and rejected a stale end-of-run frame and a half-stale blend in each view.
