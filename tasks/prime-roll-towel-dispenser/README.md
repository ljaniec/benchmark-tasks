# Prime a commercial roll-towel dispenser after a refill

Feed the loose leader from a seated fresh roll through the open dispenser. Leave a straight 50 mm or longer tail below the outlet and let go of the lever. The exposed tail is the check a custodian makes before closing the cover. O*NET lists supplying restrooms as a core duty of janitors and cleaners. The dispenser instructions describe placing the paper between the rollers and working the lever for initial feed. [O*NET duties](https://www.onetonline.org/link/details/37-2011.00) · [Dispenser loading instructions](https://tork-images.essity.com/images-c5/305/159305/original/83tr-hwrt-lever-dispenser-manual.pdf)

## Setup and reset

Use the standard two-arm YAM workbench. Mount the 318 mm wide, 324 mm high ABS housing on an 18 × 410 × 410 mm fixed mounting plate. The 195 × 310 × 6 mm open cover is below the feed bay. Fit a 203 mm wide, nominally 150 mm diameter hardwound paper roll on the spindle at x = 461 mm and z = 1065 mm. Positive x points away from the arms, positive y to their left and positive z up. The outlet lower lip is at z = 820 mm. Its front and rear edges are at x = 399 and 438 mm.

Set the idler arm and feed lever at their released stops. Center the roll on its spindle and lay the square, connected leader on the two removable loading rails. Their top surfaces are at z = 980.6 mm. The reset edge protrudes 15 mm past the front of the rails, at x = 311 mm and z = 981.5 mm. The sheet starts clear of the rollers. The two roller axes sit at x = 408 and 448 mm and z = 934 mm. The front 6 × 250 mm and rear 9 × 283 mm steel axles carry 39.6 mm diameter rubber bands. Pulling down the rectangular tab opens the idler to its 58° stop. The hinged carrier, preloaded spring and sprung idler bearing set the nip when the arm closes. The sprung feed lever is on the opposite side. The detailed centers, dimensions and material assignments are specified by the scene in `task.py`.

The reset restores the arm and dispenser poses, the web and roller joint states, velocities, score counters and the camera buffers. Use a new square leader or rewind the paper and cut it across a straightedge for a repeat on the physical fixture.

## Scoring a trial

Inspect the paper path from the open feed bay. Both edges must pass through the actual closed nip, with the loading arm within 3° of rest. Check that the free edge deviates by no more than 8 mm from square across the width. This earns 0.25. Measure exposed paper vertically below the outlet lip. Observe the paper moving during the lever stroke while both marked roller surfaces turn. Award 0.5 at 15 mm of exposed paper and 10 mm of lever-driven advance. Award 1.0 at 50 mm exposed and 35 mm of lever-driven advance, with both hands clear and the lever within 3° of rest for 0.5 seconds. An unthreaded web, torn leader or damaged setup earns zero. A ruler next to the outlet and slow-motion review of the two roller marks can verify the result.

## Run and reproduce

Point `ARENA_ENV` to the installed YAM/Arena environment script and run from the task folder. The scripted demonstration plays fixed waypoints. The Dream-1 adapter uses the three camera streams and standard robot joint and gripper readings. The scorer's geometry and contact audit are outside the model input.

```bash
source "$ARENA_ENV"
python -m unittest test_web_contact
bash run_demo.sh 200 smoke
bash run_demo.sh 1000 demo
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E "task=$PWD/task.py" -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 400 \
  --store-frames --log-dir model_run --scorer task_score \
  --instruction 'Start the new roll in the open paper-towel dispenser. Feed a straight, unbroken tail through the outlet and leave the lever released.'
```

Use seed 0, object jitter zero and 20 Hz. The task allows up to 1,200 controls, or 60 seconds of simulated time. The scripted waypoint sequence lasts 46.95 seconds and the bounded Dream-1 command allows 20 seconds. The overhead and two wrist RGB streams are 320 × 176 at the policy interface. After closing and releasing the pressure arm, the demo brings the right wrist to a side view beneath the outlet while the left arm finishes the feed.

The recorded demonstration scored 1.0 in 905 controls, 45.25 seconds of simulated time and about 280 seconds of simulator wall time including reset verification. The outlet had 51.4 mm of exposed towel. The web advanced 184.2 mm during lever motion and simultaneous counterrotation. Native contact records show both gripper pads touching the paper during the pickup and both rollers touching the web during feed. The right wrist shows the tail below the outlet. The repeat reset passed for physical state, counters, timestamps and the three RGB buffers.

Dream-1 beta completed the bounded 400-control trial and scored 0.0 on the same executable source. The episode covered 20 simulated seconds and took about 137 seconds of wall time. The demo and Dream videos show the delivered RGB streams at 20 frames per second. The setup image is the first delivered overhead frame.

The tested environment used Python 3.12.16, Isaac Sim 6.1.0.0, IsaacLab 17.0.2, the PhysX backend 6.0.0, Inspect Robots 0.60.0, the Inspect Dream adapter 0.1.0 and YAM Arena 0.1.0. The Arena checkout was at `c7edf3a9d5`. The launcher saves Inspect logs and frames, the measurement trace, source hashes and repeat-reset evidence. To replay the numerical score, read `results.metrics.task_score` in the Inspect JSON and compare it with the terminating row of `trace.jsonl`.

## Simulation approximations and license

The roller transmission is torque-limited and advances from the physical lever stroke. The lever applies no drive on its return. Paper is a connected set of colliding panels with bending hinges and a 1.4 mm effective collision thickness. The roller advances the sheet through contact while the idler presses against it. Paper stiffness, rubber friction, the lever ratio and bearing loads are estimated. The web can bend along the feed direction. Twist, in-plane wrinkling and tearing are omitted.

The external YAM robot USD and meshes are MIT licensed and used through [YAM Isaac Lab-Arena](https://github.com/kairosiann/yam-isaac-lab-arena), which carries the I2RT notice. Install the robot model separately. The procedural scene and supporting code use the MIT license. `mechkit.py` is reused from the repository’s MIT-licensed brake caliper task.
