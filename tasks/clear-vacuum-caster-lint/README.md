# Clear a robot vacuum’s jammed front caster

Hair and lint can stop a vacuum's front caster from rolling and can mark flooring. This task uses the documented maintenance sequence of a Roomba-style caster. Remove the module, clear the wheel and axle, refit the original assembly, and verify that the wheel rotates freely. [Caster service procedure](https://homesupport.irobot.com/articles/en_US/Knowledge/19952) · [Chassis and product dimensions](https://www.irobot.com/en_US/roomba-i3plus-evo-self-emptying-robot-vacuum/I355020.html)

## Scene and reset

Place the powered-off vacuum upside down and secure it to a bench without covering the caster opening. The simulation fixes the approximately 337 mm polymer underside in that position. The caster uses a 33 × 17 mm rubber wheel, 4 × 31 mm steel axle and polymer fork with two bearings. Put the mat and trays in the locations in `task.json` and `setup.png`. The 77 × 44 mm keyway holder supports the removed fork during service. Its returned edge retains the nominal 6.9 mm stem below the holder face while allowing the stem to slide out along the keyway. The 49 × 53 mm wheel rest has parallel rails 26 mm apart at their centers and 45 mm long. These supports remain passive throughout the trial.

Fit the original wheel and fork before loading the fault. Press a 2 g folded lint sample into the right hub clearance, with the loose end exposed above the fork. Record the folded orientation and depth against the start image. The simulated wad is 10 × 1.8 × 10 mm with a flexible face initially compressed 2.04 mm. Return the trays to empty, place both arms in their ready poses and use seed 0 with object jitter disabled. The reset rebuilds the simulation scene and restores the initial body and joint states, velocities, score history, timestamps and camera buffers.

## Run and score

The reference runtime is Python 3.12.16, Inspect Robots 0.60.0, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2 and Isaac Lab-Arena 0.3.0 at revision `c7edf3a9d5`. The external YAM USD and mesh assets use the MIT license through the [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena). The vacuum and fixtures use procedural geometry. The task needs no external asset download beyond the robot installation.

```bash
source "$ARENA_ENV"
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 1200 \
  --store-frames --log-dir demo --scorer task_score \
  --instruction 'Clear the material jamming the front caster, refit the original parts, and show that the wheel turns freely.'

python -B -m unittest -v
```

The step limit is 1,200 at 20 Hz, or 60 seconds of simulated time. The scripted recording scores 1.0 at step 1,175. It took 232.6 seconds to run and shows 24.7° of released rotation across ten frames. Repeat reset reproduced the initial physical state and each camera buffer. Replaying controls reproduced the measured state at steps 1, 20 and 50. The bounded Dream-1 beta recording scores 0.0 in 400 steps at seed 0. Its run took 103.8 seconds. To repeat it, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false --max-steps 400 --log-dir model_run`. The fixed demonstration uses robot actions only. The model adapter receives three RGB images and the robot's joint and gripper readings. `proof.py` tracks the observable service milestones and the release interval. The supplied test takes approximately four minutes to simulate on an NVIDIA L40S.

## Physical judging

Use a millimeter scale on the work surface and a fixed 20 Hz or faster video showing the caster mount, holder and wheel. Put six paint marks at 60° intervals around the tread. Use one distinctive mark plus five lighter marks to keep at least one visible as the wheel turns. Use the same visible mark across the ten release frames, or track two adjacent marks with their 60° separation. Zero the score if any part is damaged, discarded or leaves the marked workspace. See `task.json` for the bounds.

- Score 0.25 after lifting the complete fork, wheel and axle 30 mm above the installed seat. A height scale beside the socket gives the reference.
- Score 0.5 if the lint is then visibly detached and comes to rest in the catch pocket or lint tray.
- Score 1.0 after reseating the original fork and both axle ends, flicking the tread, and withdrawing both hands. The fork is within 2 mm of its marked seat. Both axle ends sit in their bearings. Start the timer when all fingertips are at least 42 mm from the axle center. Over the next ten 20 Hz frames, expect at least 15° of net rotation and an initial speed of 20–172° per second. The ten frames must show seated parts and hands clear. The visible tread stripe measures angular advance and the scale measures withdrawal.

## Simulation approximations

The fixed underside keeps installed access and the original caster interfaces. Two spring clips retain the caster stem and two sprung ramps retain the axle. Real wheel and debris contact provide the drag. Detent stiffness, small retention dimensions and the lint spring use approximations. In the same fixed scripted component test, the clean simulated wheel advanced 17.49° in 0.5 seconds and the jammed wheel advanced 0.29°. The score uses a 15° threshold from that simulated comparison. For a physical repeat, measure the clean caster's release and coast before choosing the lint loading.
