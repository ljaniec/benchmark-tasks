# Park a mobile robot workcell on its feet

Press the two front leveling pedals to shift the parked robot off its wheels and onto the pads. Mobile robot stations can be rolled into place and stabilized for service. Levers that lower a separate load-bearing foot are commercially documented, as are mobile robot bases that use deployed feet. [FATH lifting-foot casters](https://www.fath24.com/en/Product-World/Quicklinks/Product-portraits/Swivel-and-Fixed-Caster-with-Lifting-Foot/) · [Omron robot stand, section 3.1.1](https://www.omron-ap.com/data_pdf/mnu/i629-e-02_robot_stand.pdf?id=3739)

## Setup

Use a level worktop 747.5 mm above the floor with x forward and y left from the midpoint of the two working robot bases. The parked arm weighs 8.0 kg, consistent with the lightweight industrial arm documented on a mobile base. The aluminum deck and bare robot weigh about 10.72 kg together. The mounted robot has a 130 × 140 mm footprint. Center the 430 × 390 × 6 mm deck at (480, 0) mm, its underside initially 82 mm above the worktop. Two rear steel supports at local (155, ±137) mm are on the worktop. The two front 73 mm square metal housings each contain a nylon 50 × 25 mm wheel and a 32 × 8 mm elastomer support pad. The pad centers are at (−155, ±137) mm. The wheel axles are 38 mm farther from the working arms. The front pads begin 1.2 mm clear. [Compact arm reference](https://www.ufactory.cc/products/ufactory-lite-6)

Under each front housing is an eccentric cam behind a rectangular pedal. The pad follower is on the same guided rod as the pad. Each steel cam has a 9 mm radius, 8 mm eccentricity and a stop 12° beyond the point of maximum deployment. Its pivot is 51 mm above the worktop and 55 mm closer to the working arms than the pad center. The pedal extends 87 mm from the pivot and starts 75° above horizontal. Its 41 × 40 × 14 mm rectangular grip is on the lever tip. A light return spring retracts each pad before the cam moves. The compact bench fixture reproduces the commercially documented pedal-to-foot sequence with dimensioned cam parts supplied in the scene source.

## Score and reset

Compare the final deck footprint with the starting position. A credited pad touches the worktop and its own wheel has at least 2 mm of free clearance, verified by a feeler or a side image. Hands must be clear of both pedals and the footprint. Award 0.25 for one supported corner and 0.5 for two. Award 1.0 when both conditions hold for 0.5 s with the parked base within 6 mm of the starting marks. Direct lifting, blocking or moving the chassis to create clearance, passing outside the parking limit, or tipping more than 15° earns zero.

Lift both stopped pedals, restore the base to the marks, and confirm the front wheels are fully supported while the corresponding feet have the stated 1.2 mm gap. The simulation resets the parked arm and base, wheel and pedal state, return springs, velocities, scorer and camera buffers. The host arm stays folded and mechanically secured for the trial. The cam profile and pad compliance are modeled approximations. Each cam moves its pad through physical contact and retains the load after passing over center. The external I2RT YAM arm and gripper assets are under the MIT license. [External YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run

Use the Isaac Lab-Arena YAM station. This fixed layout uses seed 0, zero jitter, 20 Hz control and a 210-control limit. The scripted path uses fixed poses with no privileged state. Dream-1 receives the three RGB views and stock robot joint/gripper readings. Inspect reports the terminal `task_score`.

```sh
python -m pytest -q test_scorer.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/mounted_pedal.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 210 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Press the two leveling pedals so the front wheels clear the worktop. Leave the robot base parked on its feet." \
  --log-dir rollout
```

## Evaluation results

Both runs use seed 0, zero jitter, three RGB views at 320 × 176, and a maximum of 210 controls at 20 Hz. The Dream-1 adapter also receives ordinary joint and gripper readings. Physics runs at 480 Hz. Recorded software versions are Inspect Robots 1.0.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu130.

| Run | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 201 | 95.5 s |
| Dream-1 | 0.00 | 210 | 88.0 s |

For the model run, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` in place of `--policy yam_scripted`.

Both front pads supported the parked base after the grippers withdrew. Each wheel had 6.15 mm or more of clearance at the end of the demonstration. The two wrist views show the released wheel and pad on their respective sides. The same-seed reset audit passed the physical state, counters, rates and camera checks.
