# Balance a motorcycle wheel after a tire change

On a free axle, the wheel rolls heavy-side down until its mass is balanced. Press a corrective strip onto each rim face, then use the fixed scale to check that the wheel stays near each of the four reference marks after release. This follows the two-sided weight placement in the service procedure for the represented front wheel. [Wheel service procedure](https://static.nhtsa.gov/odi/tsbs/2018/MC-10152625-9999.pdf#page=11)

## Reproduce the fixture

Mount the wheel on a level, horizontal passive axle with both ends supported. Use the dimensions, materials and initial tool positions in `task.json` and the included scene. The frame uses x away from the arms, y to their left and z up. The deck is about 92 × 82 cm. The two bolted YAM bases sit at y = ±305 mm and z = 750 mm and the axle center is at (380, 0, 830) mm.

For a physical trial, balance the mounted wheel using two equal 14.174 g adhesive-backed steel weights on opposite rim faces at the same angle, centered 232 mm from the axle. Use compact weights, nominally 17.2 × 15 × 7 mm, and confirm both the mass and the tape footprint on the prepared ledge. The simulated weights are rectangular steel blocks. Confirm that the wheel turns freely and remains near each reference mark after release. Remove the weights to create the repeatable imbalance, clean the rim and place two matched replacements on the magnetic applicators. Align the red index at 270° with the heavy side settled down. Use a scale or a known test mass to check that the stand can reveal an imbalance. Follow the selected adhesive maker's cleaning and application instructions. [Adhesive preparation](https://www.motionpro.com/a/wheel-weight-installation-instructions)

In simulation, a new episode reloads the fixture, arm and object states, camera buffers, tape contact model and score counters. The robot moves the same physically constrained wheel and weights throughout the trial.

## Judge the trial

Record the stand and wheel face at 20 frames per second or faster. Use the fixed degree scale to measure the center of the 43 × 22 mm red index. Place the overhead camera to see the index when the grippers hold the wheel at all four marks. The simulation records that view and both wrist views. Hold the index within 1° of the selected mark for half a second. For each counted release record the largest angle excursion from the release frame over the next 4.5 seconds. Keep both hands and the applicators at least 20 mm clear of the wheel, which must remain seated on its axle and free of external supports. Use short close views to confirm the lower edge of each installed strip lies on the flat rim ledge and its adhesive face sits flush.

A green index a quarter turn ahead of the red one gives the RGB demo another reference when the hands cover the red index during its approach. Use the red index and the degree scale to judge each hold and release. The camera stand remains outside the arm paths while the overhead optical view is repositioned.

- **0.25** for at least one correct strip that remains mounted through 20° of wheel motion, ending hands off with its applicator separated.
- **0.5** for both strips mounted and all four released checks at 12° maximum drift or less. The last completed release must remain valid when the trial ends.
- **1.0** for both strips mounted, all four checks at 5° or less and another ten valid controls after the last 4.5-second release.
- **0.0** if the axle lifts, tilts more than 15° or moves more than 5 mm, a part exits the service area, or the result fails all the above conditions. Measure the parts against the work area boundaries x = −25 to 910 mm, y = −430 to 430 mm and z = 415 to 1340 mm. Stop counting a release if a hand, tool or other object supports the wheel or the index is hidden.

## Run and score

Use Python 3.12, Inspect Robots 0.60, Isaac Sim 6.1 and Isaac Lab-Arena 0.3. The external YAM robot USD and meshes are MIT-licensed. The wheel, stand, trays and tools are procedural. [YAM simulation repository and license](https://github.com/kairosiann/yam-isaac-lab-arena)

```bash
source "$ARENA_ENV"
WHEEL_TRIAL_TRACE="$PWD/demo-score.jsonl" python run_wheel_demo.py --seed 0 --log-dir demo

WHEEL_TRIAL_TRACE="$PWD/model-score.jsonl" inspect-robots run \
  --embodiment yam_arena --policy dream -P model=dreamlabs/dream-1-beta -P record=false \
  -E "task=$PWD/inspect_wheel_task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 1600 \
  --store-frames --log-dir model --scorer task_score --no-prompt --grader none \
  --instruction "$(python -c 'import json; print(json.load(open("task.json"))["instruction"])')"
```

Use the standard credited Dream adapter and run limit. For either saved Inspect JSON log use `inspect-robots video` to render the synchronized camera views. The nominal scripted path uses 1,464 controls, or 73.2 seconds of simulated time. It can pause briefly for a readable index. A trial is capped at 1,600 controls, or 80 seconds at 20 Hz. The seed-0 demo scored 1.0 in 1,447 controls, with 247.62 seconds of wall time. The final release passed its extra half-second dwell.

The demonstration uses fixed fixture geometry for the press and chooses the tire turns from the overhead RGB index. The model gets the overhead and two wrist RGB streams and standard robot joint and gripper readings. The scorer compares visible index travel and geometric checks of the seated strips and hands-off clearance. A missing angle breaks the current hold or release and can be followed by a new attempt. Solver pressure and magnetic and adhesive reaction forces affect physics only. The tire collision shape is rigid and the forces model short-term tape adhesion.

The seed-0 Dream-1 beta trial scored 0.0 in 400 controls and took 110.72 seconds of wall time. Both trials used 20 Hz control, zero object jitter, and the three 320 × 176 RGB views. The Dream-1 adapter also receives its standard robot joint and gripper readings.
