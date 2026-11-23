# Synchronize two pneumatic stops in a robot workcell

Start the diagnostic cycle, adjust the slower stop's original speed control, lock it and test again. The two stops must arrive together on two consecutive verification cycles. An independently metered stop can reach a fixture late after servicing. Comparing timing and adjusting the original setting is part of bringing that workcell back into operation. The Polish technician practical exam provides a related example of diagnosing two pneumatic actuators by their timing. [CKE practical example](https://bip.cke.gov.pl/attachments/download/9186)

## Setup and reset

Use two Festo ADN-32-80-A-P-A cylinders with 32 mm bores, 80 mm stroke and 12 mm steel rods. Fit each branch with a 6 mm tube and an SMC AS2201FS-G01-06 meter-out valve. Use the same regulated supply for both branches and keep the rods behind clear guards during testing. The cap diameter is 12 mm, the collar diameter 20 mm and the movement between the locked and unlocked positions 1.4 mm. Protect the reference valve. Each rod carries a 14 × 33 × 17 mm rigid polymer flag. Use yellow for the reference and red for the adjustable branch. The green test button has 4 mm of available motion. [Cylinder datasheet](https://ftp.festo.com/public/PNEUMATIC/SOFTWARE_SERVICE/DataSheet/EN_GB/536277.pdf) · [Valve dimensions and curve](https://ca01.smcworld.com/catalog/New-products-en/mpv/es20-229-as-fs/data/es20-229-as-fs.pdf)

Measure x forward and y to the robot's left, from the midpoint of the evaluating arm bases. Center the 408 × 428 × 34 mm aluminum plate at x = 420 mm and y = 0 on the 747.5 mm high worktop, with 15° yaw counterclockwise viewed from above. The rest of the coordinates below are in the rotated plate frame, with z measured from the bottom of the plate. The rods extend along local −x. The flag centers are (−73, ±109, 72) mm at reset. The 4 × 38 × 2 mm yellow finish stripes are centered at (−153, ±109, 36) mm. The simulated aluminum cylinder envelopes are 138 × 51 × 62 mm at (93, ±109, 67) mm. The adjustable cap is centered at (−60, 165, 102) mm. The reference cap is at (−60, −165, 102) mm inside a 22 mm diameter, 24 mm high cover. The 18 mm diameter green button projects above a 61 × 56 × 42 mm polymer box at (−115, −175, 55) mm. Each rod lane has a 184 × 46 × 6 mm polycarbonate shield centered at (−86, ±109, 121) mm. A 138 × 283 × 8 mm shield at (62, 0, 140) mm covers the cylinder bodies. The 6 mm blue polyurethane tubes run from each valve toward its cylinder without crossing the exposed flag path.

Set the reference dial to four turns from closed and the fault dial to three and a half turns from closed. Lock both, retract the rods and release the test button. Confirm the slower starting condition in a supervised test. Return the arm joints, part joints, velocities, controller and scorer counters, and the camera buffers to the declared start between trials.

## Physical judging

Use the video frame showing test-button actuation as the time origin. Each flag has arrived when its center is within 3 mm of the center of its visible finish stripe. Use the 80 mm spacing from the starting center to the stripe as a scale. Award 0.25 after one completed test with both arrivals within 0.20 s of each other. Award 0.5 after a test with both arrivals within 0.10 s. Award 1.0 when two consecutive tests meet the 0.10 s limit and both rods have been home with the test button released and cap locked for 0.5 s. A valid cycle reaches the finish area in 0.8 to 1.5 s and stays locked throughout. Touching the guarded rods or altering the protected branch invalidates the trial.

The simulation uses the manufacturer-sized cap and a procedural spline to represent its lock. The physical dial and button operate through contact. A steady-flow estimate sets two force-limited native cylinder drives. It uses the manufacturer's related 1/8-inch valve curve at 0.5 MPa and the rod-side cylinder area. The valve curve, chamber pressure and native drive response are approximations of the assembled pneumatic circuit. The timing rubric judges the actual flag arrivals.

The external I2RT YAM robot asset includes the arms and grippers under the MIT license. The rest of the task uses procedural geometry and the stock YAM Isaac Lab-Arena embodiment. [YAM asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run and score

Activate the installed IsaacLab-Arena environment. Run from the task folder using seed 0, no object jitter and a 410-control limit at 20 Hz. The terminal Inspect metric is `task_score`.

```sh
python -m pytest -q test_diagnostic_cycle.py
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 410 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Run the test cycle. Adjust and lock the slow stop's speed control, then show that the two stops finish together twice." \
  --log-dir rollout
```

The demo uses fixed arm targets. For the model evaluation, use `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` with the same task options. Dream-1 receives three RGB camera streams and the normal joint and gripper readings. To save both RGB sizes and check a matched-seed reset, set `PAIRED_STOPS_AUDIT_DIR` to an existing output folder and run the same scripted arguments through `python audit_paired.py run`.

## Verified runs

Both runs used seed 0, zero object jitter, three 320 × 176 RGB streams and 20 Hz control. Native physics ran at 480 Hz. The runtime used Inspect Robots 0.60.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu128.

| Policy | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 385 | 135.7 s |
| Dream-1 | 0.00 | 410 | 133.8 s |

The diagnostic arrivals differed by 0.25 s. With the original cap relocked after adjustment, the two verification cycles differed by 0.00 s and 0.00 s. Reset was checked after the verified run. Fixture and arm state, image timestamps, camera buffers and episode history returned to the matched start.
