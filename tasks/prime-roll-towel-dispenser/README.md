# Prime a commercial roll-towel dispenser after a refill

A fresh roll is already seated in an open wall-mounted dispenser. Position its loose leader at the roller nip and work the return lever until an intact, straight tail hangs from the outlet. The task ends at the feed check while the housing is open for inspection.

Restocking public washrooms is a core duty of janitors and cleaners. O*NET 37-2011.00 lists supplying restrooms and BLS OEWS counted 2,209,760 US jobs in this occupation in May 2025. Employment is reported for the occupation. The San Jamar Element loading sheet documents placing the paper in between the two rollers and operating the lever to feed it through before shutting the cover.

- [O*NET 37-2011.00](https://www.onetonline.org/link/summary/37-2011.00)
- [BLS May 2025 employment release](https://www.bls.gov/news.release/pdf/ocwage.pdf)
- [San Jamar Element paper loading instructions](https://cdnimg.webstaurantstore.com/documents/pdf/101838_elementinstruction.pdf)
- [Manufacturer's dimensional sheet](https://www.webstaurantstore.com/documents/specsheets/element-sell-sheet.pdf)

The [accepted twin-roll tissue task](https://tasks.robocurve.org/board?task=TK-22CFQJ) relocates a usable roll, installs a refill and selects the correct outlet. Its tissue tail starts threaded. The [accepted soap refill](https://tasks.robocurve.org/board?task=TK-Z3X5F5) works a latch and seats a rigid refill. This prototype tests the alignment and feed of a wide, loose paper web into a roller pair. An empty lever stroke leaves the outlet empty.

## Scene and approximation

The housing is a full-width unbranded reconstruction. See task.json for the reset, dimensions and directions and task.py for every collider. The thick mounting panel is secured to a stand behind the housing. The cradle and roll start fixed. The return lever is a native hinge with limits from −38 to 0 degrees, spring 1.5 N m/rad, damping 0.05 N m s/rad, mass 80 g and estimated dry drag 0.016 N m. The roller pair has fixed collision geometry.

The paper model represents the free edge by two sample points 150 mm apart and 40 mm up from the tip. A gripper can acquire a point only on closing around the thin web, aligned with its normal. Both edges have to enter the nip together, with at most 8 mm of height difference. Once seated, the modeled roller transmission advances the paper 84 mm per radian of downward lever rotation beyond 2 degrees of backlash. A stroke must return within 1 degree of the top before it can advance further. The roll does not unwind continuously under its own weight. The paper shape is visible but simplified, with no force feedback, buckling, roller deformation, measured transmission ratio or material tearing calibration. A pair of grippers pulling the samples out of their allowable separation produces an invalid outcome. These assumptions need a physical or simulation check before submission.

Use unheated paper and a secured dispenser. Mark the two inspection locations on the front of the rollers and keep the outlet in a low side camera view with a millimetre scale. Judge a straight captured edge before feeding and the connected paper below the outlet after the lever is back at its upper stop. The final exposed paper and lever state have to persist after withdrawal for 0.5 seconds. The shell remains open for this priming trial.

## Inspect commands

Use the official YAM/Arena simulation environment. Source the installed Arena environment, check the versions in the evidence log, and run from the task directory. The prototype plays fixed waypoint targets through yam_scripted, with no live object poses or scoring feedback. The evaluation uses overhead and two wrist RGB views and standard robot joint and gripper readings. The scorer's geometric samples stay within the simulation.

```bash
source /data/IsaacLab-Arena/yam.env
inspect-robots list embodiments
inspect-robots list policies
python -m unittest test_feed
bash run_demo.sh 200 smoke
bash run_demo.sh 1200 demo
```

The demo runner records frames at 320 × 176 and 20 Hz. Reset seed is 0 and object jitter is 0. The trial limit is 60 simulated seconds. Wall runtime, exact installed versions, grasp reliability, camera coverage, reset and final scores are pending simulation validation. The native gripper effort is configured at 40 N per finger for this standalone task process, and the scripted opening command is below 75 mm. Runtime readback is required.

The external YAM robot USD and meshes are MIT licensed and used through the official [YAM Isaac Lab-Arena adapter](https://github.com/kairosiann/yam-isaac-lab-arena), which carries the I2RT license notice. They are installed separately. The procedural dispenser geometry and simulation code use MIT. The local copy of mechkit.py comes from the MIT-licensed brake-pads-caliper example in the benchmark repository.
