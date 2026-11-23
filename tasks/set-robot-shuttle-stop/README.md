# Set and clamp a robot shuttle's mechanical stop

Set the original stop of a compact pneumatic slide and verify that the installed shuttle repeats at the specified extension. This is the physical sequence used to commission the precision end position on a DGSL slide. [Festo commissioning instructions, section 9](https://docs.rs-online.com/cf71/A700000009217788.pdf#page=3)

## Equipment and setup

Use a Festo DGSL-6-30-P1A type mini-slide, which has a 30 mm nominal stroke, an 8 mm piston, a 35 mm width, a 20 mm height and a 74 mm body dimension. Secure the slide and a 150 g aluminum workholding block to the pedestal as dimensioned in the scene. Its adjustable end has the installed M5 × 0.5 cushioning component, a 3 mm adjustment hex, a clamp and the elastic buffer with a metal stop. The slide points toward the right-hand working arm. Keep the separate 1.5 mm cushioning adjustment unchanged. The modeling uses a shaped 2 mm clamp key and assumes the stated measured access. [Product data](https://ftp.festo.com/public/pneumatic/SOFTWARE_SERVICE/DataSheet/EN_GB/543923.pdf) · [Dimension and tooling tables](https://media.festo.com/media/202794_documentation.pdf#page=29)

Use a mechanically rated source of clean compressed air, a shutoff ahead of a checked regulator, a visible 3.0 bar gauge, and a detented 5/3 exhaust-center VHEF-H-B53E-G18 manual lever valve. Fit a 40 × 32 × 34 mm rectangular slip-on grip and a clamped 56 mm extension to the original lever. The extension puts the grip center 112 mm from the pivot and preserves the valve’s 0.70 N·m actuation torque. Feed port 1 from the regulator. Run working ports 2 and 4 through M3 adapters and one-way flow controls to the slide's two chambers. Exhaust 3 and 5. Plumb the two lines so the green end of the lever advances the yoke. The orange end retracts it. Set the flow controls to 80 mm/s or slower under the installed load. The valve starts in its center exhaust position. [Valve catalogue](https://www.festo.com/media/pim/169/D15000100144169.pdf)

Use a fixed dial indicator with at least 0.01 mm graduations and align the green sector to a 26.00 mm reference. The accept band is 25.92–26.08 mm. Set the original released operating stroke to 26.125 mm. Before each run, loosen the clamp to its free-running position and back off a quarter turn from that datum. Start the slide retracted. Set a clean steel adjustment key in its saddle with the bit seated. Set the calibrated indicating key in the clamp socket and on its cradle. Use the 39 × 36 × 37 mm rectangular swivel grip so the gripper can turn it while the paired beam and pointer remain visible.

The coordinates below use millimetres and the standard YAM bench frame, with x forward, y left and height above the worktop. The slide extends toward negative y. The origin of the slide is (438, 77, 92). Use a 158 × 108 × 12 mm aluminum mount on a rigid pedestal. Bolt the 31 × 26 × 16 mm, 150 g workholding block to the 34 × 32 × 3 mm steel yoke.

| Item | Starting or mounting point, x, y, height | Dimensions and material |
| --- | --- | --- |
| Adjustment driver | Bit origin (427, −14.4, 109) | 3 mm steel hex, 4.4 mm diameter × 70 mm steel shank and 45 × 17 × 38 mm rectangular grip |
| Clamp driver | Bit origin (421.8, −7.1, 112.8) | 2 mm steel hex, 6 mm diameter × 49 mm steel shank and steel beam with a 64 mm grip radius |
| Driver supports | Adjustment axis on its rest at (427, −73.4, 109), wrench web rest at (446.3, −18.5, 169.3) | Rigid polymer saddles, approximately 17 × 16 mm and 12 × 13 mm at their respective contact areas |
| Adjustment driver parking stand | Grip center after parking (449, −150.4, 82) | Two fixed 15 × 17 mm polymer uprights supporting the grip and shank |
| Proof valve | Base at (340, 227, 0) | Approximately 93 × 29 × 42 mm aluminum body, extended metal lever and polymer grip |
| Regulator | Body center (385, 136, 36) | 42 × 36 × 71 mm metal body with 36 mm gauge face and 4 mm outside-diameter air lines |
| Stroke dial | Base at (307, 78, 88) | 56 mm face on a fixed steel and polymer support |

An existing DGSL slide can be used with a lower-cost detented 5/3 exhaust-center valve. The E.MC 4H230E-06 preserves the three positions and the 1/8-inch ports. Use a spring scale at the grip to confirm that its actuation stays at or below the reference lever's 0.70 N·m. Fit the marked 112 mm lever reach and use the same regulator and working-port arrangement. [Valve circuit and dimensions](https://www.festo.com/media/pim/169/D15000100144169.pdf) · [Alternative valve and listed price](https://www.tom-parker.co.uk/upload/files/literature/6.pdf)

A lower-cost bench fixture can preserve the working contact and access geometry with a 30 mm stroke, 8 mm bore Artec ODM 008 030 GS M cylinder behind an MGN7 miniature guide. Mount the guide and steel yoke on a 158 × 108 × 12 mm aluminum plate and put the cylinder below the plate, parallel to the guide, with a rod-end link to the yoke. Keep the moving yoke and payload at the scene locations. At the front, mount an M5 × 0.5 threaded steel stop in a split aluminum clamp with an M2.5 steel clamp screw and vertical 2 mm hex access. Use a 3 mm hex at the rear of the stop, and put a compliant 4.8 mm diameter pad on its nose with about 2.4 mm of compression before the steel face bears. Use the mounting, tool and stop dimensions in `slide_stop_geometry.py` and `slide_tools.py` to leave the exposed approach and proof region unchanged.

The supplier examples list the cylinder and alternate valve at about £92 combined before shipping and taxes. The guide, mounting stock, markings, tube, flow controls and gauge are additional. The alternative is a build specification. The measurements below are from the simulated reference layout. Before a physical trial, confirm the loose reset varies from the target, the reference clamp torque holds the sleeve, and the installed stop arrests the 150 g payload on two advances with air regulated to 3.0 bar. [Cylinder listing](https://aegstore.uk.com/category/pneumatic-cylinders/series-o/) · [Miniature guide specifications](https://hiwin.com/products/linear-guideways/mg-series/)

## Physical judging

First vent the fixture with the lever in the middle. Guide the yoke by hand to the target side of the gauge. Turn the original end-setting key until the metal sleeve meets the yoke against the elastic buffer. This contact within the stated band earns 0.25.

Tighten the separate clamp while the wrench is fully seated and clear of its saddle. Use the hardware's 0.20 N·m torque requirement. The calibrated pointer should sit in its 0.18–0.22 N·m band for 0.5 s while the output remains stationary. Withdraw both keys and stay at least 44 mm from the moving yoke reference. That earns 0.5. Mark the position of the tightened head for the proof check. A visible reverse rotation of more than 3 degrees clears the tightening witness and both proof strokes.

Before a physical trial, check the pointer with the bit held in a rigid socket. Pull tangentially at the marked 64 mm grip radius with a spring scale. The lower and upper band edges are 2.81 N and 3.44 N at that radius.

Advance twice with the valve. Retract below 5 mm between proofs. Keep the other hand and both tools clear of the moving yoke. Each advance must settle on the original metal stop within the marked band for 0.5 s. The second held advance earns 1.0. Blocking the carriage, operating it with a key seated, loading the wrench above 0.25 N·m in either direction, or bypassing the original stop earns zero.

The simulator uses the original screw contact and thread coupling, a slipping clamp resistance model, a spring beam and a load-limited drive controlled by the visible valve. The pressure regulator, air circuit and gauge have documented physical equivalents. Air flow, exact screw preload and volume changes are approximated. The external I2RT YAM arms and grippers are MIT licensed. [External asset license](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/LICENSE.i2rt)

## Run

Use the stock YAM Isaac Lab-Arena embodiment, seed 0, zero object jitter and three 320 × 176 RGB camera views. The task restores the arms, slide, screw and valve states, velocities, torque witness, episode history and camera buffers on reset. The scripted run follows predetermined poses. The model gets the same RGB views and stock robot joint and gripper readings.

```sh
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E "task=$PWD/mounted_slide.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 560 \
  --store-frames --no-prompt --grader none --scorer task_score \
  --instruction "Set the shuttle stop at the green mark. Tighten the clamp to the green torque band, withdraw the tools, and prove the position twice with the valve." \
  --log-dir rollout
```

## Evaluation results

Both runs use seed 0, zero jitter, three RGB views at 320 × 176, and a maximum of 560 controls at 20 Hz. The Dream-1 adapter also receives ordinary joint and gripper readings. Physics runs at 480 Hz. Recorded software versions are Inspect Robots 1.0.0, Isaac Lab 17.0.2, Isaac Sim 6.1.0.0 and PyTorch 2.11.0+cu130.

| Run | Terminal score | Controls | Wall time |
| --- | ---: | ---: | ---: |
| Scripted demonstration | 1.00 | 560 | 303.6 s |
| Dream-1 | 0.00 | 560 | 322.0 s |

For the model run, use the same command with `--policy dream -P model=dreamlabs/dream-1-beta -P record=false -P timeout_s=90` in place of `--policy yam_scripted`.

The wrench remained in the calibrated band for 0.55 s. The two loaded advances settled at 25.980 and 25.980 mm, with a separate retraction below 5 mm between them. The same-seed reset audit passed.
