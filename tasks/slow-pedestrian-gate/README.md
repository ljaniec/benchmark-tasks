# Slow a self-closing pedestrian gate and make sure it latches

Adjust the main speed valve, lift the hold-open hook and watch the gate close on its own. A slower sweep gives pedestrians more time to clear the opening. The installer can regulate the main sweep and the final closing phase separately. The 5-second minimum for the measured 90° to 12° sweep comes from §404.2.8.1 of the U.S. Access Board guidance. [Closer adjustment instructions](https://www.locinox.com/en-gb/resources/file?guid=0dc14b4b-bb1e-44e5-950f-808b03fe05a3) · [Door and gate closing speed](https://www.access-board.gov/ada/chapter/ch04/#404-2-8-closing-speed)

## Physical setup

- Use a 910 × 980 mm welded steel leaf with a 40 × 40 × 2 mm perimeter and nine 20 × 20 × 1.5 mm pickets. Bolt the 75 mm square hinge, strike and holdback posts to the base. The gate weighs approximately 16.7 kg. The hinge is at (0.220, −0.330) m in the layout and the closed leaf lies along positive x. Its centre plane is y = −0.374 m. The two YAM base centres are (0, +0.305, 0.750) and (0, −0.305, 0.875) m. Positive z points up and x away from the arms. The layout source specifies the supports and tool rack.
- Fit the Locinox TIGER and PUMA kit (P00008665) and the National Hardware V29 N342-600 gravity latch with its 9.5 mm strike. Use the 3 mm top-entry socket for the main speed. Secure a steel hold-open hook to a fixed post with its eye 615 mm from the hinge. Fit a fixed 22 × 27 × 26 mm flat-sided grip tab to the hook. Tune the keeper position so it physically holds the stopped gate at 90°. Check that a lift clears the hook and the released leaf can move through the complete sweep. [Closer kit](https://www.locinox.com/en-gb/gate-closers/compact-hinge-and-gate-closer-in-one-p00008665) · [Latch drawing](https://nationalhardwarestorage.blob.core.windows.net/documents/nh_td_v29_v1261.pdf)
- Fit a Wera 3 × 25 mm hex bit (05056315001) and 152 mm bit holder (05053458001) with a rigid flat-sided grip. Set the grip centre 203 mm above the bit tip. Check sleeve clearance with the tool seated before positioning the rack. The simulated driver has a 26 × 15 × 40 mm grip and a 6.35 mm shaft. Park it tip-down at (0.164, −0.222, 0.762) m before each trial. [3 mm bit](https://hybris-media.wera.de/download/pdfgenerator-datasheets/en/05056315001.pdf) · [Bit holder](https://www.wera.de/en/tools/899-4-1-universal-bit-holder/)

Set the main speed valve to the fast witness and leave the separate final closing valve preset. For a physical fixture, establish the fast witness with three successive 90° to 12° sweeps of 1.75–1.95 seconds. Record the valve position and return it to that witness between trials. The model starts with the gate at 90° and stationary, the hook engaged and the strike aligned. The reset restores both robot arms, tool, joints, velocities, camera images and scoring history.

## Judge the released result

Use fixed 90° and 12° angle marks and a camera view of the pawl. Start timing at the visible release of the hold-open hook. Arms and tools remain clear of the moving leaf. Measure elapsed time to the 12° mark.

- Award 0.25 for a sweep of at least 4.0 and less than 5.0 seconds if it latches within two more seconds.
- Award 0.5 for a sweep of 5.0–8.0 seconds.
- Award 1.0 when the latch also catches within the next two seconds and stays engaged for 0.5 seconds, with the leaf within 0.5° of the closed mark.
- Award zero when the timing and release conditions do not qualify. Movement of the fixed fixture and robot or tool intervention in the timed sweep invalidate the observation. The tool tip stays inside the stated work envelope. The tool may remain in the right gripper clear of the leaf once withdrawn.

The native retention check for the selected fixture unloaded the closer after the gate latched and applied 5 N·m opening torque for 0.5 seconds. The tuned gate opened 0.62°, confirming physical engagement. The scored run uses the visible sweep and latch. A real fixture can be checked separately with a spring scale and measured lever arm.

## Run and score

Set `ARENA_ENV` to the installed Isaac Lab Arena environment script. Use seed 0, no object jitter and 20 Hz. The task limit is 1,000 control steps, or 50 simulated seconds. The released demo uses fixed calibrated robot waypoints. The controls do not read live object poses or scores. Dream-1 receives the standard joint and gripper readings plus overhead and wrist RGB at 320 × 176.

```bash
source "$ARENA_ENV"
python -m unittest test_closing
bash run_demo.sh 240 smoke
bash run_demo.sh 675 demo
inspect-robots run --embodiment yam_arena --policy dream \
  -P model=dreamlabs/dream-1-beta -P record=false \
  -E "task=$PWD/inspect_gate_task.py" -E control_hz=20 -E object_jitter=0 \
  -E cam_width=320 -E cam_height=176 --seed 0 --max-steps 600 --store-frames \
  --scorer task_score --log-dir dream \
  --instruction "Slow the gate's main closing speed, release the hold-open hook, and let the gate swing into its latch on its own."
```

Read `results.metrics.task_score` in the Inspect JSON. The demo launcher records native joint states and contact checks separately from the policy and verifies that the physical state, counters, timestamps and RGB buffers replay at reset.

The full-score demonstration completed 441 controls, or 22.05 simulated seconds. The overhead view shows the hook release at 14.15 seconds, the rail reaching 12° at 20.20 seconds and the latch seated at 21.60 seconds. The recorded unassisted sweep took 6.05 seconds. The right wrist shows the recessed valve and driver. The left wrist shows the release tab after its half-second occlusion while passing the upper gate rail before the grasp. Reviewed camera samples include the policy input at 320 × 176 and the native render at 640 × 360.

The demonstration and reset audit ran in approximately 143 seconds on an L40S. The completed 600-control Dream-1 trial took 146 seconds and scored 0.0. The recorded environment used Python 3.12.16, Isaac Sim 6.1.0.0, Isaac Lab 17.0.2, Isaac Lab PhysX 6.0.0, Inspect Robots 0.60.0, Inspect Robots Dream 0.1.0 and YAM Arena 0.1.0. The runtime checkout was `c7edf3a9d5`.

## Fidelity and assets

The installed leaf, recessed hex valve, hold-open hook and latch are colliding native parts. The magnitude of the regulated closing torque depends on the valve position and leaf velocity. Its fast and slow phases are passive. Estimated valve drag, damper response and material values stand in for measured internal hydraulics. The faceted cam preserves the strike's lift and retention operation. The short key and socket chamfers are modeled contact surfaces.

The procedural gate, driver and fixture use the MIT license. The external [YAM robot USD and mesh assets](https://github.com/kairosiann/yam-isaac-lab-arena/tree/main/yam_arena/assets/yam) are MIT licensed and installed separately through YAM Isaac Lab Arena. Keep the upstream I2RT license notice. The task uses the MIT-licensed `mechkit.py` support from the benchmark examples.
