# Guillotine latch-before-clear: real-hardware build

Product class: 12 inch (305 mm) guillotine paper trimmer with a spring-raised blade arm and a latch at the handle end of the arm. This folder describes a custom test rig modelled on that class, not a copy of one named unit: its latch is a slide plate on a guide block with a thumb knob (a real unit may use a stamped flip clip), and the spring (about 3.3 N at the handle, a value chosen for the YAM fingers) and the slide resistance are the rig's own. The card stack is about 180 g (banded card stock), the same as the simulated mass. Regenerate the parts with `python gen_cad.py` (numpy only). STL units are mm; each part sits with its footprint centred on the origin and its bottom on z = 0.

## Bill of parts

| Part | STL | Size (x, y, z mm) | Material | Color | How made |
|---|---|---|---|---|---|
| Trimmer base | `trimmer_base_v1.stl` | 355 x 450 x 30 | MDF or ABS board | white (printed 2 mm grid lines optional) | cut from sheet, or printed in tiles |
| Blade bar | `blade_bar_v1.stl` | 10 x 305 x 20 | steel flat bar | silver | cut from flat bar, drill the pivot at the left end |
| Handle block | `handle_block_v1.stl` | 24 x 24 x 40 | ABS or PLA | black | 3D printed, bonded to the front end of the bar |
| Latch plate | `latch_plate_v1.stl` | 36 x 16 x 5 (length, width, thickness) | PLA or ABS | red | 3D printed, slides in x on the guide block |
| Latch knob | `latch_knob_v1.stl` | 12 x 16 x 30 | PLA or ABS | red | 3D printed, bonded on the rear of the plate |
| Latch guide block | `latch_guide_v1.stl` | 45 x 24 x 50 | aluminium or ABS | grey | machined or printed, screwed to the base, carries the plate slide |
| Pivot housing | `pivot_housing_v1.stl` | 40 x 30 x 35 | ABS or aluminium | grey | printed, carries the blade arm pin and the torsion spring |
| Rail | `rail_v1.stl` | 12 x 450 x 20 | aluminium strip | grey | cut to length, screwed to the base |
| Card stack | `card_stack_v1.stl` | 130 x 26 x 70 | cut cards, about 180 g (banded card stock) | blue | cards stacked on edge, wrapped in blue tape or card stock |
| Tray | `tray_v1.stl` | 220 x 160 x 40 outside, 4 mm walls | ABS or PLA | yellow | printed in sections or bent from sheet |

## Assembly

Station frame in mm, x away from the robot base side, y along the trimmer, h is height above the table top (the base top face is at h 30).

1. Base: x 170 to 525, y -245 to 205. Rail on the base at x 351, full length, centred 10 mm above the base top. Screw both down.
2. Pivot housing: centre x 330, y 189, standing on the base top (35 mm tall).
3. Blade arm hinge pin: axis along x at (x 330, y 175, h 45 above the table top, i.e. 15 mm above the base top), at the left end of the bar. Arm travel 40 deg: 0 deg is raised, 40 deg is flat on the base.
4. Blade bar: spans y -130 to 175 (305 mm), centred on the shear line x 330. Handle block bonded on the front end at y -142, centre 10 mm above the bar axis; its top is at h 75 when the arm is flat.
5. Spring: torsion spring on the hinge pin, 1.5 N m/rad, about 3.3 N felt at the handle when the arm is flat. Use a damped return so the arm stays up when released.
6. Latch guide block: centre x 277.5, y -142, standing on the base top (50 mm tall, top at h 80), footprint 45 x 24.
7. Latch plate: 36 x 16 x 5, underside at h 80 (5 mm above the flat handle top at h 75), centred on y -142, rear end at x 274 at rest (front end x 310, handle starts x 318). It slides in +x on the guide block, travel 32 mm. The plate should stay where pushed (light friction or a detent; the simulator uses a joint-friction setting of 1.5, not a measured force). Thumb knob 12 x 16 x 30 on the plate rear.
8. Plate overlap: at 40 percent of travel (13 mm push) the front end reaches x 323 and covers the handle by 4.8 mm; this counts as engaged. At full travel (32 mm) the front end reaches x 342 and covers 24 mm.
9. Card stack: standing on edge at x 245, y 120 on the cutting bed (inside the danger zone, which runs from the base front edge to 100 mm behind the shear line, along the whole blade span). Gripper width 26 mm.
10. Tray: centre x 280, y 300, off the base on the table.

## Checks

- Arm flush when within 3 deg of flat (joint value 37 deg or more).
- Slide latch pushed at least 40 percent of its 32 mm travel (13 mm) counts as engaged.
- Plate to flat handle gap 5 mm, overlap 4.8 mm at 40 percent, 24 mm at full travel.
- Guide block does not touch the arm in any pose.
- Stack and tray clear of the arm and each other; stack fits inside the tray (inner 212 x 152 mm; stack 130 x 26 x 70 mm).
