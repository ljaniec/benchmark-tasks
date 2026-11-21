# Real-hardware build kit: hook-on laryngoscope task

Printable closed cradle plus the off-the-shelf parts for a physical copy of the task. Regenerate the STL files with `python gen_cad.py` (numpy only, millimetres).

## Parts

| Part | Spec | Notes |
|---|---|---|
| Handle | ISO 7376 hook-on, penlight class (e.g. Welch Allyn 60814 class), 18.3 mm barrel, about 149 mm overall, about 0.18 kg | Orange paint (painted for visibility, real handles are chrome or black), 128 mm barrel plus 3 mm end cap. Hinge pin 4 mm upright between two cheeks, lamp switch plunger in the head's stop face (4000 N/m spring), and an M4-M5 steel spring plunger (end force 10-40 N, e.g. Misumi/Kipp class) in the head pressing its ball down on the blade heel. On real ISO 7376 fittings the spring balls sit in the blade and the detents on the handle; here the plunger is on the handle head |
| Blade | Macintosh 3 fibre-optic hook-on class, about 142 mm overall, painted blue, about 75 g | Heel about 16 mm wide and 12 mm high, hook slot about 4.6 mm on the 4.0 mm pin (class-typical for ISO 7376 hook-on fittings), only a 1.5 mm per side chamfer over 3 mm at the mouth, heel back end 12 mm behind the slot bottom. The heel's -t prong needs an engage chamfer, a flat and a 1 mm step down (the lock) for the ball: machine or add it to the blade heel |
| Tray | Stainless, 260 x 380 x 20 mm | Tray outline is the judging boundary |
| `cradle_rail.stl` | 108 x 5 x 9 mm | One rail along the handle's far side |
| `cradle_rail_near.stl` | 114 x 5 x 9 mm | Rail on the blade side (pin frame x -139 to -25 mm), clear of the blade's swing from about 30 deg; holds the head against -y yaw |
| `cradle_stop.stl` | 27.3 x 5 x 9 mm (the 5 mm side along x) | End stop behind the handle's end cap |
| `cradle_front_post.stl` | 3 x 9 x 12 mm | Post just ahead of the head cheeks (pin frame x 6.5-9.5, y 15-24 mm), closes the cradle in +x |
| `cradle_assembly.stl` | far rail, near rail, stop and front post placed relative to each other | Pin-axis frame, for checking fit only |

Print in PLA or PETG, 100 percent infill on the thin sides, 0.2 mm layers. Fix to the tray floor with double-sided tape or a thin acrylic adhesive layer.

## Placement

Tray centred at x 0.250, y -0.085 m, 260 x 380 mm (x forward from the arm-base line, y left). Hinge pin axis at x 0.300, y -0.100 m, with the handle lying along x, head toward +x. The handle barrel axis is 16.5 mm from the pin axis along +y. The rail sits on the +y side of the barrel (the side away from the blade) with its centre 13.65 mm from the barrel axis, running 108 mm along the barrel. The end stop is centred on the barrel axis, spanning across the barrel, its centre 6 mm behind the barrel end (front face 0.5 mm behind the 3 mm end cap). The blade-side rail sits on the -y side of the barrel, its centre 13.65 mm from the barrel axis, 114 mm long (pin frame x -139 to -25 mm), clear of the blade's swing from about 30 deg. The front post stands ahead of the head cheeks. The far rail, near rail, end stop and front post together stop the handle moving in +x and turning, so no human hand is needed in a real trial; the handle can still be lifted out. The blade origin (slot bottom) is the reference point; the heel back end is 12 mm further along the blade. The blade starts lying on its side, heel at x 0.300, y -0.175 m (75 mm from the pin axis along -y), pointing toward -x, parallel to the handle.

## Tolerances

- Pin axis position: +-1 mm. Barrel axis offset 16.5 mm: +-0.5 mm.
- Rail and stop height 9 mm +-0.3 mm; far rail length 108 mm +-0.5 mm, near rail 114 mm +-0.5 mm. The rail inner face should touch the barrel or leave at most 2 mm gap (rail centre 13.65 mm from the barrel axis, stop 27.3 mm wide, as simulated).
- Blade start heel position +-2 mm, blade parallel to the handle within 3 deg.
- Rail and stop must not touch the head cheeks, pin or blade path: keep the rails and stop at least 20 mm behind the pin axis (x less than -22 mm from the pin); only the front post stands ahead of the head, clear of the heel's sweep.
- Check before each trial: lamp switch stands about 1 mm proud of the stop face and moves freely, hinge pin straight, detent plunger moves freely.
