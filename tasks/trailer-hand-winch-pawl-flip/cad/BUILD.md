# Trailer hand-winch pawl-flip: real-hardware build

Assemble under 10 minutes per fresh trial. Simulation uses the nominal coordinates below; built hardware within the stated tolerances matches the scoring bands.

## Parts

- Fulton T1100 hand winch (Part #142101), 1,100 lb, 4.1:1, 178 mm crank with Comfort Grip sleeve, two-way ratchet pawl with neutral payout (Fulton #0133306S01), 50 mm yellow polyester strap.
- Fabricated 4.8 mm steel trailer winch riser post (`winch_riser_post_v1.step` / companion `.stl`).
- MGN12H linear rail and carriage, 200 mm.
- McMaster-Carr 9654K303 steel extension spring (10.3 lbf/in = 1,804 N/m, 63.5 mm free length, 19 mm OD).
- Printed or machined: bow-load sled (`bow_load_sled_v1.stl`), green bow-stop chock (`chock_stop_v1.stl`), red payout target bracket (`target_pointer_bracket_v1.stl`), color-coded pawl pocket bezel with through mouths (`pawl_pocket_bezel_v1.stl`), rail pedestal (`rail_pedestal_v1.step` / `.stl`).
- 1:1 paper template: `pawl_mouth_template_v1.dxf` (tape on cheek, cut 14 x 14 mm apertures).
- Exploded torque BOM: `assembly_bom_v1.pdf`.

Low-cost path: Fulton winch on a 12 mm plywood deck (0.460 x 0.180 m) without cosmetic stand sheet metal, provided pinion, pawl pockets, and strap height above the table stay within +-2 mm of the table below. Purchasable without printing: only the pawl pocket bezel and target tab bracket must be printed or cut from the DXF template taped to the housing.

## Procedure

1. Bolt the Fulton T1100 to the riser post with the supplied 4.8 mm plate and four M8 bolts (25 N*m).
2. Snap or bond the pawl pocket bezel on the -y housing cheek so the scored green UP mouth centre is (0.1920, -0.0440, h = 0.2725 m) and the scored magenta DOWN mouth centre is (0.1748, -0.0440, h = 0.2666 m). Each scored aperture is a through 14 x 14 mm opening in the x-h plane at y = -0.0440 m. Pocket depth 8 mm. At theta_pawl = 0 deg / +38 deg the blue paddle centre lies within 1.0 mm of the matching mouth centre in the x-h plane. Or print the DXF 1:1, tape, and cut the mouths.
3. Mount the MGN12H rail on `rail_pedestal_v1`, hang the 9654K303 spring from the sled rear lug to the pedestal anchor, and set sled rest at x_sled = 0.4800 m with 0 N spring force.
4. Route the 50 mm webbing through the drum per the Fulton manual, anchor on the sled clevis at (0.4150, 0.0000, h = 0.0795 m), wind until the black tape stripe sits at x = 0.3400 m with 45.1 N tension (x_sled = 0.4550 m).
5. Bolt the red target bracket so both tabs flank the strap at x = 0.3650 m; press a 14 mm wide black vinyl stripe on the strap at payout zero.

## Post-build verification (one-time per lab)

- Magenta mouth vs drawing: <= 1.0 mm.
- Paddle-centre-to-mouth at +38 deg: <= 1.0 mm in x-h.
- Stripe-to-tab alignment at 25 mm payout: <= 2.0 mm.
- Chock contact at full payout: <= 1.0 mm gap.

## CAD binaries

Shipped in this folder (regenerate with `python gen_cad.py` if needed):

- `winch_riser_post_v1.step` + `.stl`, `rail_pedestal_v1.step` + `.stl` (faceted solid envelopes)
- `bow_load_sled_v1.stl`, `chock_stop_v1.stl`, `target_pointer_bracket_v1.stl`
- `pawl_pocket_bezel_v1.stl` (through green/magenta mouths, not filled blocks)
- `pawl_mouth_template_v1.dxf` (1:1 mm paper template)
- `assembly_bom_v1.pdf` (exploded stack + torque table)

Bezel mouth centres match green (0.1920, -0.0440, 0.2725) and magenta (0.1748, -0.0440, 0.2666) within 0.1 mm in the generator.
