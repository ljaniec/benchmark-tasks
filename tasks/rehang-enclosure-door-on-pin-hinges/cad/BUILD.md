# Rehang enclosure door on pin hinges: real-hardware build

One trial setup. Coordinates are table frame, mm: x forward from the arm bases, y left, z up from the table top.

## Parts

- Enclosure body and door: modelled on the nVent Hoffman INLINE LHC252015 wall-mount enclosure, 250 x 200 x 150 mm, 16 gauge (1.5 mm) steel, hinged door with removable hinge pins, door opens 180 degrees (Hoffman catalog pages 6.10-6.11, datasheet https://datasheet.octopart.com/LHC252015-Hoffman-datasheet-17579.pdf; product page https://us.rs-online.com/product/nvent-hoffman/lhc252015/70066796/). The product's flange trough and gasket are not modelled (dimensions unpublished): the body is a plain 1.5 mm steel box with plain front edges, built to the drawings `enclosure_body_v1.dxf` and `door_v1.dxf`, grey powder coat. Door: 250 x 200 mm 16 gauge plate (1.52 mm nominal, drawn as 1.5 mm; the simulated plate collider is 1.6 mm) with the return flanges below and a quarter-turn latch (fixed and not scored). Either the real product with its own hinge fittings or a fabricated copy to these drawings can be used; the hinge geometry below is ours.
- 2 representative 3-knuckle loose-pin hinges, machined zinc-plated steel, made from the drawings in `cad/`. No maker publishes knuckle or pin dimensions (Hoffman LHC datasheet: "Hinged door with removable hinge pins"; Hammond Eclipse hidden hinges with stainless steel hinge pins), so this geometry is an assumption to verify on a first build. Frame leaf (`hinge_frame_leaf_v1.stl`): 1.5 mm leaf screwed to the side wall, two fixed knuckles 10 mm OD x 15 mm long, 5.1 mm bore, 16 mm empty gap between them (inner faces 8 mm either side of the hinge centre), each with a leaf strap alongside it only (none across the gap), spanning x = 303.0 to 326.6 mm in table x. Door leaf (`hinge_door_leaf_v1.stl`): 1.5 mm leaf, 24 mm wide, on the outer face of the door plate (3.0 to 27.0 mm from the hinge axis across the door width), screwed to the door, with one knuckle 10 mm OD x 14.5 mm long, 5.1 mm bore, so the door has 0.75 mm of axial play per side in the 16 mm gap. Two 4.5 mm screw holes per frame strap; the door leaf is attached to the door plate by spot welds or two M4 screws (builder's choice, not part of the scored geometry). Machine shop or 3D-printed steel-filled or PA12 parts are acceptable for first trials; use steel for the scored build.
- 2 hinge pins (`hinge_pin_v1.dxf`): stainless shank 4.8 mm x 52 mm with a rounded (ball) tip, steel head 9 mm OD x 6 mm, head press-fit on the shank, covered by an orange vinyl cap (generic 11 mm vinyl tube cap, 9 mm bore) 11 mm OD x 14 mm tall pushed over the head (the cap is the 11 x 14 mm head that is gripped and pushed). Loose stainless hinge pins are a catalogue item for Hammond Eclipse enclosures (part no. EPINSS, Hammond Electrical Products Guide pages 287-288, "Stainless steel. Packaged in quantities of 5."); our pin is 4.8 mm, a representative size.
- 18 mm plywood backboard, 500 x 400 mm.
- 15 mm plywood spacer, 200 x 145 mm (`backboard_spacer_v1.stl`).
- Door support block and two guide strips (`door_setting_block_v1.stl`): must be cut from acetal (POM) (code friction 0.08; PhysX averages it with the door's 0.7, so the effective pair is 0.39, matching dry acetal on powder-coated steel at about 0.3 to 0.4), not printed plastic. The block is 17 mm tall: its top is 2.5 mm below the height that puts the door knuckles in the hinge gaps (19.5 mm), so the hand must lift the door by about 1.75 to 3.25 mm (door bottom between 18.75 and 20.25 mm) to get the 14.5 mm knuckle inside the 16 mm gap, and slide it the last 12 mm into line. It is a support block, like an installer's shim or bench support, deliberately below hinge height; it is not an alignment stop. At coaxial the door's hinge-side edge is 13.2 mm short of the side wall's front edge, so nothing stops the slide. Block: x = 38.4 to 288 mm, y = 30 to 93 mm. Guide strips: 16 x 20 x 30 mm, x = 41.4 to 57.4 mm, on top of the block (strip tops 47 mm above the table); outer strip y = 71.5 to 91.5 mm (1.5 mm from the plate's outer face at y = 70), inner strip y = 34.9 to 54.9 mm (13.5 mm from the plate's inner face at y = 68.4).
- Pin block (`pin_block_v1.stl`): blue closed-cell EVA or PE foam (friction 0.8), cut from foam, not printed.
- Door flanges: 12 mm deep, side flanges 4.5 mm in from the door edges, top flange inset 7.5 mm, bottom flange flush with the door bottom edge. Wing knob 20 x 12 x 30 mm, 25 mm from the latch edge, mid height. Door mass about 0.78 kg.
- Door-leaf heights: the door knuckle centres are 35.5 mm and 205.5 mm from the door bottom (44.5 mm from the door top), whereas the body hinges are 40 mm from the body bottom and top, because the door hangs 4.5 mm above the body bottom when its knuckles are centred in the gaps (3.75 mm once it rests on the lower fixed knuckles). Mounting the door leaves at 40 mm from the door ends would not work. The hinge axis is 6.6 mm outboard of the door's hinge-side plate edge.
- The back panel, DIN rail and terminal blocks inside the body are visual only; they may be omitted.
- Wood screws, M4 screws for the leaves, 1:1 paper templates `layout_template_v1.dxf`, `hinge_drilling_v1.dxf`.

## Procedure

1. Print `layout_template_v1.dxf` 1:1 and tape it to the table.
2. Screw the backboard upright at the rear, face at x = 456.6 mm, centred on y = -30.0 mm.
3. Screw the 15 mm spacer to the backboard, then mount the enclosure on it: front plane x = 306.6 mm, hinge axis x = 300.0 mm, y = 75.0 mm, enclosure bottom 15 mm above the table. Body spans y = -130.0 to 70.0 mm. Hinge centres 55 mm and 225 mm above the table.
4. Lift the door off by pulling both pins.
5. Stand the door on the support block between the two guide strips, opened 90 deg: door hinge axis at x = 260.0 mm, y = 75.0 mm, door bottom 17 mm above the table (resting on the support block).
6. Stand both pins head up in the pin block holes: pin starts at x = 220.0 mm, y = 195.0 and 225.0 mm.

Frame leaf holes: drill the left side wall per `hinge_drilling_v1.dxf` for `hinge_frame_leaf_v1` (2 holes of 4.5 mm per strap, 5 and 14 mm behind the front edge, at each fixed knuckle mid-height, at least 5 mm inside the leaf outline; hinge centres 40 mm from body top and bottom).

## Physical validation

Physical validation: the hardware has not yet been built and tested. A first build must check four things: (1) the real knuckle geometry and axial play of the hinge actually used (the 14.5 mm door knuckle in a 16 mm gap is our assumption); (2) a pin drops through all three bores under its own weight with the knuckles aligned; (3) the friction of the acetal block under the door; (4) an unlatched door stays within 3 mm of closed after release.

## One-time verification

- Before the first trial, check with a level that the two frame hinge axes are plumb (within 0.5 mm over the 170 mm hinge spacing) and coaxial.
- Hinge axis within 1 mm of the template crosshair (x = 300.0, y = 75.0), both hinges.
- Fixed knuckles of each hinge coaxial: a pin drops through both under its own weight.
- Fixed knuckle length 15 mm, gap between them 16 mm; door knuckle length 14.5 mm (0.75 mm axial play per side).
- Pin 4.8 mm in a 5.1 mm bore.
- Door knuckle fits in the gap once the door is lifted 1.75 to 3.25 mm off the support block.

## Judging tools

- Steel rule: axis position, knuckle and gap lengths.
- Feeler gauge: closing gap at the latch edge, 3 mm or less at both top and bottom.
- Visual: each pin head sits on the top fixed knuckle (head underside at most 1 mm above it); pin tip shows at least 3 mm below the bottom knuckle.

## Regenerate

`cd cad && python gen_cad.py` (runs a self-check: knuckle gap at least 16 mm, door knuckle axial play at most 0.75 mm per side, screw holes 5 mm inside the leaf outline).
