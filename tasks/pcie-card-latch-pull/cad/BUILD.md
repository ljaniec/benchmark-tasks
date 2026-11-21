# PCIe card latch-pull: real-hardware build

Assemble in under 15 minutes per fresh trial. Simulation uses the nominal coordinates in `layout_template_v1.dxf`; hardware within +-3 mm of the card and slot positions matches the scoring bands.

## Real counterpart

A Dell Precision 3660 Tower with its own system board, its own CPU heatsink and two DDR5 DIMMs, and a low-profile NVIDIA T400 4GB in the primary x16 slot (the T400 4GB is listed as a discrete graphics option in the Precision 3660 Tower Setup and Specifications). The side cover is removed, the PCIe door is open and the tower is laid on its side on the bench mat, board facing up. The Precision 3660 Tower Service Manual, section "Removing the graphics card", says to push and hold the securing tab on the graphics-card slot and lift the graphics card from the slot; this task is that step.

What the simulation simplifies: the board layout (heatsink and DIMM positions relative to the slot) is approximate and not measured from a 3660 board. The system board is modelled as a 244 x 244 mm slab, the CPU heatsink as a 95 x 95 x 54 mm block, the DIMMs as plain boxes, and the chassis as one flat plate under the board; the chassis walls (which rise above the card in the real tower, so the real card is lifted over them on its way to the tray), the rear wall with its expansion-slot openings, the power supply and the drive bays are not modelled. With the PCIe door open the card bracket is free to lift out. All force numbers below are model estimates, not measurements.

## Parts

- Dell Precision 3660 Tower (its own system board, CPU heatsink and 2 x DDR5 UDIMM fitted), side cover removed, PCIe door open.
- NVIDIA T400 4GB low profile graphics card (e.g. PNY VCNT4004GB-PB) with its full-height bracket (120 mm), in the primary x16 slot. Card shroud: black plastic over an aluminium heatsink.
- Conductive polypropylene antistatic tray, 300 x 180 x 15 mm, grey.
- Rubber bench mat, 440 x 640 x 3 mm.
- 40 mm wide green vinyl tape; orange paint pen.
- Push-pull digital force gauge, 0 to 50 N range, 0.01 N resolution, with a flat tip and a hook.
- Printed gauges: `tail_lift_gauge_v1.stl` (60 x 20 mm, steps 15.3 mm and 21.0 mm tall, stood on the system board top), `flat_check_wedge_v1.stl` (10 deg, 60 x 40 mm base).
- 1:1 layout template `layout_template_v1.dxf` and one-page BOM `assembly_bom_v1.pdf`.

## Procedure

1. Lay the mat and mark the world origin cross from the template (DXF layer ORIGIN).
2. Lay the tower on its side on the mat, board facing up, so the x16 slot runs along +y and the seated card matches layer CARD (card plane at x = 180 mm, tail end at x = 180 mm, y = -30 mm, bracket end toward +y, shroud facing -x toward the arms) and the slot matches layer SLOT. Align by the card and slot only: the MOTHERBOARD, COOLER, DIMM and BENCH_TRAY outlines are the approximate model positions, not alignment targets.
3. Place the antistatic tray to its outline (layer ANTISTATIC_TRAY), left (+y) of the tower.
4. Seat the card fully until the latch closes (the 89 mm gold fingers, 52.5 to 141.5 mm from the tail end, fill the slot; the latch hook sits in the notch at the tail-side end of the fingers). Leave the bracket screw out and the PCIe door open, so the bracket is free.
5. Put a 40 mm green tape band across the card top edge, 95 to 135 mm from the tail end (y 65 to 105 mm), wrapping down the shroud face to the shroud bottom and down the PCB back face (layer TAPE_BAND).
6. Paint the whole top of the latch tab orange with the paint pen (layer LATCH_TAB, 19 mm along x by 10 mm along y in the model). The tab sits beside the PCB back face (+x side), between the card and the CPU heatsink; it is pressed down from above beside the card.

## Post-build verification

- Card seated: gold fingers hidden in the slot, latch hook locked.
- Press the tab and release: the latch must return to locked and the card must stay held. If the latch stays open, do not run trials.
- Tab press force with the push-pull gauge (flat tip on the orange tab, push down): model estimate (not measured), latch spring 0.53 N m/rad gives roughly 2 N at the start of the press, 2.4 to 3.4 N at 3 deg and 4.9 to 7.0 N at the 12 deg open stop.
- Card withdrawal force with the latch held open (gauge hook on a loop of cord around the card top edge at the green band, pull up): model estimate about 12 N tail-first, about 25 N straight pull (spec floor for a straight pull about 12.3 N); the real board will differ.
- Record all three readings per build.

## Reset between trials

- Return the card to the slot and press it down until the latch clicks closed.
- Check the latch holds the card and the tab springs back; check the green band and orange patch are intact.
- Empty the antistatic tray and square the tower and tray to the template.

## Judging aids

- Tail lift: stand the step gauge beside the tail end in the side view. Standing on the system board top, the 15.3 mm step top marks 5 mm of tail lift; the tail bottom corner must rise above it.
- Gold fingers: the 21.0 mm step top (board datum) marks 10 mm above the slot top; the finger bottoms must clear it.
- Tab pressed: a finger pad on the orange tab top with the tab pushed down. A hand hovering above the tab does not count.
- Flat in tray: place the wedge beside the card in the side view. The card PCB must look flatter than the 10 deg slope.
- Overhead video: overlay the DXF at 1:1 on the mat to check the card lands inside the antistatic tray outline.
