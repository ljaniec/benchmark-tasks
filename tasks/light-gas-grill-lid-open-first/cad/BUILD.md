# Turn on a Weber Q 1000 lid-open-first: real-hardware build

Source product: Weber Q 1000 (LP, US model 50060001; the Q 1200 has the same cookbox and lid plus side tables), used as bought. There are no printed grill parts: the lid, its handle, the cookbox, the frame and the knob's own push-to-turn latch are the grill's own. Weber spec (https://www.weber.com/US/en/grills/gas-grills/q-series/50060001.html): lid closed 36.9 cm H x 68.6 cm W x 42.0 cm D, lid open 59.7 cm H x 52.1 cm D. Only two small parts are printed (`python gen_cad.py`, numpy only):

## Parts (printed, mm)

- `angle_card_v1.stl` - flat card 200 x 307 x 3 mm with a 6 mm round hinge-axis mark (1 mm high) and two 1 mm raised tape-guide ribs, 30 x 6 mm, at 45 deg and 70 deg of lid opening, 110 mm from the axis mark. The axis mark is 20 mm from the rear edge and 6.5 mm below the card centre. Needs a bed of about 320 mm; otherwise cut the card from 3 mm plastic or foam board and mark the axis point and the two rib positions with a pencil.
- `knob_cover_v1.stl` - red cover for the stock knob (the real knob is black; the red cap is added so the camera and the instruction can name it): 56 mm cap shell, 14 mm deep, 3 mm closed front wall, open back, with a 12 mm wide x 40 mm tall grip bar standing 18 mm proud of the front face. The inner diameter is 50 mm in `gen_cad.py` (`KNOB_D`): measure the stock knob and set it before printing. Print in red, or paint it.

**Safety: run every physical trial with the propane cylinder removed (or empty) and the regulator vented; no gas may be present. The igniter may click.**

## Assembly

1. Knob cover: press it onto the stock knob so it turns with the knob (a thin foam strip inside the shell helps), grip bar vertical at OFF.
2. Dial sticker: print or draw a ring of ticks 40 mm from the knob axis on the cookbox front: OFF at the top (0 deg), then 10 deg, 80 deg, START/HI at 90 deg, 100 deg and LO at 120 deg, all counted anticlockwise as seen from the front (use a protractor). Match OFF and START/HI to the knob's own markings; the 10 deg tick is the latch check line.
3. Angle card: stand it upright on the left of the lid, 12 mm from the lid side wall, plane parallel to the side wall, the axis mark exactly beside the lid hinge axis (card centre 0.08 m in front of the hinge axis, card bottom 0.147 m below it, standing on the frame). Hold it with a clamp or a stand so it does not touch the grill. Put white tape on the two ribs: 45 deg (optional) and 70 deg (the lid threshold).
4. Place the grill at its marked position on the table. A real grill is free-standing (about 11-13 kg); no fixing is needed.

## Checks

- Lid opens to its 90 deg stop; its edge crosses the 70 deg tape mark on the card.
- Knob unpushed cannot leave OFF (the tap's own push-to-turn latch); pushed in, it turns anticlockwise to START/HI (90 deg).
- Knob cover grip bar is 12 mm wide, 40 mm tall, 18 mm proud of the cap front face.
- A tape measure reads at least 50 mm from each open gripper to the frame, carry grips, cookbox, lid, handle, knob, cylinder, regulator and angle card in the final state.
