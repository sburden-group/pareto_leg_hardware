# Pareto Leg Hardware
This is a CAD snapshot (Solidworks 2020) of the Pareto Leg.

## Summary
This model is modeled "[Top-Down](https://help.solidworks.com/2018/english/SolidWorks/sldworks/c_Top-Down_Design_Overview.htm#:~:text=In%20top%2Ddown%20assembly%20design,moves%20down%20to%20the%20parts.)," where most of the structural parts are driven from parameters in *main\_body\_sketch.SLDPRT*.
The advantage is that variables can be tweaked from one file, and the rest of the top assembly will resize to accommodate the changes.
The top assembly also contains several *Configurations* with predefined parameters that will resize the model to create a particular leg instance.

If you need to make model changes, follow the guidelines in [editng_the_cads.md](./editing_the_cads.md)


# Fabrication Notes

## Fabrication Tools
* Drill Press
* 7mm Carbide Drill bit, [McMaster-Carr](https://www.mcmaster.com/3030A34/)
* Handheld Pipe Cutter, [McMaster-Carr option](https://www.mcmaster.com/pipe-cutters/metal-tube-and-conduit-cutters-8/)
* Soldering iron (for installing heat set inserts)
* Soldering iron Heat Set Insert Installation Tip, [Amazon](https://www.amazon.com/Heat-Set-Inserts-Compatible-SP40NKUS-Connecting/dp/B08B17VQLD)

## Stainless Steel Parts
Flat stainless steel parts (spring anchors) need to be sourced externally.
All spring anchors can be fabricated externally from [SendCutSend](https://sendcutsend.com/).
Simply import a DXF and specify *0.8mm thick 304 Stainless Steel* as the material.
(Pricing is ~$30 for about 10 spring anchors, and turnaround time is about 1 week.)

## Carbon Fiber Parts
Carbon Fiber Parts need to be sourced externally.
All carbon fiber parts can be fabricated externally from [CNCMadness.com](https://cncmadness.com/).
Just send him a DXF and specify *6mm plain weave carbon fiber*.
(Pricing is < $200 for two leg setups, and turnaround time is about 1 week.)

### Post-Processing
Parts from CNC Madness will need to be touched up before they can be used.
Specifically, the holes need to be cleaned up with a 7mm drill bit and a 5mm bushin needs to be installed in each joint hole.

### Warnings:
Carbon fiber is both sharp and very abrasive.
It can be wet-sanded to reduce the risk of injuring people who run their hands along the edge.
You must use carbide tooling to work with carbon fiber.
Conventional high speed steel tools will dull immediately.

## 3D Printed Parts

### Printed Part Settings
Print the following parts with these settings.
Note that exceptions exist, though.
See the notes for exceptions to these settings.

* 0.2mm or 0.24mm thick layer lines
* PLA or HTPLA
* print in the provided STL orientations
* 6 perimeter layers (This is critical especially for parts with inserts)
* No support material

| **Qty** |                                                                    **Part Link**                                                                   |                                                                                                     **Notes**                                                                                                    |
|:-------:|:--------------------------------------------------------------------------------------------------------------------------------------------------:|:----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
|    1    | [Foot](./fabrication_exports/stls/foot.STL)                                                                                                        | Print with ~20 perimeter layers to create a solid object. Print from something more impact resistant than PLA, perhaps [CF HTPLA](https://www.proto-pasta.com/products/light-grey-carbon-fiber-composite-htpla). |
|    1    | [Threaded Foot Tip](./fabrication_exports/stls/threaded_foot_tip.STL)                                                                              | Print from TPU or something rubbery                                                                                                                                                                              |
|    2    | [Knee Bumper](./fabrication_exports/stls/knee_bumpter.STL)                                                                                         |                                                                                                                                                                                                                  |
|    2    | [Pogo Bearing Spacer](./fabrication_exports/stls/pogo_bearing_spacer.STL)                                                                          |                                                                                                                                                                                                                  |
|    2    | [Knee Spring Anchor](./fabrication_exports/stls/knee_bumper.STL)                                                                                   |                                                                                                                                                                                                                  |
|    2    | [Pogo Center Tabs](./fabrication_exports/stls/pogo_center_tab.STL)                                                                                 |                                                                                                                                                                                                                  |
|    2    | [Pogo Center Tube Retainer](./fabrication_exports/stls/pogo_center_tube_retainer.STL)                                                              | can also be laser cut from 3.175mm Delrin sheet                                                                                                                                                                  |
|    2    | [Inc Encoder Shaft Adaptor](./fabrication_exports/stls/inc_encoder_shaft_adaptor.STL)                                                              | can also be laser cut from 3.175mm Delrin sheet. Only needed if using AMT102-V encoders.                                                                                                                         |
|    2    | [Inc Encoder Shaft](./fabrication_exports/stls/incremental_encoder_shaft.STL)                                                                      | Print *with* support material. Only needed if using AMT102-V encoders.                                                                                                                                           |
|    8    | [M3 8mm Spacer](./fabrication_exports/stls/8mm_m3_spacer.STL)                                                                                      |                                                                                                                                                                                                                  |
|    1    | [Base Plate](./fabrication_exports/stls/base_plate.STL) or [No-Knee Springs Base Plate](./fabrication_exports/stls/base_plate_no_knee_springs.STL) | two different sizes depending on if the design uses knee springs or not.                                                                                                                                         |

### Post Processing
The Foot Piece needs to have its 7mm hole drilled out, and then two 5mm bushing need to be pressed in, one from either end.
(You can use the same 7mm carbon fiber drill bit also used on the carbon fiber parts.)

## (3D Printed) Parts with Inserts
Some parts need heat-set inserts installed into them.
To install these, use a heat-set insert tip

## Stainless Steel Guide Tube
This thin-walled tube comes from McMaster-Carr and must be cut to length with a pipe cutter. Length varies depending on design.

# Assembly
These robot legs can be assembled by you (or by some eager mechanically-curious undergrads).
For some of the finer details of the leg joint assembly, see [this doc](./knee_assembly_details.pdf).
For ordering more parts, see the [BOM](https://github.com/sburden-group/pareto_leg_hardware/blob/main/leg/no_spring_leg_bom%20L.pdf).

## Assembly Tools
You will need:
* 2mm Ball End Hex Key, [McMaster-Carr](https://www.mcmaster.com/5497A52/)
* 2.5mm Ball End Hex Key, [McMaster-Carr](https://www.mcmaster.com/5497A53/)
* 3mm Ball End Hex Key, [McMaster-Carr](https://www.mcmaster.com/5497A54/)
* 7mm Hex Nut Driver, [Amazon](https://www.amazon.com/gp/product/B000BQJ5ZY)

