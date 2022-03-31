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

## (3D Printed) Parts with Inserts

## Stainless Steel Guide Tube
This thin-walled tube comes from McMaster-Carr and must be cut to length with a pipe cutter.


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

