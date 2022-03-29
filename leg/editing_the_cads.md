# Editing The Model

Here's a quick walkthrough on exporting leg assemblies with custom dimenionsions.

## Configuraiton Overview
Many parts in the assembly are driven from a single part (`main\_body\_sketch.SLDPRT`) with many configurations, each of which has parameters set to different values.
The assembly itself also several configurations, each of which represent a different leg designs at a particular point in the design space.
This assembly configuration is built from parts pinned to a particular configuration.

## Tweaking Existing Leg Designs
Four Leg designs exist as separate configurations in the top assembly.

![](https://github.com/sburden-group/pareto_leg_hardware/blob/main/leg/pics/assembly_configurations.png?raw=true)

To tweak an existing design's parameters, start by opening `main_body_sketch.SLDPRT`. Note that this part has configurations with names that match that of the corresponding assembly configuration.
To edit the equations that drive any configuration, right-click on *Equations* in the feature tree and select *Manage Equations*.

![](https://github.com/sburden-group/pareto_leg_hardware/blob/main/leg/pics/manage_equations.png?raw=true)

This new window gives you direct edit access to parameters that drive the model.
Note that these parameters can be made to apply globally or be configuration-specific.
In pretty-much all cases, we want our parameter values to apply to be configuration-specific.

To edit a parameter, first ensure that the desired configuration is selected in the upper right corner and make sure that the variable value scope is set to *This Configuration*.

![](https://github.com/sburden-group/pareto_leg_hardware/blob/main/leg/pics/this_configuration.png?raw=true)

Now you can just check *OK*, and the model configuration, and all derived configurations will apply the new value.
At this point, if you have the assembly open, it should prompt you about rebuilding the model.
If you say yes, the parts will resize.
If you save `main_body_sketch.SLDPRT`, these changes will take effect.
