# Collada dae converter
This script was written to support the integration of custom animations in gazebo simulations.

## Basic animation components
We want to use a human motion as described in this [gazebo tutorial](http://gazebosim.org/tutorials?tut=actor&cat=build_robot). However, it is desired to use any animation and not just walking. The animations could be created in an external tool such as [ADAPT](https://github.com/ashoulson/ADAPT) and exported to a bvh motion file.
In gazebo, we can import a `actor`, which consists of two parts, a `skin` and an `animation`.
The `skin` needs to be provided in form of a [dae file](http://wazim.com/collada-tutorial1/). This file format not only describes the visual mesh, but also its skeleton.
The `animation` can then be described in either a bvh or dae file. It is crucial, that the skeleton (or armature) of  the `skin` and `animation` file are the same.
Therefore, if we let's say want to create 1000 custom human motions, we first have to make sure, we use the same skeleton in every animation.
A good starting point hereby is the [CMU motion capture database](https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture) that provides a lot of different human motions with the same skeleton.
Then, we have to create one dae file with our human mesh (any other mesh would work as well, try zombies if you like) and the desired skeleton. This can be done in blender, we **only** tested blender 2.79. Here is a human mesh, just import [walk.dae](https://raw.githubusercontent.com/osrf/gazebo/master/media/models/walk.dae) and delete the armature.
The basic workflow for this is described in the following section.

## Basic workflow:
The basic workflow is based on this [video tutorial](https://youtu.be/mzWyO838C-0?t=152), watch it, use it!
1. Create custom bvh animations or download them from [CMU database](https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture)
2. Open a character model in blender 2.79. You can also import the [walk.dae](https://raw.githubusercontent.com/osrf/gazebo/master/media/models/walk.dae) example file of the [gazebo tutorial](http://gazebosim.org/tutorials?tut=actor&cat=build_robot) and delete the armature.
3. Import your custom bvh file into blender, hereby select front to y, top to z and scaling to 1.
4. Most of the time, your animation and armature do not have the same size and position. First, scale your animation by pressing `s` and then type in your scaling factor (for CMU it is 0.01). **Please note this scaling factor.** Then, press `ctrl` + `a` to **apply the scaling**. (This is very important!) 
5. Now, rotate you animation if needed by pressing `r` then the rotation axis `x`, `y`, or `z` and the rotation in degree (90). **Please note this rotation.** Then, press `ctrl` + `a` to **apply the rotation**. (This is very important!)
6. Go to edit mode, activate xray vision and move the armature to the desired location. You can also note this translation but I think it is not necessary. Then, press `ctrl` + `a` to **apply the translation**. (This is very important!)
7. Now move the skin model on the armature as good as possible. Apply every rotation, scaling, or translation.
8. Select the armature again and go into edit mode. Connect the arm bones and leg bones as shown in the video and move the single joints so that they fit well into the skin. Since you are only transforming single joints, you don't need to apply the transformation here.
9. Go into pose mode and clear all transforms - the armature should be in the right position now.
10. Select the skin, `Shift` + `click` to selct a single armature bone, `Ctrl` + `p` to connect the skin with the armature with automatic weights.
11. Press play to check if you did everything correctly.
12. Export the human model with these settings

![blender dae export settings](./../../docs/blender_2_79_dae_export_options.png?raw=true)

13. Create a gazebo world file with the new converted dae as actor and animation (different bvh animation not supported yet). For example `test.world`:
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://sun</uri>
    </include>
    <actor name="actor">
      <skin>
        <filename>path_to_your_file/your_file.dae</filename>
      </skin>
      <animation name="animation">
        <filename>path_to_your_file/your_file.dae</filename>
      </animation>
    </actor>
  </world>
</sdf>
```
14. Run animation with `gazebo test.world`

## Problems that can occur
- Blender is not able to export transformed animations, so it is very important that you apply all transformations
- The original bvh files are obviously not transformed yet. So we need to transform them in the same way, you transformed the armature in blender. That's why you noted the scaling and rotation (I hope you did ;) )

## Helpful sources
[Gazebo actor tutorial](http://gazebosim.org/tutorials?tut=actor&cat=build_robot)

[Blender stickman tutorials](http://blender.freemovies.co.uk/stickman/)

[Collada DAE explanation](http://wazim.com/collada-tutorial1/)

The [dae converter branch](https://gitlab.lrz.de/cps-robotics/robot-rl/-/blob/dae_converter/) includes some useful blender files we created to test the script. Steps 1-6 are already made in the example blender files.

## ToDos
- Also make it possible to transform the skeleton and animation of a bvh file to replace the animation in the world file with any animation of the same skeleton structure
