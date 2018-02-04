# What is this?

"naoV40red.urdf" is a NAO model that does not include the fingers (16 joints)
as articulated joints, since they are only used to open or close the hand. This
model has thus 26 degrees of freedom instead of 42.

The urdf has been taken from the nao_description package:

```
nao_robot/nao_description/urdf/naoV40_generated_urdf/nao.urdf
```

but "continuous" joints for the fingers have been changed to "fixed". The
original urdf model can be found at https://github.com/ros-naoqi/nao_robot
