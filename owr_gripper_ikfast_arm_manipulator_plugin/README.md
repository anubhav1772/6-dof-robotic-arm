### Testing output from IKFast
To test the forward and inverse kinematics results, download this demo code:
https://github.com/davetcoleman/clam_rosbuild/blob/master/clam_ik/models/ikfastdemo.cpp

Copy the source file to the same directory as the output files from IKFast.
Edit `ikfastdemo.cpp` and define `IK_VERSION` with the version of IKFast you used.
Make sure the `#include` line points to the output file from IKFast.

    #define IK_VERSION 61
    #include "output_ikfast61.cpp"
#### Compile the program:

    g++ ikfastdemo.cpp -lstdc++ -llapack -o compute -lrt
(If you get an error, you can try changing the order of the filename/arguments to g++. You may need to include openravepy. You may need to install the lapack library.)

    g++ ikfastdemo.cpp -lstdc++ -llapack -o compute -lrt -I<openravepy>
Run the program to check the correct usage:

    ./compute
You can compute the forward kinematics by specifying all joint angles. Inverse kinematics can be found from either a translation-quaternion or translation-rotation matrix pose. Kinematics can also be verified visually after creating a plugin (below), using the planning-scene environment.

#### Usage:

    ./compute fk j0 j1 ... j5

Returns the forward kinematic solution given the joint angles (in radians).

    ./compute ik  t0 t1 t2  qw qi qj qk  free0 ...

Returns the ik solutions given the transformation of the end effector specified by a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k). There are 0 free parameters that have to be specified.

    ./compute ik  r00 r01 r02 t0  r10 r11 r12 t1  r20 r21 r22 t2  free0 ...

Returns the ik solutions given the transformation of the end effector specified by a 3x3 rotation R (rXX), and a 3x1 translation (tX). There are 0 free parameters that have to be specified.

    ./compute iktiming

For fixed number of iterations, generates random joint angles, then calculates fk, calculates ik, measures average time taken.

    ./compute iktiming2

For fixed number of iterations, with one set of joint variables, this finds the ik solutions and measures the average time taken.
