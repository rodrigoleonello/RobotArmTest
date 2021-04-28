# RobotArmTest
This package is related to the control test task from Remy.

## Installing and Running

To install and run the system, clone this repository and run, in the root folder:
```
mkdir build
cd build
cmake ..
make
```
Return to the root folder and run (the two arguments are required):
```
./RobotArm cfg/input.in cfg/config.json
```
The output file "out.csv" is composed of time, the end-effector path (x,y,z), the joints path (t1,t2,t3) and control signals (ux,uy,uz) for each joint.

## Results

The results are available in the "output" folder and can be generate by the .m file, available on the root folder.