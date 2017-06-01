# Space Robot Parameter Estimation

# Generating test cases
- The files present in config_files have to pasted in ReDySim_Floating_Base/ForDyn
- Execute run_me.m to generate .dat files

## .dat files indexing
In files, envar.dat, mtvar.dat, and statevar.dat, data in each row is observed at the time mentioned in the corresponding row in timevar.dat
- mtvar.dat: Contains momentum data
C1, C2, C3: Linear momentum along x,y, and z axes respectively
C4, C5, C6: Angular momentum along x,y, and z axes respectively

- statvar.dat: Contains the kinematic data of the base and the joints
Suppose there are n joints on all the arms and links in the robot, then
C1, C2, C3: x, y, and z positions of the base
