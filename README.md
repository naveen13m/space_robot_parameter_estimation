# Space Robot Parameter Estimation

# Generating test cases
- The files present in config_files have to pasted in ReDySim_Floating_Base/ForDyn
- Execute run_me.m to generate .dat files

# .dat files indexing
In files, envar.dat, mtvar.dat, and statevar.dat, data in each row is observed at the time mentioned in the corresponding row in timevar.dat<br />
- mtvar.dat: Contains momentum data<br />
C1, C2, C3: Linear momentum along x,y, and z axes respectively<br />
C4, C5, C6: Angular momentum along x,y, and z axes respectively<br />

- statvar.dat: Contains the kinematic data of the base and the joints<br />
Suppose there are n joints on all the arms and links in the robot, then<br />
C1, C2, C3: x, y, and z positions of the base<br />
