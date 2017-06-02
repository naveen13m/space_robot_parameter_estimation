# Space Robot Parameter Estimation

# Generating test cases
- The files present in config_files have to pasted in ReDySim_Floating_Base/InvDyn
- Execute run_me.m to generate .dat files

# .dat files indexing
Note: All the data is in SI units<br />
In files, envar.dat, mtvar.dat, and statevar.dat, data in each row is observed at the time mentioned in the corresponding row in timevar.dat<br />
- mtvar.dat: Contains momentum data<br />
C1, C2, C3: Linear momentum along x,y, and z axes respectively<br />
C4, C5, C6: Angular momentum along x,y, and z axes respectively<br />

- statvar.dat: Contains the kinematic data of the base and the joints<br />
For n joints on the arms of the robot, then<br />
C1, C2, C3: x, y, and z positions of the base<br />
C4, C5, C6: Euler angles of the base (ZXY euler angle convention)<br />
C7 to C(6+n): Joint position<br />
C(7+n), C(8+n), C(9+n): x, y, and z velocities of the base<br />
C(10+n), C(11+n), C(12+n): Rates of euler angles of the base<br />
C(13+n) to C(13+2n-1): Joint velocities<br />
C(13+2n): Total energy of the system<br />
