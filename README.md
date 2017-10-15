# Space Robot Parameter Estimation

# Generating test cases
- The files present in test_case_data/{robot_make}/config_files have to pasted into ReDySim_Floating_Base/InvDyn
- From ReDySim_Floating_Base/InvDyn , execute run_me.m('/robot_make') <br />
The data files are generated and also copied to test_case_data/robot_make/sim_data 
- From test_case_data/ , execute simdata_to_realdata('/robot_make', [x_coord, y_coord, z_coord].') <br />
The argument is the sensor coordinates seen in the base CoM frame (ReDySim frame) <br />
The transformed data is stored in test_case_data/{robot_make}/sim_real_data <br />
- From src/ , execute run_me <br />
Before executing, make sure that the robot_make and the sensor coordinates are same as the ones used for simdata_to_realdata

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

# Add common functions path to matlab search path
In the Matlab command window, find out the matlab root folder <br />
matlabroot <br />
In the terminal, move to the folder which has matlabrc.m file <br />
cd matlabroot/toolbox/local <br />
Allow user to edit matlabrc.m <br />
Add the following command to the last line of the file <br />
addpath(location_to_common_funtions_directory) <br />

# Generate case wise regressor matrices
Generate the configuration files for every case in the designated location <br />
execute case_wise_reg_mat with the relevant inputs <br />

# Performance evaluation
## Obain SPV of estimated MPV
Obtain the double form of PCM from ./symbolic_derivations/generate_param_coupling_mat <br /> 
Save the PCM to param_coupling_mat.mat in the location, ./src/performance_evaluation <br />
Save true MPV, estimated MPV, and true SPV to true_estimated_params.mat in the location, ./src/performance_evaluation <br />
Run compute_est_spv.m

## Generating validation joint trajectories
Set the config parameters in make_rand_tr_params.m in ./ReDySim_Floating_base/InvDyn_fourier and run it <br />

## Obtain state and torque data with true and estimated SPV
Set t_final and incr in ./ReDySim_Floating_base/InvDyn_fourier/initials.m as per the requirement <br />

### With true SPV
Copy the inputs file which constitute the true SPV to ./ReDySim_Floating_base/InvDyn_fourier <br />
Goto to Getting the data files

### With estimated SPV
Copy the inputs.m and input_param_vec.mat file from src/performance_evaluation to ./ReDySim_Floating_base/InvDyn_fourier <br />
Goto to Getting the data files

### Getting the data files
After copying the appropriate inputs file to the InvDyn folder, the following steps are to be followed for both true and estimaed SPV <br />
Execute run_me('/dual_arm_articulate_validation') <br />
Copy tor.dat and timevar.dat to ./src/performance_evaluation/robot_make <br />
Set base_sensor_base_com_position_base_com to the value used for estimation <br />
simdata_to_realdata('/dual_arm_articulate_validation', base_sensor_base_com_position_base_com, 0) <br />
Copy ./test_case_data/dual_arm_articulate_validation/sim_real_data/statevar.dat to ./src/performance_evaluation/robot_make  <br />
For .dat files generated with true parameters and estimated, rename them to *_1.dat and *_2.dat respectively
