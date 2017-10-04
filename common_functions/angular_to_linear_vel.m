% Compute the component of linear velocity contributed by angular velocity
% Inputs: Position of the reference frame in inertial frame (3 X 1)
%         Position of the target frame in inertial frame (3 X 1)
%         Angular velocity of the ref. frame in inertial frame (3 X 1)
% Outputs: Linear velocity contributed by angular velocity (3 X 1)
function lin_vel = angular_to_linear_vel(ref_frame_position, ...
    target_frame_position, ref_frame_ang_velocity)
    lin_vel = cross(ref_frame_ang_velocity, target_frame_position - ref_frame_position);
end