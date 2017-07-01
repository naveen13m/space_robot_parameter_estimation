function lin_vel = angular_to_linear_vel(ref_frame_position, ...
    link_frame_position, ref_joint_ang_velocity)
    lin_vel = cross(ref_joint_ang_velocity, link_frame_position - ref_frame_position);
end