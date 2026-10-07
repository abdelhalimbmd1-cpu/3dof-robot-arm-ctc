function tau = ctc_inverse_dynamics_helper(q, qd, qdd_star)
% Evaluates rigidBodyTree inverse dynamics for Computed Torque Control
global robot_nominal;

if isempty(robot_nominal)
    load_system('ThreeDOF_Robot_Arm');
    robot_nominal = importrobot('ThreeDOF_Robot_Arm');
    robot_nominal.DataFormat = 'column';
end

tau = inverseDynamics(robot_nominal, q, qd, qdd_star);
end
