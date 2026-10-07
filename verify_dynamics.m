cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

load_system('ThreeDOF_Robot_Arm');
robot = importrobot('ThreeDOF_Robot_Arm');
robot.DataFormat = 'column';

% 3 test configurations
rng(42); % for repeatability
configs = [
    0.0,  0.0,  0.0;
    0.5, -0.4,  0.3;
   -0.3,  0.6, -0.5
]';

vels = [
    0.0,  0.0,  0.0;
    0.2, -0.1,  0.15;
   -0.1,  0.3, -0.2
]';

accs = [
    0.0,  0.0,  0.0;
    0.5, -0.2,  0.4;
   -0.4,  0.3, -0.3
]';

fprintf('=== DYNAMICS VERIFICATION AT 3 CONFIGURATIONS ===\n\n');

for i = 1:3
    q = configs(:, i);
    qd = vels(:, i);
    qdd = accs(:, i);
    
    % rigidBodyTree computations
    M = massMatrix(robot, q);
    V = velocityProduct(robot, q, qd);
    G = gravityTorque(robot, q);
    tau_rbt = inverseDynamics(robot, q, qd, qdd);
    
    % Check consistency of M*qdd + V + G vs inverseDynamics
    tau_decomp = M * qdd + V + G;
    err_rbt = norm(tau_rbt - tau_decomp);
    
    fprintf('--- Configuration %d ---\n', i);
    fprintf('q   = [%.3f, %.3f, %.3f] rad\n', q(1), q(2), q(3));
    fprintf('qd  = [%.3f, %.3f, %.3f] rad/s\n', qd(1), qd(2), qd(3));
    fprintf('qdd = [%.3f, %.3f, %.3f] rad/s^2\n', qdd(1), qdd(2), qdd(3));
    fprintf('Mass Matrix M(q):\n');
    disp(M);
    fprintf('Velocity Product C(q,qd)*qd: [%.4f, %.4f, %.4f] N*m\n', V(1), V(2), V(3));
    fprintf('Gravity Torque G(q):         [%.4f, %.4f, %.4f] N*m\n', G(1), G(2), G(3));
    fprintf('Total Torque tau:            [%.4f, %.4f, %.4f] N*m\n', tau_rbt(1), tau_rbt(2), tau_rbt(3));
    fprintf('Internal Dynamics Consistency Error: %.2e N*m\n\n', err_rbt);
end

close_system('ThreeDOF_Robot_Arm', 0);
