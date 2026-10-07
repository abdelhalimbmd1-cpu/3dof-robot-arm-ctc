% test_kinematics_waypoints.m
cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');
robot = importrobot('ThreeDOF_Robot_Arm');
robot.DataFormat = 'column';
ee_name = robot.BodyNames{end};

% Helper function to get Cartesian position of EE
get_pos = @(q) getTransform(robot, q(:), ee_name);

% Let's test different pitch angles to see vertical reach
q_home = [-2.6763; -1.8134; 0.4091];
T = get_pos(q_home); p_home = T(1:3, 4);

% Yaw angle for Station A: -2.8 rad (~ -160 deg)
% Yaw angle for Station B: -1.6 rad (~ -92 deg)

% Pick Station A:
% Hover above Station A:
q_A_hover = [-2.80; -1.40; 0.50];
T_Ah = get_pos(q_A_hover); p_Ah = T_Ah(1:3, 4);

% Pick at Station A (reach down/forward):
q_A_pick = [-2.80; -1.00; 0.90];
T_Ap = get_pos(q_A_pick); p_Ap = T_Ap(1:3, 4);

% Place Station B:
% Hover above Station B:
q_B_hover = [-1.60; -1.40; 0.50];
T_Bh = get_pos(q_B_hover); p_Bh = T_Bh(1:3, 4);

% Place at Station B (reach down/forward):
q_B_place = [-1.60; -1.00; 0.90];
T_Bp = get_pos(q_B_place); p_Bp = T_Bp(1:3, 4);

fprintf('--- PICK & PLACE WAYPOINTS ---\n');
fprintf('Home:         [%6.3f, %6.3f, %6.3f] m | q = [%.3f, %.3f, %.3f] rad\n', p_home, q_home);
fprintf('Pick Hover:   [%6.3f, %6.3f, %6.3f] m | q = [%.3f, %.3f, %.3f] rad\n', p_Ah, q_A_hover);
fprintf('Pick Point A: [%6.3f, %6.3f, %6.3f] m | q = [%.3f, %.3f, %.3f] rad\n', p_Ap, q_A_pick);
fprintf('Place Hover:  [%6.3f, %6.3f, %6.3f] m | q = [%.3f, %.3f, %.3f] rad\n', p_Bh, q_B_hover);
fprintf('Place Point B:[%6.3f, %6.3f, %6.3f] m | q = [%.3f, %.3f, %.3f] rad\n', p_Bp, q_B_place);

fprintf('Pick Delta Z (Hover -> Pick):   %+.3f m\n', p_Ap(3) - p_Ah(3));
fprintf('Place Delta Z (Hover -> Place): %+.3f m\n', p_Bp(3) - p_Bh(3));
fprintf('Horizontal Distance (A -> B):    %.3f m\n', norm(p_Bp(1:2) - p_Ap(1:2)));
exit;
