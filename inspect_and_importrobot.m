cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');
load_system('ThreeDOF_Robot_Arm');

joints = {'Revolute2', 'Revolute1', 'Revolute'};
for i = 1:length(joints)
    jName = ['ThreeDOF_Robot_Arm/' joints{i}];
    fprintf('=== Joint: %s ===\n', joints{i});
    try
        fprintf('  Z Axis: %s\n', get_param(jName, 'ZAxisDirection'));
    catch
    end
    try
        fprintf('  Actuation Torque: %s\n', get_param(jName, 'TorqueActuationMode'));
        fprintf('  Actuation Motion: %s\n', get_param(jName, 'MotionActuationMode'));
        fprintf('  Sense Position: %s\n', get_param(jName, 'SensePosition'));
        fprintf('  Sense Velocity: %s\n', get_param(jName, 'SenseVelocity'));
    catch ME
        fprintf('  Param error: %s\n', ME.message);
    end
end

% Try importrobot
fprintf('\n--- Testing importrobot(''ThreeDOF_Robot_Arm'') ---\n');
try
    robot = importrobot('ThreeDOF_Robot_Arm');
    disp('importrobot succeeded!');
    disp(robot);
    showdetails(robot);
catch ME
    fprintf('importrobot FAILED: %s\n', ME.message);
    disp(ME.stack);
end

close_system('ThreeDOF_Robot_Arm', 0);
exit;
