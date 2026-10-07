% fix_model_defaults.m
cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

load_system('ThreeDOF_Robot_Arm_CTC');

% 1. Set Constant block 'ctrl_mode' to direct value '2' (or use workspace variable with default)
set_param('ThreeDOF_Robot_Arm_CTC/ctrl_mode', 'Value', '2');

% 2. Set StopTime to 14.0 for the Pick-and-Place mission
set_param('ThreeDOF_Robot_Arm_CTC', 'StopTime', '14.0');

% 3. Set Model Workspace defaults so the model is 100% self-contained
hws = get_param('ThreeDOF_Robot_Arm_CTC', 'ModelWorkspace');
hws.assignin('ctrl_mode', 2);
hws.assignin('payload_mass', 0.5);
hws.assignin('Kp_pid', [250; 400; 250]);
hws.assignin('Kd_pid', [35; 50; 35]);
hws.assignin('Ki_pid', [15; 25; 15]);
hws.assignin('Kp_ctc', [225; 225; 225]);
hws.assignin('Kd_ctc', [30; 30; 30]);

% 4. Set InitFcn callback in model properties
init_code = sprintf([ ...
    'if ~exist(''ctrl_mode'', ''var''), ctrl_mode = 2; end;\n' ...
    'if ~exist(''payload_mass'', ''var''), payload_mass = 0.5; end;\n' ...
    'if ~exist(''Kp_pid'', ''var''), Kp_pid = [250; 400; 250]; end;\n' ...
    'if ~exist(''Kd_pid'', ''var''), Kd_pid = [35; 50; 35]; end;\n' ...
    'if ~exist(''Ki_pid'', ''var''), Ki_pid = [15; 25; 15]; end;\n' ...
    'if ~exist(''Kp_ctc'', ''var''), Kp_ctc = [225; 225; 225]; end;\n' ...
    'if ~exist(''Kd_ctc'', ''var''), Kd_ctc = [30; 30; 30]; end;\n' ...
]);
set_param('ThreeDOF_Robot_Arm_CTC', 'InitFcn', init_code);

% Save model
save_system('ThreeDOF_Robot_Arm_CTC');
disp('Model parameters and self-contained defaults saved successfully.');

% 5. Test simulating with completely clear base workspace
clear all;
load_system('ThreeDOF_Robot_Arm_CTC');
disp('Testing sim with empty workspace...');
simOut = sim('ThreeDOF_Robot_Arm_CTC', 'StopTime', '0.5');
disp('Simulation succeeded with zero errors!');
close_system('ThreeDOF_Robot_Arm_CTC', 0);
