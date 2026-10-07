cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

global robot_nominal;
load_system('ThreeDOF_Robot_Arm');
robot_nominal = importrobot('ThreeDOF_Robot_Arm');
robot_nominal.DataFormat = 'column';

% Parameters
payload_mass = 0.0;
Kp_pid = [250; 350; 150];
Kd_pid = [40; 60; 25];
Ki_pid = [10; 15; 8];
Kp_ctc = [225; 225; 225];
Kd_ctc = [30; 30; 30];

% 1. Test PID
fprintf('Testing PID simulation (ctrl_mode=1)...\n');
ctrl_mode = 1;
load_system('ThreeDOF_Robot_Arm_CTC');
set_param('ThreeDOF_Robot_Arm_CTC', 'StopTime', '1.0');
simOut_pid = sim('ThreeDOF_Robot_Arm_CTC');
disp('PID simulated successfully!');

% 2. Test CTC
fprintf('Testing CTC simulation (ctrl_mode=2)...\n');
ctrl_mode = 2;
simOut_ctc = sim('ThreeDOF_Robot_Arm_CTC');
disp('CTC simulated successfully!');

close_system('ThreeDOF_Robot_Arm_CTC', 0);
close_system('ThreeDOF_Robot_Arm', 0);
exit;
