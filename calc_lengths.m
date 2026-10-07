cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');
load_system('ThreeDOF_Robot_Arm');
robot = importrobot('ThreeDOF_Robot_Arm');
robot.DataFormat = 'column';

q0 = [0; 0; 0];
T_base = getTransform(robot, q0, 'Base');
T_b1 = getTransform(robot, q0, 'Body1');
T_b2 = getTransform(robot, q0, 'Body2');
T_b3 = getTransform(robot, q0, 'Body3');

disp('T_base:'); disp(T_base);
disp('T_Body1:'); disp(T_b1);
disp('T_Body2:'); disp(T_b2);
disp('T_Body3:'); disp(T_b3);

p0 = T_base(1:3, 4);
p1 = T_b1(1:3, 4);
p2 = T_b2(1:3, 4);
p3 = T_b3(1:3, 4);

fprintf('Length Base->Joint1: %.4f m\n', norm(p1 - p0));
fprintf('Length Joint1->Joint2: %.4f m\n', norm(p2 - p1));
fprintf('Length Joint2->Joint3: %.4f m\n', norm(p3 - p2));

close_system('ThreeDOF_Robot_Arm', 0);
exit;
