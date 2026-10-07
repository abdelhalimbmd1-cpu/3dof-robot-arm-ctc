cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');
load_system('ThreeDOF_Robot_Arm');
robot = importrobot('ThreeDOF_Robot_Arm');
showdetails(robot);
close_system('ThreeDOF_Robot_Arm', 0);
exit;
