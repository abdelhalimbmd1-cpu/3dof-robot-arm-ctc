cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');
load_system('ThreeDOF_Robot_Arm');

% Test creating a solid block
try
    new_system('test_solid');
    load_system('test_solid');
    add_block('sm_lib/Body Elements/Brick Solid', 'test_solid/Payload');
    set_param('test_solid/Payload', 'InertiaType', 'PointMass');
    set_param('test_solid/Payload', 'PointMass', '0');
    disp('PointMass = 0 set successfully!');
catch ME
    disp(['Error setting PointMass 0: ' ME.message]);
end
close_system('test_solid', 0);
close_system('ThreeDOF_Robot_Arm', 0);
exit;
