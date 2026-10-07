cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

load_system('ThreeDOF_Robot_Arm_CTC');
sub = 'ThreeDOF_Robot_Arm_CTC/Link2_1_RIGID';

% Check port handles
ph_solid = get_param([sub '/Solid'], 'PortHandles');
ph_trans = get_param([sub '/Payload_Transform'], 'PortHandles');
ph_payload = get_param([sub '/Payload_Solid'], 'PortHandles');

% Check existing lines
lines = get_param(sub, 'Lines');
disp(['Existing lines in Link2_1_RIGID: ', num2str(length(lines))]);

% Connect Solid to Payload_Transform B port (LConn(1))
try
    add_line(sub, ph_solid.LConn(1), ph_trans.LConn(1), 'autorouting', 'smart');
    disp('Connected Solid -> Payload_Transform');
catch ME
    disp(['Solid->Transform: ', ME.message]);
end

% Connect Payload_Transform F port (RConn(1)) to Payload_Solid R port (LConn(1))
try
    add_line(sub, ph_trans.RConn(1), ph_payload.LConn(1), 'autorouting', 'smart');
    disp('Connected Payload_Transform -> Payload_Solid');
catch ME
    disp(['Transform->Solid: ', ME.message]);
end

save_system('ThreeDOF_Robot_Arm_CTC');
close_system('ThreeDOF_Robot_Arm_CTC');
disp('Model saved successfully with payload connected!');
exit;
