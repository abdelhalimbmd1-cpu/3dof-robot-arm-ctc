%% Build ThreeDOF_Robot_Arm_CTC.slx
% Bulletproof builder for CTC vs PID model using Interpreted MATLAB Functions.

cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

modelName = 'ThreeDOF_Robot_Arm_CTC';

% Close if loaded
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
copyfile('ThreeDOF_Robot_Arm.slx', [modelName '.slx']);
load_system(modelName);

fprintf('1. Configuring revolute joints...\n');
joints = {'Revolute2', 'Revolute1', 'Revolute'}; % Joint 1, Joint 2, Joint 3
for i = 1:length(joints)
    jPath = [modelName '/' joints{i}];
    set_param(jPath, 'TorqueActuationMode', 'InputTorque');
    set_param(jPath, 'MotionActuationMode', 'ComputedMotion');
    set_param(jPath, 'SensePosition', 'on');
    set_param(jPath, 'SenseVelocity', 'on');
end

fprintf('2. Adding payload solid to Link2_1_RIGID...\n');
link2Sub = [modelName '/Link2_1_RIGID'];

% Add Rigid Transform for payload
payloadTrans = [link2Sub '/Payload_Transform'];
if isempty(find_system(link2Sub, 'SearchDepth', 1, 'Name', 'Payload_Transform'))
    add_block('sm_lib/Frames and Transforms/Rigid Transform', payloadTrans);
    set_param(payloadTrans, 'TranslationMethod', 'Cartesian');
    set_param(payloadTrans, 'TranslationCartesianOffset', '[0 0 -0.25]');
    set_param(payloadTrans, 'TranslationCartesianOffsetUnits', 'm');
end

% Add Payload Brick Solid
payloadSolid = [link2Sub '/Payload_Solid'];
if isempty(find_system(link2Sub, 'SearchDepth', 1, 'Name', 'Payload_Solid'))
    add_block('sm_lib/Body Elements/Brick Solid', payloadSolid);
    set_param(payloadSolid, 'BrickDimensions', '[0.06 0.06 0.06]');
    set_param(payloadSolid, 'InertiaType', 'Custom');
    set_param(payloadSolid, 'Mass', 'payload_mass');
    set_param(payloadSolid, 'MomentsOfInertia', '[1e-5 1e-5 1e-5]');
    set_param(payloadSolid, 'ProductsOfInertia', '[0 0 0]');
    set_param(payloadSolid, 'CenterOfMass', '[0 0 0]');
    set_param(payloadSolid, 'GraphicDiffuseColor', '[0.2 0.7 0.9]');
end

% Connect payload inside Link2_1_RIGID
try
    add_line(link2Sub, 'Solid/R', 'Payload_Transform/B', 'autorouting', 'smart');
    add_line(link2Sub, 'Payload_Transform/F', 'Payload_Solid/R', 'autorouting', 'smart');
catch
end

fprintf('3. Adding Converter blocks and Controller subsystems...\n');
% Add Simulink-PS converters for torques
for i = 1:3
    s2pName = [modelName '/S2P_Tau' num2str(i)];
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ['S2P_Tau' num2str(i)]))
        add_block('nesl_utility/Simulink-PS Converter', s2pName);
        set_param(s2pName, 'Unit', 'N*m');
    end
end

% Add PS-Simulink converters for Position (q) and Velocity (qd)
for i = 1:3
    p2sQ = [modelName '/P2S_Q' num2str(i)];
    p2sW = [modelName '/P2S_W' num2str(i)];
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ['P2S_Q' num2str(i)]))
        add_block('nesl_utility/PS-Simulink Converter', p2sQ);
        set_param(p2sQ, 'Unit', 'rad');
    end
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ['P2S_W' num2str(i)]))
        add_block('nesl_utility/PS-Simulink Converter', p2sW);
        set_param(p2sW, 'Unit', 'rad/s');
    end
end

% Connect physical signal ports to joints
for i = 1:3
    jName = joints{i};
    jPorts = get_param([modelName '/' jName], 'PortHandles');
    
    % Torque connection
    s2pPorts = get_param([modelName '/S2P_Tau' num2str(i)], 'PortHandles');
    try
        add_line(modelName, s2pPorts.RConn(1), jPorts.LConn(2), 'autorouting', 'smart');
    catch
    end
    
    % Position sensing
    p2sQPorts = get_param([modelName '/P2S_Q' num2str(i)], 'PortHandles');
    try
        add_line(modelName, jPorts.RConn(2), p2sQPorts.LConn(1), 'autorouting', 'smart');
    catch
    end
    
    % Velocity sensing
    p2sWPorts = get_param([modelName '/P2S_W' num2str(i)], 'PortHandles');
    try
        add_line(modelName, jPorts.RConn(3), p2sWPorts.LConn(1), 'autorouting', 'smart');
    catch
    end
end

% Mux Q and Mux W
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Mux_Q'))
    add_block('simulink/Signal Routing/Mux', [modelName '/Mux_Q']);
    set_param([modelName '/Mux_Q'], 'Inputs', '3');
end
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Mux_W'))
    add_block('simulink/Signal Routing/Mux', [modelName '/Mux_W']);
    set_param([modelName '/Mux_W'], 'Inputs', '3');
end

for i = 1:3
    try
        add_line(modelName, ['P2S_Q' num2str(i) '/1'], ['Mux_Q/' num2str(i)], 'autorouting', 'smart');
        add_line(modelName, ['P2S_W' num2str(i) '/1'], ['Mux_W/' num2str(i)], 'autorouting', 'smart');
    catch
    end
end

% Demux Tau
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Demux_Tau'))
    add_block('simulink/Signal Routing/Demux', [modelName '/Demux_Tau']);
    set_param([modelName '/Demux_Tau'], 'Outputs', '3');
end
for i = 1:3
    try
        add_line(modelName, ['Demux_Tau/' num2str(i)], ['S2P_Tau' num2str(i) '/1'], 'autorouting', 'smart');
    catch
    end
end

% Add Clock
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Clock'))
    add_block('simulink/Sources/Clock', [modelName '/Clock']);
end

% Add Trajectory Generator (Interpreted MATLAB Function)
trajBlock = [modelName '/Trajectory_Generator'];
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Trajectory_Generator'))
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', trajBlock);
    set_param(trajBlock, 'MATLABFcn', 'trajectory_generator_fcn');
end
try
    add_line(modelName, 'Clock/1', 'Trajectory_Generator/1', 'autorouting', 'smart');
catch
end

% Add Constant for ctrl_mode
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'ctrl_mode'))
    add_block('simulink/Sources/Constant', [modelName '/ctrl_mode']);
    set_param([modelName '/ctrl_mode'], 'Value', 'ctrl_mode');
end

% Add Mux for Controller Input: [q (3), qd (3), traj (9), ctrl_mode (1), Clock (1)] -> 5 inputs, total width 17
muxCtrl = [modelName '/Mux_CtrlIn'];
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Mux_CtrlIn'))
    add_block('simulink/Signal Routing/Mux', muxCtrl);
    set_param(muxCtrl, 'Inputs', '5');
end
try
    add_line(modelName, 'Mux_Q/1', 'Mux_CtrlIn/1', 'autorouting', 'smart');
    add_line(modelName, 'Mux_W/1', 'Mux_CtrlIn/2', 'autorouting', 'smart');
    add_line(modelName, 'Trajectory_Generator/1', 'Mux_CtrlIn/3', 'autorouting', 'smart');
    add_line(modelName, 'ctrl_mode/1', 'Mux_CtrlIn/4', 'autorouting', 'smart');
    add_line(modelName, 'Clock/1', 'Mux_CtrlIn/5', 'autorouting', 'smart');
catch
end

% Add Controller block (Interpreted MATLAB Function)
ctrlBlock = [modelName '/Robot_Controller'];
if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', 'Robot_Controller'))
    add_block('simulink/User-Defined Functions/Interpreted MATLAB Function', ctrlBlock);
    set_param(ctrlBlock, 'MATLABFcn', 'robot_controller_fcn');
end
try
    add_line(modelName, 'Mux_CtrlIn/1', 'Robot_Controller/1', 'autorouting', 'smart');
    add_line(modelName, 'Robot_Controller/1', 'Demux_Tau/1', 'autorouting', 'smart');
catch
end

% Add To Workspace blocks for logging
logs = {
    'Mux_Q', 'q_out';
    'Mux_W', 'qd_out';
    'Trajectory_Generator', 'traj_ref';
    'Robot_Controller', 'tau_out';
};
for i = 1:size(logs, 1)
    toWsName = [modelName '/ToWs_' logs{i, 2}];
    if isempty(find_system(modelName, 'SearchDepth', 1, 'Name', ['ToWs_' logs{i, 2}]))
        add_block('simulink/Sinks/To Workspace', toWsName);
        set_param(toWsName, 'VariableName', logs{i, 2});
        set_param(toWsName, 'SaveFormat', 'Timeseries');
        set_param(toWsName, 'SampleTime', '0.005');
    end
end
try
    add_line(modelName, 'Mux_Q/1', 'ToWs_q_out/1', 'autorouting', 'smart');
    add_line(modelName, 'Mux_W/1', 'ToWs_qd_out/1', 'autorouting', 'smart');
    add_line(modelName, 'Trajectory_Generator/1', 'ToWs_traj_ref/1', 'autorouting', 'smart');
    add_line(modelName, 'Robot_Controller/1', 'ToWs_tau_out/1', 'autorouting', 'smart');
catch
end

set_param(modelName, 'StopTime', '8.0');
set_param(modelName, 'Solver', 'ode15s');
set_param(modelName, 'MaxStep', '0.005');

save_system(modelName);
close_system(modelName);
fprintf('Model %s built and saved successfully!\n', modelName);
exit;
