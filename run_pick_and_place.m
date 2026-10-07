% run_pick_and_place.m
% =========================================================================
%   3-DOF ROBOT ARM: PICK-AND-PLACE MISSION SIMULATION & CONTROL BENCHMARK
% =========================================================================
% Simulates a realistic industrial pick-and-place operation:
%   Phase 1 (0.0 - 2.0s):  Approach Pick Station A (Hover)
%   Phase 2 (2.0 - 3.5s):  Descend onto Object at Point A
%   Phase 3 (3.5 - 4.5s):  Grasp Object at Point A (Dwell)
%   Phase 4 (4.5 - 6.0s):  Lift Object to Clearance Height
%   Phase 5 (6.0 - 8.5s):  Transit across workspace to Place Station B (Hover)
%   Phase 6 (8.5 - 10.0s): Descend onto drop fixture at Point B
%   Phase 7 (10.0 - 11.0s): Release Object at Point B (Dwell)
%   Phase 8 (11.0 - 12.0s): Retract / Lift from Point B
%   Phase 9 (12.0 - 14.0s): Return to Safe Home / Standby Pose
%
% Controllers:
%   - Computed Torque Control (CTC): Feedback linearization + PD
%   - Joint-space PID: Standard decoupled PID
% =========================================================================

% Start of script
fprintf('========================================================================\n');
fprintf('        3-DOF ROBOT ARM: PICK-AND-PLACE SIMULATION & BENCHMARK          \n');
fprintf('========================================================================\n\n');

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

model_nominal = 'ThreeDOF_Robot_Arm';
model_ctc = 'ThreeDOF_Robot_Arm_CTC';

% Load robot dynamics model
load_system(model_nominal);
global robot_nominal;
robot_nominal = importrobot(model_nominal);
robot_nominal.DataFormat = 'column';
robot_nominal.Gravity = [0, -9.80665, 0];
ee_name = robot_nominal.BodyNames{end};
robot = robot_nominal;

% Load Simulink model
load_system(model_ctc);

% Simulation timing
t_sim_stop = 14.0;
set_param(model_ctc, 'StopTime', num2str(t_sim_stop));

% Define test conditions:
% Compare CTC vs PID on the Pick-and-Place mission carrying a 0.5 kg object
payload_mass = 0.5; % 0.5 kg picked object
set_param([model_ctc '/Link2_1_RIGID/Payload_Solid'], 'Mass', 'payload_mass');
assignin('base', 'payload_mass', payload_mass);

% PID & CTC Controller Gains
global Kp_pid Kd_pid Ki_pid Kp_ctc Kd_ctc;
Kp_pid = [250; 400; 250];
Kd_pid = [35; 50; 35];
Ki_pid = [15; 25; 15];
Kp_ctc = [225; 225; 225];
Kd_ctc = [30; 30; 30];

assignin('base', 'Kp_pid', Kp_pid);
assignin('base', 'Kd_pid', Kd_pid);
assignin('base', 'Ki_pid', Ki_pid);
assignin('base', 'Kp_ctc', Kp_ctc);
assignin('base', 'Kd_ctc', Kd_ctc);

controllers = {'CTC', 'PID'};
ctrl_modes = [2, 1];
pnp_results = struct();

fprintf('--- Running Pick-and-Place Simulations (Payload = %.1f kg) ---\n', payload_mass);
for c = 1:length(controllers)
    cName = controllers{c};
    cMode = ctrl_modes(c);
    fprintf('Running %s controller ... ', cName);
    
    % Reset controller state
    clear robot_controller_fcn;
    global prev_t e_int;
    prev_t = [];
    e_int = zeros(3, 1);
    
    set_param([model_ctc '/ctrl_mode'], 'Value', num2str(cMode));
    assignin('base', 'ctrl_mode', cMode);
    
    simOut = sim(model_ctc, 'StopTime', num2str(t_sim_stop));
    
    t = simOut.q_out.Time;
    q = simOut.q_out.Data;
    qd = simOut.qd_out.Data;
    traj = simOut.traj_ref.Data;
    tau = simOut.tau_out.Data;
    
    qd_ref = traj(:, 1:3);
    qdot_ref = traj(:, 4:6);
    e = qd_ref - q;
    
    % Compute Forward Kinematics along trajectory
    N_pts = length(t);
    pos_actual = zeros(N_pts, 3);
    pos_desired = zeros(N_pts, 3);
    
    % Downsample FK computation if needed for speed
    step_skip = max(1, floor(N_pts / 300));
    sub_idx = 1:step_skip:N_pts;
    if sub_idx(end) ~= N_pts, sub_idx = [sub_idx, N_pts]; end
    
    pos_actual_sub = zeros(length(sub_idx), 3);
    pos_desired_sub = zeros(length(sub_idx), 3);
    t_sub = t(sub_idx);
    
    for i = 1:length(sub_idx)
        k_idx = sub_idx(i);
        T_act = getTransform(robot, q(k_idx, :)', ee_name);
        T_des = getTransform(robot, qd_ref(k_idx, :)', ee_name);
        pos_actual_sub(i, :) = T_act(1:3, 4)';
        pos_desired_sub(i, :) = T_des(1:3, 4)';
    end
    
    % Performance metrics
    rms_err = sqrt(mean(e.^2, 1));
    max_err = max(abs(e), [], 1);
    peak_tau = max(abs(tau), [], 1);
    
    % Station A (Pick) accuracy: evaluate during grasp dwell t in [3.5, 4.5]s
    dwell_A_idx = find(t >= 3.5 & t <= 4.5);
    err_A_rad = max(abs(e(dwell_A_idx, :)), [], 1);
    
    % Station B (Place) accuracy: evaluate during release dwell t in [10.0, 11.0]s
    dwell_B_idx = find(t >= 10.0 & t <= 11.0);
    err_B_rad = max(abs(e(dwell_B_idx, :)), [], 1);
    
    % Store
    pnp_results(c).Controller = cName;
    pnp_results(c).Time = t;
    pnp_results(c).q = q;
    pnp_results(c).qd = qd;
    pnp_results(c).q_ref = qd_ref;
    pnp_results(c).qdot_ref = qdot_ref;
    pnp_results(c).e = e;
    pnp_results(c).tau = tau;
    pnp_results(c).Time_sub = t_sub;
    pnp_results(c).pos_actual = pos_actual_sub;
    pnp_results(c).pos_desired = pos_desired_sub;
    pnp_results(c).RMS = rms_err;
    pnp_results(c).MaxErr = max_err;
    pnp_results(c).PeakTau = peak_tau;
    pnp_results(c).Err_A_deg = err_A_rad * 180 / pi;
    pnp_results(c).Err_B_deg = err_B_rad * 180 / pi;
    
    fprintf('Done. (RMS Joint Errors: [%.4f, %.4f, %.4f] rad)\n', ...
        rms_err(1), rms_err(2), rms_err(3));
end

%% Print Benchmark Summary Table
fprintf('\n=========================================================================================\n');
fprintf('                           PICK-AND-PLACE PERFORMANCE SUMMARY                            \n');
fprintf('=========================================================================================\n');
summary_rows = cell(2, 11);
for c = 1:2
    summary_rows(c, :) = {
        pnp_results(c).Controller, ...
        pnp_results(c).RMS(1), pnp_results(c).RMS(2), pnp_results(c).RMS(3), ...
        pnp_results(c).MaxErr(1), pnp_results(c).MaxErr(2), pnp_results(c).MaxErr(3), ...
        pnp_results(c).PeakTau(1), pnp_results(c).PeakTau(2), pnp_results(c).PeakTau(3), ...
        max(pnp_results(c).Err_A_deg)
    };
end
pnp_table = cell2table(summary_rows, 'VariableNames', { ...
    'Controller', ...
    'RMS_J1_rad', 'RMS_J2_rad', 'RMS_J3_rad', ...
    'MaxErr_J1_rad', 'MaxErr_J2_rad', 'MaxErr_J3_rad', ...
    'PeakTau_J1_Nm', 'PeakTau_J2_Nm', 'PeakTau_J3_Nm', ...
    'MaxPickErr_deg' ...
});
disp(pnp_table);

% Save results
save(fullfile(projectDir, 'pick_and_place_results.mat'), 'pnp_results', 'pnp_table');
writetable(pnp_table, fullfile(projectDir, 'pick_and_place_results.csv'));


%% Generate Visualizations
fprintf('\n--- Generating Pick-and-Place Visualization Figures ---\n');

% Figure 1: 3D Task-Space Cartesian Trajectory
fig1 = figure('Name', '3D Pick and Place Trajectory', 'Position', [100, 100, 950, 750], 'Visible', 'off');
hold on; grid on; box on;
view(45, 28);

% Draw desired path
plot3(pnp_results(1).pos_desired(:, 1), pnp_results(1).pos_desired(:, 2), pnp_results(1).pos_desired(:, 3), ...
    'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired EE Path');

% Draw CTC actual path
plot3(pnp_results(1).pos_actual(:, 1), pnp_results(1).pos_actual(:, 2), pnp_results(1).pos_actual(:, 3), ...
    '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 2.2, 'DisplayName', 'Actual Path (CTC)');

% Draw PID actual path
plot3(pnp_results(2).pos_actual(:, 1), pnp_results(2).pos_actual(:, 2), pnp_results(2).pos_actual(:, 3), ...
    '-.', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8, 'DisplayName', 'Actual Path (PID)');

% Key Cartesian waypoints
p_home  = pnp_results(1).pos_desired(1, :);
p_Ahov  = [0.309, 0.509, 0.585];
p_Apick = [0.289, 0.398, 0.640];
p_Bhov  = [0.023, 0.509, 0.467];
p_Bdrop = [-0.036, 0.398, 0.469];

% Markers
plot3(p_home(1), p_home(2), p_home(3), 'ks', 'MarkerSize', 11, 'MarkerFaceColor', [0.7 0.7 0.7], 'DisplayName', 'Home / Rest Pose');
plot3(p_Apick(1), p_Apick(2), p_Apick(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', [0.2 0.8 0.2], 'DisplayName', 'Point A (Pick Station)');
plot3(p_Bdrop(1), p_Bdrop(2), p_Bdrop(3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', [0.9 0.2 0.2], 'DisplayName', 'Point B (Place Station)');

% Annotation text
text(p_home(1)+0.02, p_home(2), p_home(3)+0.02, 'Home Pose', 'FontWeight', 'bold', 'FontSize', 9);
text(p_Apick(1)+0.02, p_Apick(2), p_Apick(3)+0.02, 'POINT A (Pick)', 'Color', [0 0.5 0], 'FontWeight', 'bold', 'FontSize', 10);
text(p_Bdrop(1)-0.05, p_Bdrop(2), p_Bdrop(3)+0.02, 'POINT B (Place)', 'Color', [0.8 0 0], 'FontWeight', 'bold', 'FontSize', 10);

xlabel('X Position (m)', 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontWeight', 'bold');
zlabel('Z Position (m)', 'FontWeight', 'bold');
title('3D Cartesian End-Effector Pick-and-Place Trajectory (Point A \rightarrow Point B)', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);

exportgraphics(fig1, fullfile(projectDir, 'pick_and_place_3d_trajectory.png'), 'Resolution', 150);
close(fig1);
fprintf('Saved pick_and_place_3d_trajectory.png\n');

% Figure 2: Cartesian Coordinates (X, Y, Z) vs Time
fig2 = figure('Name', 'Cartesian Coordinates vs Time', 'Position', [150, 150, 950, 700], 'Visible', 'off');
coords = {'X Position (m)', 'Y Position (m)', 'Z Position (m)'};
for c_i = 1:3
    subplot(3, 1, c_i); hold on; grid on; box on;
    % Shaded zones for Pick and Place
    y_lim_dummy = [min(pnp_results(1).pos_desired(:, c_i))-0.05, max(pnp_results(1).pos_desired(:, c_i))+0.05];
    patch([3.5 4.5 4.5 3.5], [y_lim_dummy(1) y_lim_dummy(1) y_lim_dummy(2) y_lim_dummy(2)], ...
        [0.85 0.95 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Pick Window (Pt A)');
    patch([10.0 11.0 11.0 10.0], [y_lim_dummy(1) y_lim_dummy(1) y_lim_dummy(2) y_lim_dummy(2)], ...
        [0.95 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Place Window (Pt B)');
    
    plot(pnp_results(1).Time_sub, pnp_results(1).pos_desired(:, c_i), 'k--', 'LineWidth', 2.0, 'DisplayName', 'Desired');
    plot(pnp_results(1).Time_sub, pnp_results(1).pos_actual(:, c_i), '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.8, 'DisplayName', 'CTC');
    plot(pnp_results(2).Time_sub, pnp_results(2).pos_actual(:, c_i), '-.', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.6, 'DisplayName', 'PID');
    
    ylabel(coords{c_i}, 'FontWeight', 'bold');
    if c_i == 1
        title('End-Effector Cartesian Trajectory Profile (Approach \rightarrow Pick \rightarrow Transfer \rightarrow Place)', ...
            'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'northeastoutside', 'FontSize', 8);
    end
    if c_i == 3, xlabel('Time (s)', 'FontWeight', 'bold'); end
end
exportgraphics(fig2, fullfile(projectDir, 'pick_and_place_cartesian_xyz.png'), 'Resolution', 150);
close(fig2);
fprintf('Saved pick_and_place_cartesian_xyz.png\n');

% Figure 3: Joint Tracking Errors vs Time (with Phase annotations)
fig3 = figure('Name', 'Joint Tracking Error vs Time', 'Position', [120, 120, 1000, 750], 'Visible', 'off');
for j = 1:3
    subplot(3, 1, j); hold on; grid on; box on;
    % Phase bands
    y_range = [min([pnp_results(1).e(:, j); pnp_results(2).e(:, j)])*1.2, ...
               max([pnp_results(1).e(:, j); pnp_results(2).e(:, j)])*1.2];
    if diff(y_range) == 0, y_range = [-0.01, 0.01]; end
    
    patch([3.5 4.5 4.5 3.5], [y_range(1) y_range(1) y_range(2) y_range(2)], ...
        [0.85 0.95 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Pick Window (Pt A)');
    patch([10.0 11.0 11.0 10.0], [y_range(1) y_range(1) y_range(2) y_range(2)], ...
        [0.95 0.85 0.85], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Place Window (Pt B)');
    
    plot(pnp_results(1).Time, pnp_results(1).e(:, j), '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.8, 'DisplayName', 'CTC Error');
    plot(pnp_results(2).Time, pnp_results(2).e(:, j), '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.6, 'DisplayName', 'PID Error');
    
    ylabel(sprintf('Joint %d Error (rad)', j), 'FontWeight', 'bold');
    if j == 1
        title('Joint Tracking Error During Pick-and-Place: CTC vs PID', 'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'northeastoutside', 'FontSize', 8);
    end
    if j == 3, xlabel('Time (s)', 'FontWeight', 'bold'); end
end
exportgraphics(fig3, fullfile(projectDir, 'pick_and_place_tracking.png'), 'Resolution', 150);
close(fig3);
fprintf('Saved pick_and_place_tracking.png\n');

% Figure 4: Joint Actuation Torques vs Time
fig4 = figure('Name', 'Joint Actuation Torques', 'Position', [150, 150, 1000, 750], 'Visible', 'off');
for j = 1:3
    subplot(3, 1, j); hold on; grid on; box on;
    plot(pnp_results(1).Time, pnp_results(1).tau(:, j), '-', 'Color', [0.00, 0.45, 0.74], 'LineWidth', 1.8, 'DisplayName', 'CTC Torque');
    plot(pnp_results(2).Time, pnp_results(2).tau(:, j), '--', 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.6, 'DisplayName', 'PID Torque');
    ylabel(sprintf('Joint %d Torque (N*m)', j), 'FontWeight', 'bold');
    if j == 1
        title('Joint Control Torques During Pick-and-Place Mission', 'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'northeastoutside', 'FontSize', 8);
    end
    if j == 3, xlabel('Time (s)', 'FontWeight', 'bold'); end
end
exportgraphics(fig4, fullfile(projectDir, 'pick_and_place_torques.png'), 'Resolution', 150);
close(fig4);
fprintf('Saved pick_and_place_torques.png\n');

close_system(model_ctc, 0);
close_system(model_nominal, 0);
fprintf('\nPick-and-Place mission execution and benchmarking completed successfully!\n');
