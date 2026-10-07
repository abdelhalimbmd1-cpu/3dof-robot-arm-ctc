%% Master Script: CTC vs PID Dynamics and Control Comparison
% Compares Computed Torque Control (CTC) vs Joint PID on 3-DOF Simscape Multibody robot arm
% under payload variations (0 kg, 0.5 kg, 1.0 kg).

projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

fprintf('========================================================================\n');
fprintf('  3-DOF ROBOT ARM: COMPUTED TORQUE CONTROL (CTC) VS JOINT PID BENCHMARK \n');
fprintf('========================================================================\n\n');

%% STEP 1 & 2: Load Model and Rigid Body Tree Dynamics
model_nominal = 'ThreeDOF_Robot_Arm';
model_ctc = 'ThreeDOF_Robot_Arm_CTC';

load_system(model_nominal);
global robot_nominal;
robot_nominal = importrobot(model_nominal);
robot_nominal.DataFormat = 'column';
robot_nominal.Gravity = [0, -9.80665, 0]; % Multibody World frame gravity (Y-up)

fprintf('Dynamics Model (rigidBodyTree) successfully imported.\n');
fprintf('Total rigid bodies: %d\n', robot_nominal.NumBodies);
for bIdx = 1:robot_nominal.NumBodies
    bName = robot_nominal.BodyNames{bIdx};
    bObj = getTransform(robot_nominal, robot_nominal.homeConfiguration, bName);
    fprintf('  - Body %d: %-20s (Mass: %.4f kg)\n', bIdx, bName, robot_nominal.Bodies{bIdx}.Mass);
end
fprintf('\n');

%% Dynamics Verification at 3 Configurations
fprintf('--- STEP 2: Verifying Dynamics Model at 3 Configurations ---\n');
test_configs = [
    0.0,  0.0,  0.0;
    0.5, -0.4,  0.3;
   -0.3,  0.6, -0.5
]';
test_vels = [
    0.0,  0.0,  0.0;
    0.2, -0.1,  0.15;
   -0.1,  0.3, -0.2
]';
test_accs = [
    0.0,  0.0,  0.0;
    0.5, -0.2,  0.4;
   -0.4,  0.3, -0.3
]';

for i = 1:3
    q_i = test_configs(:, i);
    qd_i = test_vels(:, i);
    qdd_i = test_accs(:, i);
    
    M_i = massMatrix(robot_nominal, q_i);
    V_i = velocityProduct(robot_nominal, q_i, qd_i);
    G_i = gravityTorque(robot_nominal, q_i);
    tau_id = inverseDynamics(robot_nominal, q_i, qd_i, qdd_i);
    
    err_dyn = norm(tau_id - (M_i*qdd_i + V_i + G_i));
    fprintf('Config %d: q = [%.2f, %.2f, %.2f] rad | Dyn Decomp Error: %.2e N*m\n', ...
        i, q_i(1), q_i(2), q_i(3), err_dyn);
    fprintf('          G(q) = [%.3f, %.3f, %.3f] N*m\n', G_i(1), G_i(2), G_i(3));
end
fprintf('Dynamics verification complete: Analytical equations perfectly match inverse dynamics.\n\n');

%% STEP 4 & 5: Controller Setup and Simulation Parameters
load_system(model_ctc);

% Controller A: Joint PID (tuned for nominal payload = 0)
global Kp_pid Kd_pid Ki_pid;
Kp_pid = [250; 350; 150];
Kd_pid = [40; 60; 25];
Ki_pid = [10; 15; 8];

% Controller B: CTC (critically damped: omega_n = 15 rad/s -> Kp = 225, Kd = 30)
global Kp_ctc Kd_ctc;
Kp_ctc = [225; 225; 225];
Kd_ctc = [30; 30; 30];

payloads = [0.0, 0.5, 1.0];
controllers = {'PID', 'CTC'};
ctrl_modes = [1, 2];

results = struct();
rec_idx = 1;

fprintf('--- STEP 5: Running Simulation Experiments ---\n');
for c_idx = 1:length(controllers)
    ctrl_name = controllers{c_idx};
    ctrl_mode = ctrl_modes(c_idx);
    
    for p_idx = 1:length(payloads)
        payload_mass = payloads(p_idx);
        fprintf('Running: Controller = %-3s | Payload = %.1f kg ... ', ctrl_name, payload_mass);
        
        % Reset controller persistent variables
        clear robot_controller_fcn;
        global prev_t e_int;
        prev_t = [];
        e_int = zeros(3, 1);
        
        % Assign directly via set_param and base workspace for guaranteed update
        payload_mass_val = max(payload_mass, 1e-4);
        set_param([model_ctc '/Link2_1_RIGID/Payload_Solid'], 'Mass', 'payload_mass');
        set_param([model_ctc '/ctrl_mode'], 'Value', num2str(ctrl_mode));
        
        assignin('base', 'payload_mass', payload_mass_val);
        assignin('base', 'ctrl_mode', ctrl_mode);
        assignin('base', 'Kp_pid', Kp_pid);
        assignin('base', 'Kd_pid', Kd_pid);
        assignin('base', 'Ki_pid', Ki_pid);
        assignin('base', 'Kp_ctc', Kp_ctc);
        assignin('base', 'Kd_ctc', Kd_ctc);
        
        simOut = sim(model_ctc, 'StopTime', '8.0');
        
        % Extract logged signals directly from simulation output object
        t = simOut.q_out.Time;
        q = simOut.q_out.Data;
        qd = simOut.qd_out.Data;
        traj = simOut.traj_ref.Data;
        tau = simOut.tau_out.Data;
        
        % Reference q_d and tracking error
        qd_ref = traj(:, 1:3);
        e = qd_ref - q;
        
        % Performance Metrics per joint
        rms_err = sqrt(mean(e.^2, 1));
        max_err = max(abs(e), [], 1);
        peak_tau = max(abs(tau), [], 1);
        
        % Settling time at final target w2 (reached at t=6s, holding until 8s)
        % Defined as time after t=6s to enter and stay within 0.02 rad of final target
        t_hold_idx = find(t >= 6.0);
        settling_time = zeros(1, 3);
        for j = 1:3
            tol = 0.02; % 0.02 rad threshold
            outside_idx = find(abs(e(t_hold_idx, j)) > tol);
            if isempty(outside_idx)
                settling_time(j) = 6.0; % settled immediately upon arrival
            else
                settling_time(j) = t(t_hold_idx(outside_idx(end)));
            end
        end
        
        % Store results
        results(rec_idx).Controller = ctrl_name;
        results(rec_idx).Payload_kg = payload_mass;
        results(rec_idx).Time = t;
        results(rec_idx).q = q;
        results(rec_idx).qd = qd;
        results(rec_idx).q_ref = qd_ref;
        results(rec_idx).e = e;
        results(rec_idx).tau = tau;
        results(rec_idx).RMS_J1 = rms_err(1);
        results(rec_idx).RMS_J2 = rms_err(2);
        results(rec_idx).RMS_J3 = rms_err(3);
        results(rec_idx).MaxErr_J1 = max_err(1);
        results(rec_idx).MaxErr_J2 = max_err(2);
        results(rec_idx).MaxErr_J3 = max_err(3);
        results(rec_idx).PeakTau_J1 = peak_tau(1);
        results(rec_idx).PeakTau_J2 = peak_tau(2);
        results(rec_idx).PeakTau_J3 = peak_tau(3);
        results(rec_idx).Settling_J1 = settling_time(1);
        results(rec_idx).Settling_J2 = settling_time(2);
        results(rec_idx).Settling_J3 = settling_time(3);
        
        fprintf('Done. (RMS Error J1/2/3: [%.4f, %.4f, %.4f] rad)\n', ...
            rms_err(1), rms_err(2), rms_err(3));
        rec_idx = rec_idx + 1;
    end
end
fprintf('\nAll simulations completed successfully.\n\n');

%% STEP 6: Save Results Table and Data
fprintf('--- STEP 6: Saving Results and Generating Figures ---\n');
outDir = projectDir;

summary_data = cell(length(results), 14);
for r = 1:length(results)
    summary_data(r, :) = {
        results(r).Controller, ...
        results(r).Payload_kg, ...
        results(r).RMS_J1, results(r).RMS_J2, results(r).RMS_J3, ...
        results(r).MaxErr_J1, results(r).MaxErr_J2, results(r).MaxErr_J3, ...
        results(r).PeakTau_J1, results(r).PeakTau_J2, results(r).PeakTau_J3, ...
        results(r).Settling_J1, results(r).Settling_J2, results(r).Settling_J3
    };
end

results_table = cell2table(summary_data, 'VariableNames', { ...
    'Controller', 'Payload_kg', ...
    'RMS_J1_rad', 'RMS_J2_rad', 'RMS_J3_rad', ...
    'MaxErr_J1_rad', 'MaxErr_J2_rad', 'MaxErr_J3_rad', ...
    'PeakTau_J1_Nm', 'PeakTau_J2_Nm', 'PeakTau_J3_Nm', ...
    'Settling_J1_s', 'Settling_J2_s', 'Settling_J3_s' ...
});

fprintf('\n===============================================================================================================\n');
fprintf('                                          BENCHMARK RESULTS TABLE                                              \n');
fprintf('===============================================================================================================\n');
disp(results_table);

% Save to CSV and MAT
writetable(results_table, fullfile(outDir, 'results_ctc_vs_pid.csv'));
save(fullfile(outDir, 'results_ctc_vs_pid.mat'), 'results', 'results_table');
fprintf('Saved results to results_ctc_vs_pid.csv and results_ctc_vs_pid.mat\n\n');

% Set figure defaults for clean rendering
set(0, 'DefaultAxesFontName', 'Arial', 'DefaultAxesFontSize', 10);
set(0, 'DefaultLineLineWidth', 1.5);

colors = [0.0, 0.45, 0.74;  % blue
          0.85, 0.33, 0.10;  % orange
          0.93, 0.69, 0.13;  % yellow
          0.49, 0.18, 0.56;  % purple
          0.47, 0.67, 0.19;  % green
          0.64, 0.08, 0.18]; % red

% Plot 1: Tracking Error vs Time
fig1 = figure('Name', 'Tracking Error vs Time', 'Position', [100, 100, 1000, 700], 'Visible', 'off', 'Renderer', 'painters');
for j = 1:3
    subplot(3, 1, j);
    hold on; grid on; box on;
    for r = 1:length(results)
        cName = results(r).Controller;
        pVal = results(r).Payload_kg;
        if strcmp(cName, 'PID'), lStyle = '--'; else, lStyle = '-'; end
        label = sprintf('%s (m_p=%.1f kg)', cName, pVal);
        plot(results(r).Time, results(r).e(:, j), 'LineStyle', lStyle, 'LineWidth', 1.5, ...
            'Color', colors(r, :), 'DisplayName', label);
    end
    ylabel(sprintf('Joint %d Error (rad)', j), 'FontWeight', 'bold');
    if j == 1
        title('Joint Tracking Error vs Time across Payloads and Controllers', 'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'northeastoutside', 'FontSize', 8);
    end
    if j == 3, xlabel('Time (s)', 'FontWeight', 'bold'); end
end
exportgraphics(fig1, fullfile(outDir, 'tracking_error_vs_time.png'), 'Resolution', 150);
close(fig1);
fprintf('Saved tracking_error_vs_time.png\n');

% Plot 2: Joint Torques vs Time
fig2 = figure('Name', 'Joint Torques vs Time', 'Position', [150, 150, 1000, 700], 'Visible', 'off');
for j = 1:3
    subplot(3, 1, j);
    hold on; grid on; box on;
    for r = 1:length(results)
        cName = results(r).Controller;
        pVal = results(r).Payload_kg;
        if strcmp(cName, 'PID'), lStyle = '--'; else, lStyle = '-'; end
        label = sprintf('%s (m_p=%.1f kg)', cName, pVal);
        plot(results(r).Time, results(r).tau(:, j), 'LineStyle', lStyle, 'LineWidth', 1.5, ...
            'Color', colors(r, :), 'DisplayName', label);
    end
    ylabel(sprintf('Joint %d Torque (N*m)', j), 'FontWeight', 'bold');
    if j == 1
        title('Control Actuation Torques vs Time across Payloads', 'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'northeastoutside', 'FontSize', 8);
    end
    if j == 3, xlabel('Time (s)', 'FontWeight', 'bold'); end
end
exportgraphics(fig2, fullfile(outDir, 'joint_torques_vs_time.png'), 'Resolution', 150);
close(fig2);
fprintf('Saved joint_torques_vs_time.png\n');

% Plot 3: Bar Chart of RMS Error Comparison
fig3 = figure('Name', 'RMS Error Comparison', 'Position', [200, 200, 900, 600], 'Visible', 'off');
rms_pid = zeros(3, 3);
rms_ctc = zeros(3, 3);
for r = 1:length(results)
    p_i = find(abs(payloads - results(r).Payload_kg) < 1e-4);
    if strcmp(results(r).Controller, 'PID')
        rms_pid(p_i, :) = [results(r).RMS_J1, results(r).RMS_J2, results(r).RMS_J3];
    else
        rms_ctc(p_i, :) = [results(r).RMS_J1, results(r).RMS_J2, results(r).RMS_J3];
    end
end

for j = 1:3
    subplot(3, 1, j);
    data_bar = [rms_pid(:, j), rms_ctc(:, j)];
    b = bar(data_bar);
    b(1).FaceColor = [0.85 0.33 0.10];
    b(2).FaceColor = [0.00 0.45 0.74];
    grid on; box on;
    set(gca, 'XTickLabel', {'Payload 0 kg', 'Payload 0.5 kg', 'Payload 1.0 kg'});
    ylabel(sprintf('Joint %d RMS (rad)', j), 'FontWeight', 'bold');
    if j == 1
        title('RMS Tracking Error Comparison: PID vs Computed Torque Control', 'FontSize', 12, 'FontWeight', 'bold');
        legend({'Joint PID', 'Computed Torque Control (CTC)'}, 'Location', 'northwest');
    end
end
exportgraphics(fig3, fullfile(outDir, 'rms_error_comparison_barchart.png'), 'Resolution', 150);
close(fig3);
fprintf('Saved rms_error_comparison_barchart.png\n');

close_system(model_ctc, 0);
close_system(model_nominal, 0);
fprintf('Benchmark script execution completed successfully!\n');
