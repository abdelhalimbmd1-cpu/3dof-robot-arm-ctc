% create_simulation_video.m
% Generates an MP4 video and animated GIF of the 3D Pick-and-Place simulation
projectDir = fileparts(mfilename('fullpath'));
if isempty(projectDir), projectDir = pwd; end
cd(projectDir);

% Load trajectory data
load('pick_and_place_results.mat');
t_all = pnp_results(1).Time;
q_all = pnp_results(1).q; % CTC trajectory

% Load robot model
load_system('ThreeDOF_Robot_Arm');
robot = importrobot('ThreeDOF_Robot_Arm');
robot.DataFormat = 'column';
ee_name = robot.BodyNames{end};

% Sample at 15 fps for smooth, fast rendering (14 seconds -> ~140 frames)
fps = 10;
t_video = 0:(1/fps):t_all(end);
N_frames = length(t_video);

q_interp = interp1(t_all, q_all, t_video);

% Setup figure
fig = figure('Position', [100, 100, 800, 600], 'Visible', 'off', 'Color', 'w');
ax = axes('Parent', fig);

% Output video writers
mp4_file = 'pick_and_place_simulation.mp4';
gif_file = 'pick_and_place_animation.gif';

try
    v_writer = VideoWriter(mp4_file, 'MPEG-4');
    v_writer.FrameRate = fps;
    open(v_writer);
    has_mp4 = true;
catch
    has_mp4 = false;
end

p_Apick = [0.289, 0.398, 0.640];
p_Bdrop = [-0.036, 0.398, 0.469];
ee_trail = zeros(N_frames, 3);

fprintf('Rendering %d frames for video ...\n', N_frames);

for k = 1:N_frames
    t_curr = t_video(k);
    q_curr = q_interp(k, :)';
    
    cla(ax);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    
    % Show robot
    show(robot, q_curr, 'Parent', ax, 'Visuals', 'on', 'PreservePlot', false);
    
    % Compute EE position for trail
    T_ee = getTransform(robot, q_curr, ee_name);
    ee_trail(k, :) = T_ee(1:3, 4)';
    
    % Plot Trail
    if k > 1
        plot3(ax, ee_trail(1:k, 1), ee_trail(1:k, 2), ee_trail(1:k, 3), ...
            'b-', 'LineWidth', 2.0);
    end
    
    % Plot Point A (Pick Station) and Point B (Place Station)
    plot3(ax, p_Apick(1), p_Apick(2), p_Apick(3), 'go', 'MarkerSize', 12, 'MarkerFaceColor', [0.2 0.8 0.2]);
    plot3(ax, p_Bdrop(1), p_Bdrop(2), p_Bdrop(3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', [0.9 0.2 0.2]);
    
    text(ax, p_Apick(1)+0.02, p_Apick(2), p_Apick(3)+0.03, 'Point A (Pick)', 'Color', [0 0.5 0], 'FontWeight', 'bold');
    text(ax, p_Bdrop(1)-0.05, p_Bdrop(2), p_Bdrop(3)+0.03, 'Point B (Place)', 'Color', [0.8 0 0], 'FontWeight', 'bold');
    
    % Determine Mission Phase Name
    if t_curr < 2.0
        phase_str = 'Phase 1: Approach Pick Station A';
    elseif t_curr < 3.5
        phase_str = 'Phase 2: Descend to Object at Point A';
    elseif t_curr < 4.5
        phase_str = 'Phase 3: [PICK] Grasp Object at Point A';
    elseif t_curr < 6.0
        phase_str = 'Phase 4: Lift Object to Clearance Height';
    elseif t_curr < 8.5
        phase_str = 'Phase 5: [TRANSFER] Swing across to Station B';
    elseif t_curr < 10.0
        phase_str = 'Phase 6: Descend to Place Fixture at Point B';
    elseif t_curr < 11.0
        phase_str = 'Phase 7: [PLACE] Release Object at Point B';
    elseif t_curr < 12.0
        phase_str = 'Phase 8: Retract / Lift from Point B';
    else
        phase_str = 'Phase 9: Return to Standby Home Pose';
    end
    
    title(ax, { ...
        sprintf('3-DOF Robot Arm: Pick-and-Place (Point A \\rightarrow Point B)'), ...
        sprintf('Time: %.1f s / 14.0 s  |  %s', t_curr, phase_str) ...
    }, 'FontSize', 11, 'FontWeight', 'bold');
    
    xlabel(ax, 'X (m)'); ylabel(ax, 'Y (m)'); zlabel(ax, 'Z (m)');
    view(ax, 50, 22);
    axis(ax, [-0.3, 0.5, 0.1, 0.7, 0.2, 0.8]);
    
    drawnow;
    frame = getframe(fig);
    im = frame2im(frame);
    
    % Write MP4
    if has_mp4
        writeVideo(v_writer, frame);
    end
    
    % Write GIF
    [imind, cm] = rgb2ind(im, 256);
    if k == 1
        imwrite(imind, cm, gif_file, 'gif', 'Loopcount', inf, 'DelayTime', 1/fps);
    else
        imwrite(imind, cm, gif_file, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
    end
    
    if mod(k, 30) == 0
        fprintf('Frame %d / %d rendered...\n', k, N_frames);
    end
end

if has_mp4
    close(v_writer);
    fprintf('Saved %s\n', mp4_file);
end
close(fig);
close_system('ThreeDOF_Robot_Arm', 0);
fprintf('Saved %s\n', gif_file);

fprintf('Video and animation generation finished successfully!\n');
