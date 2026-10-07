% record_mechanics_explorer_video.m
% =========================================================================
%   RECORD 3-DOF ROBOT SIMULATION VIDEO FROM SIMSCAPE MECHANICS EXPLORER
% =========================================================================
% This script runs the simulation in your active MATLAB/Simulink session
% and automatically records the 3D CAD visualization from Mechanics Explorer
% into an MP4 video (matching '3 DOF robot.mp4' format).
% =========================================================================

cd('C:\Users\ZORO\Downloads\3-dof-robotic-arm-3.snapshot.1\3 DOF concept\Design');

modelName = 'ThreeDOF_Robot_Arm_CTC';
videoName = '3 DOF robot simulation.mp4';

fprintf('Opening %s in Simulink...\n', modelName);
open_system(modelName);

% Set simulation parameters
set_param(modelName, 'StopTime', '14.0');
assignin('base', 'ctrl_mode', 2);
assignin('base', 'payload_mass', 0.5);

fprintf('Starting simulation in Simulink to populate Mechanics Explorer...\n');
set_param(modelName, 'SimulationCommand', 'start');

% Wait for the simulation to finish
fprintf('Simulating Pick-and-Place mission... Please wait.\n');
while ~strcmp(get_param(modelName, 'SimulationStatus'), 'stopped')
    pause(0.5);
end
fprintf('Simulation complete! Mechanics Explorer is ready.\n\n');

% Export video directly from Mechanics Explorer
fprintf('Recording video via smwritevideo to: %s\n', videoName);
try
    smwritevideo(modelName, videoName, ...
        'VideoFormat', 'mpeg-4', ...
        'FrameSize', [1400, 1080], ...
        'FrameRate', 15, ...
        'PlaybackSpeedRatio', 1.0);
    
    fprintf('SUCCESS! Video recorded and saved to:\n  %s\n', ...
        fullfile(pwd, videoName));
catch ME
    fprintf('Could not record automatically via script: %s\n\n', ME.message);
    fprintf('You can record it with 2 clicks from the Mechanics Explorer window:\n');
    fprintf('  1. In the Mechanics Explorer window, click Tools -> Video Creator (or the camera icon).\n');
    fprintf('  2. Set Format to MPEG-4, Frame Size to 1400 x 1080, and click Create Video!\n');
end
