%% Import 3-DOF Robotic Arm into Simscape Multibody
% This script imports the SolidWorks-exported XML into Simulink Simscape Multibody.

% 1. Ensure we are in the directory containing the exported XML and STEP files
scriptDir = fileparts(mfilename('fullpath'));
cd(scriptDir);

% 2. Search for exported XML files
xmlFiles = dir(fullfile(scriptDir, '*.xml'));

if isempty(xmlFiles)
    error('No XML file found in %s.\nPlease perform the export from SolidWorks first:\n Tools -> Simscape Multibody Link -> Export -> Simscape Multibody...', scriptDir);
end

% Use the most recently modified XML file if multiple exist
[~, idx] = max([xmlFiles.datenum]);
targetXml = fullfile(scriptDir, xmlFiles(idx).name);
fprintf('Found exported XML: %s\n', targetXml);
fprintf('Importing model into Simscape Multibody...\n');

% 3. Run smimport
% smimport generates the Simulink model and opens the Mechanics Explorer
modelHandle = smimport(targetXml);

fprintf('Import successful! Model generated.\n');
fprintf('Tips:\n');
fprintf(' - You can save the generated model using: save_system(gcs, ''ThreeDOF_Robot_Arm.slx'')\n');
fprintf(' - Open joint blocks to enable Actuation (Torque/Motion) and Sensing.\n');
