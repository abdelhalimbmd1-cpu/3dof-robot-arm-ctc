new_system('test_sim_zero_mass');
load_system('test_sim_zero_mass');
add_block('sm_lib/Body Elements/Brick Solid', 'test_sim_zero_mass/BS');
set_param('test_sim_zero_mass/BS', 'InertiaType', 'Custom');
set_param('test_sim_zero_mass/BS', 'Mass', '0');
set_param('test_sim_zero_mass/BS', 'MomentsOfInertia', '[0 0 0]');
try
    set_param('test_sim_zero_mass', 'SimulationCommand', 'update');
    disp('Mass = 0 compiled OK!');
catch ME
    disp(['Mass = 0 error: ' ME.message]);
end
close_system('test_sim_zero_mass', 0);
exit;
