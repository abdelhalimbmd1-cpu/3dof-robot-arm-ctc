new_system('test_bs');
load_system('test_bs');
add_block('sm_lib/Body Elements/Brick Solid', 'test_bs/BS');
dp = get_param('test_bs/BS', 'DialogParameters');
f = fieldnames(dp);
disp('Brick Solid parameters:');
disp(f(1:min(20, end)));
for i = 1:length(f)
    if contains(lower(f{i}), 'mass') || contains(lower(f{i}), 'inertia')
        fprintf('  %s (Prompt: %s)\n', f{i}, dp.(f{i}).Prompt);
    end
end
close_system('test_bs', 0);
exit;
