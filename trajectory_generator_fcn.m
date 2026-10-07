function traj = trajectory_generator_fcn(t)
% TRAJECTORY_GENERATOR_FCN
% Generates a smooth, continuous C^2 quintic polynomial Pick-and-Place trajectory
% Mission:
%   Phase 1 (0.0 - 2.0s):  Home -> Hover above Pick Station A
%   Phase 2 (2.0 - 3.5s):  Hover A -> Pick Point A (Descend onto object)
%   Phase 3 (3.5 - 4.5s):  Dwell at Point A (Grasp object)
%   Phase 4 (4.5 - 6.0s):  Pick Point A -> Lift to clearance height
%   Phase 5 (6.0 - 8.5s):  Lift A -> Hover above Place Station B (Transfer)
%   Phase 6 (8.5 - 10.0s): Hover B -> Place Point B (Descend onto drop fixture)
%   Phase 7 (10.0 - 11.0s): Dwell at Point B (Release object)
%   Phase 8 (11.0 - 12.0s): Place Point B -> Lift to clearance height
%   Phase 9 (12.0 - 14.0s): Lift B -> Return to Home / Standby
%
% Output: traj = [q_d(1:3); qd_d(1:3); qdd_d(1:3)] (9x1 vector)

% Joint waypoints [q1; q2; q3] (rad)
% Physical CAD Initial Assembly Configuration: [-2.6763; -1.8134; 0.4091]
w_home        = [-2.6763; -1.8134; 0.4091];
w_A_hover     = [-2.8000; -1.4000; 0.5000];
w_A_pick      = [-2.8000; -1.0000; 0.9000];
w_B_hover     = [-1.6000; -1.4000; 0.5000];
w_B_place     = [-1.6000; -1.0000; 0.9000];

times = [0.0, 2.0, 3.5, 4.5, 6.0, 8.5, 10.0, 11.0, 12.0, 14.0];
waypoints = [
    w_home, ...       % 1: t=0.0s (Home)
    w_A_hover, ...    % 2: t=2.0s (Hover A)
    w_A_pick, ...     % 3: t=3.5s (Pick Point A)
    w_A_pick, ...     % 4: t=4.5s (Dwell Grasp A)
    w_A_hover, ...    % 5: t=6.0s (Lift A)
    w_B_hover, ...    % 6: t=8.5s (Hover B)
    w_B_place, ...    % 7: t=10.0s (Place Point B)
    w_B_place, ...    % 8: t=11.0s (Dwell Release B)
    w_B_hover, ...    % 9: t=12.0s (Lift B)
    w_home            % 10: t=14.0s (Return Home)
];

if t <= times(1)
    q_d = waypoints(:, 1);
    qd_d = zeros(3, 1);
    qdd_d = zeros(3, 1);
elseif t >= times(end)
    q_d = waypoints(:, end);
    qd_d = zeros(3, 1);
    qdd_d = zeros(3, 1);
else
    % Find active interval
    k = 1;
    for i = 1:(length(times) - 1)
        if t >= times(i) && t < times(i+1)
            k = i;
            break;
        end
    end
    
    t0 = times(k);
    t1 = times(k+1);
    dt = t1 - t0;
    
    tau = (t - t0) / dt;
    % Clamp tau to [0, 1] for numerical safety
    tau = max(0.0, min(1.0, tau));
    
    % Quintic polynomial blending
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    sd = (30*tau^2 - 60*tau^3 + 30*tau^4) / dt;
    sdd = (60*tau - 180*tau^2 + 120*tau^3) / (dt^2);
    
    delta_w = waypoints(:, k+1) - waypoints(:, k);
    
    q_d = waypoints(:, k) + s * delta_w;
    qd_d = sd * delta_w;
    qdd_d = sdd * delta_w;
end

traj = [q_d; qd_d; qdd_d];
end
