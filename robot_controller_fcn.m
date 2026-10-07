function tau = robot_controller_fcn(u)
% Controller block: PID (mode=1) or Computed Torque Control (mode=2)
% u: [q(1:3); qd(1:3); q_d(1:3); qd_d(1:3); qdd_d(1:3); ctrl_mode; t]
global robot_nominal Kp_pid Kd_pid Ki_pid Kp_ctc Kd_ctc prev_t e_int;

u = u(:);

q = u(1:3);
qd = u(4:6);
q_d = u(7:9);
qd_d = u(10:12);
qdd_d = u(13:15);
ctrl_mode = round(u(16));
t = u(17);

% Default gains if not set globally
if isempty(Kp_pid), try Kp_pid = evalin('base', 'Kp_pid'); catch, Kp_pid = [250; 350; 150]; end, end
if isempty(Kd_pid), try Kd_pid = evalin('base', 'Kd_pid'); catch, Kd_pid = [40; 60; 25]; end, end
if isempty(Ki_pid), try Ki_pid = evalin('base', 'Ki_pid'); catch, Ki_pid = [10; 15; 8]; end, end
if isempty(Kp_ctc), try Kp_ctc = evalin('base', 'Kp_ctc'); catch, Kp_ctc = [225; 225; 225]; end, end
if isempty(Kd_ctc), try Kd_ctc = evalin('base', 'Kd_ctc'); catch, Kd_ctc = [30; 30; 30]; end, end

if isempty(prev_t) || t < prev_t
    prev_t = t;
    e_int = zeros(3, 1);
elseif t > prev_t
    dt = t - prev_t;
    if dt < 0.05
        e_int = e_int + (q_d - q) * dt;
        e_int = max(min(e_int, 5.0), -5.0); % Anti-windup clamping
    end
    prev_t = t;
end

e = q_d - q;
ed = qd_d - qd;

if ctrl_mode == 1
    % Controller A: Joint PID (tuned for payload = 0)
    kp = Kp_pid(:);
    kd = Kd_pid(:);
    ki = Ki_pid(:);
    tau = kp .* e + kd .* ed + ki .* e_int;
else
    % Controller B: Computed Torque Control
    % tau = M(q)*(qdd_d + Kd*(qd_d - qd) + Kp*(q_d - q)) + C(q, qd)*qd + G(q)
    kp_ctc = Kp_ctc(:);
    kd_ctc = Kd_ctc(:);
    qdd_star = qdd_d + kd_ctc .* ed + kp_ctc .* e;
    
    if isempty(robot_nominal)
        if ~bdIsLoaded('ThreeDOF_Robot_Arm')
            load_system('ThreeDOF_Robot_Arm');
        end
        robot_nominal = importrobot('ThreeDOF_Robot_Arm');
        robot_nominal.DataFormat = 'column';
        robot_nominal.Gravity = [0, -9.80665, 0];
    end
    
    % Inverse dynamics evaluates M(q)*qdd* + C(q, qd)*qd + G(q)
    tau = inverseDynamics(robot_nominal, q, qd, qdd_star);
end

% Ensure column vector of length 3
tau = reshape(tau, 3, 1);
end
