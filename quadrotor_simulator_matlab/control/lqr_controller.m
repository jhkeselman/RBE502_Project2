function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
persistent gd;
persistent icnt;
 if isempty(gd)
     gd = zeros(0,3);
     icnt = 0;
 end
 icnt = icnt + 1;

%% Parameter Initialization
if ~isempty(t)
desired_state = trajhandle(t, qn);
end

g = params.grav;
m = params.mass;
I = params.I;

e_pos = qd{qn}.pos - desired_state.pos;
e_vel = qd{qn}.vel - desired_state.vel;

x_current = [e_pos; e_vel; qd{qn}.euler; qd{qn}.omega];
x_desired = [desired_state.pos; desired_state.vel; [0; 0; desired_state.yaw]; zeros(3, 1)];

persistent K;
if isempty(K)
    A = [0         0         0    1.0000         0         0         0         0         0         0         0         0
         0         0         0         0    1.0000         0         0         0         0         0         0         0
         0         0         0         0         0    1.0000         0         0         0         0         0         0
         0         0         0         0         0         0         0    9.8100         0         0         0         0
         0         0         0         0         0         0   -9.8100         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0    1.0000         0         0
         0         0         0         0         0         0         0         0         0         0    1.0000         0
         0         0         0         0         0         0         0         0         0         0         0    1.0000
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0
         0         0         0         0         0         0         0         0         0         0         0         0];
    B = [0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
   33.3333         0         0         0
         0         0         0         0
         0         0         0         0
         0         0         0         0
         0    0.6993         0         0
         0         0    0.6993         0
         0         0         0    0.3460];
    Q = diag([0.8, 0.8, 0.8, 0.1, 0.5, 0.1, 1, 1, 1, 0.03, 0.03, 0.03]);
    R= diag([0.01, 0.15, 0.15, 0.15]);
    K = lqrd(A, B, Q, R, 0.01);
end

u = -K*(x_current - x_desired);

F = u(1);
M = u(2:4);


%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
