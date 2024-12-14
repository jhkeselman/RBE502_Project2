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

pos = qd{qn}.pos; pos_des = qd{qn}.pos_des;
vel = qd{qn}.vel; vel_des = qd{qn}.vel_des;
                  acc_des = qd{qn}.acc_des;
euler = qd{qn}.euler;
omega = qd{qn}.omega;
yaw_des = qd{qn}.yaw_des;
yawdot_des = qd{qn}.yawdot_des;

g = params.grav;
m = params.mass;
I11 = params.I(1,1);
I22 = params.I(2,2);
I33 = params.I(3,3);

A = [0         0         0    1.0000         0         0         0         0         0         0         0         0
     0         0         0         0    1.0000         0         0         0         0         0         0         0
     0         0         0         0         0    1.0000         0         0         0         0         0         0
     0         0         0         0         0         0         0         g         0         0         0         0
     0         0         0         0         0         0        -g         0         0         0         0         0
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
     1/m       0         0         0
     0         0         0         0
     0         0         0         0
     0         0         0         0
     0     1/I11         0         0
     0         0     1/I22         0
     0         0         0    1/I33];

Q = diag([1,1,1,.1,.1,.1,1,1,1,.001,.001,.001]);
R = diag([0.01,1,1,1]);
K = lqr(A, B, Q, R);

x_current = [pos; vel; euler; omega];
x_desired = [pos_des; vel_des; [0, 0, yaw_des]'; [0, 0, yawdot_des]'];

u = -K*(x_current - x_desired);

F = u(1);
M = u(2:4);


%Output trpy and drpy as in hardware
trpy = [0, 0, 0, 0];
drpy = [0, 0, 0, 0];

end
% 
% 
% % function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
% % % CONTROLLER quadrotor controller
% % % The current states are:
% % % qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% % % The desired states are:
% % % qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% % % Using these current and desired states, you have to compute the desired controls
% % 
% % % =================== Your code goes here ===================
% % persistent gd;
% % persistent icnt;
% %  if isempty(gd)
% %      gd = zeros(0,3);
% %      icnt = 0;
% %  end
% %  icnt = icnt + 1;
% % 
% % %% Parameter Initialization
% % if ~isempty(t)
% % desired_state = trajhandle(t, qn);
% % end
% % 
% % g = params.grav;
% % m = params.mass;
% % I = params.I;
% % 
% % e_pos = qd{qn}.pos - desired_state.pos;
% % e_vel = qd{qn}.vel - desired_state.vel;
% % 
% % x_current = [e_pos; e_vel; qd{qn}.euler; qd{qn}.omega];
% % x_desired = [desired_state.pos; desired_state.vel; [0; 0; desired_state.yaw]; zeros(3, 1)];
% % 
% % persistent K;
% % if isempty(K)
% % A = [0         0         0    1.0000         0         0         0         0         0         0         0         0
% %      0         0         0         0    1.0000         0         0         0         0         0         0         0
% %      0         0         0         0         0    1.0000         0         0         0         0         0         0
% %      0         0         0         0         0         0         0    9.8100         0         0         0         0
% %      0         0         0         0         0         0   -9.8100         0         0         0         0         0
% %      0         0         0         0         0         0         0         0         0         0         0         0
% %      0         0         0         0         0         0         0         0         0    1.0000         0         0
% %      0         0         0         0         0         0         0         0         0         0    1.0000         0
% %      0         0         0         0         0         0         0         0         0         0         0    1.0000
% %      0         0         0         0         0         0         0         0         0         0         0         0
% %      0         0         0         0         0         0         0         0         0         0         0         0
% %      0         0         0         0         0         0         0         0         0         0         0         0];
% %  B = [0         0         0         0
% %       0         0         0         0
% %       0         0         0         0
% %       0         0         0         0
% %       0         0         0         0
% % 33.3333         0         0         0
% %       0         0         0         0
% %       0         0         0         0
% %       0         0         0         0
% %       0    0.6993         0         0
% %       0         0    0.6993         0
% %       0         0         0    0.3460];
% %     Q = diag([5, 5, 5, 0.1, 0.5, 0.1, 2.5, 2.5, 2.5, 0.05, 0.05, 0.05]);
% %     R = diag([0.01, 0.15, 0.15, 0.15]);
% %     K = lqr_solver(A, B, Q, R)
% % end
% % 
% % u = -K*(x_current - x_desired);
% % 
% % F = u(1);
% % M = u(2:4);
% % 
% % 
% % %Output trpy and drpy as in hardware
% % trpy = [0, 0, 0, 0];
% % drpy = [0, 0, 0, 0];
% % 
% % end
