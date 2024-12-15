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
    
    pos = qd{qn}.pos;
    vel = qd{qn}.vel;
    euler = qd{qn}.euler;
    omega = qd{qn}.omega;
    
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
    
    Q = diag([1.2,1.2,1.2,.1,.1,.1,1,1,1,.001,.001,.001]);
    R = diag([0.005,1,1,1]);
    K = lqr(A, B, Q, R);
    
    x_current = [pos; vel; euler; omega];
    x_desired = [desired_state.pos; desired_state.vel; [0, 0, desired_state.yaw]'; [0, 0, desired_state.yawdot]'];
    
    u = -K*(x_current - x_desired);
    
    F = u(1);
    M = u(2:4);
    
    
    %Output trpy and drpy as in hardware
    trpy = [0, 0, 0, 0];
    drpy = [0, 0, 0, 0];

end
