% Parameters
m = 0.03;   % Mass of the quadrotor (kg)
g = 9.81;  % Gravity (m/s^2)
L = 0.046;  % Distance from center to rotor (m)
I = diag([1.43, 1.43, 2.89]); % Inertia matrix (kg.m^2)
kF = 6.11e-8; % Thrust coefficient
kM = 1.5e-9;  % Moment coefficient

% Symbolic Variables
syms r1 r2 r3 dr1 dr2 dr3 phi theta psi p q r real
syms u1 tau_phi tau_theta tau_psi real

% State and Input Vectors
x = [r1; r2; r3; dr1; dr2; dr3; phi; theta; psi; p; q; r];
u = [u1; tau_phi; tau_theta; tau_psi];

% Rotation Matrix
R = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi);
     sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi), -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi);
     -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

% Translational Dynamics
ddr = [0; 0; -m*g] / m + R * [0; 0; u1] / m;

% Rotational Dynamics
omega = [p; q; r];
tau = [tau_phi; tau_theta; tau_psi];
domega = I \ (tau - cross(omega, I * omega));

% Orientation Kinematics
E = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
     0, cos(phi), -sin(phi);
     0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
dangles = E * omega;

% Full Dynamics
f = [dr1; dr2; dr3; ddr; dangles; domega];

% Linearization
A = jacobian(f, x);
B = jacobian(f, u);

% Evaluate at hover
x_hover = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
u_hover = [m*g; 0; 0; 0];
A = double(subs(A, [x; u], [x_hover; u_hover]));
B = double(subs(B, [x; u], [x_hover; u_hover]));

% Display Results
disp('A Matrix:');
disp(A);
disp('B Matrix:');
disp(B);
