% Simulation parameters
tf = 15;           % Total simulation time (seconds)
f = 3;             % Controller frequency (Hz)
dt = 1 / f;        % Time step (seconds)
t = 0:dt:tf;       % Time vector

% Joint limits and initial/final positions
theta1_initial = 0;   % Initial position of theta1 (rad)
theta1_final = pi/2;  % Final position of theta1 (rad)

rho_initial = 25;     % Initial position of rho (mm)
rho_final = 75;       % Final position of rho (mm)

theta2_initial = 0;   % Initial position of theta2 (rad)
theta2_final = pi/3;  % Final position of theta2 (rad)

% Generate degree-5 polynomial trajectories
[theta1_traj, theta1_vel, theta1_acc] = generate_trajectory(theta1_initial, theta1_final, tf, t);
[rho_traj, rho_vel, rho_acc] = generate_trajectory(rho_initial, rho_final, tf, t);
[theta2_traj, theta2_vel, theta2_acc] = generate_trajectory(theta2_initial, theta2_final, tf, t);

% Initialize arrays for storing end-effector positions
x_points = zeros(1, length(t));
y_points = zeros(1, length(t));
z_points = zeros(1, length(t));

% DH Parameters
a = [0, 0, 30];
alpha = [pi/2, 0, pi/2];

% Loop through each time step
for i = 1:length(t)
    d = [45, rho_traj(i), 0];
    theta = [theta1_traj(i), 0, theta2_traj(i)];

    % Compute transformation matrices using the DenaHart function
    [T_total, ~] = q4_DenaHart(alpha, d, theta, a);

    % Extract end-effector position
    x_points(i) = T_total(1, 4);
    y_points(i) = T_total(2, 4);
    z_points(i) = T_total(3, 4);
end

% Plot the end-effector trajectory
figure;
plot3(x_points, y_points, z_points, 'b-', 'LineWidth', 2);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('End-Effector Trajectory');
hold on;

% Annotate the start and end points
scatter3(x_points(1), y_points(1), z_points(1), 'go', 'filled');
text(x_points(1), y_points(1), z_points(1), 'Start', 'Color', 'g');
scatter3(x_points(end), y_points(end), z_points(end), 'ro', 'filled');
text(x_points(end), y_points(end), z_points(end), 'End', 'Color', 'r');
hold off;

% =================== FUNCTION: generate_trajectory ====================
function [pos, vel, acc] = generate_trajectory(q0, qf, tf, t)
    % Generates a 5th degree polynomial trajectory
    % Inputs:
    %   q0: Initial position
    %   qf: Final position
    %   tf: Total time
    %   t: Time vector
    % Outputs:
    %   pos: Position trajectory
    %   vel: Velocity trajectory
    %   acc: Acceleration trajectory

    % Coefficients of the degree-5 polynomial
    a0 = q0;
    a1 = 0;
    a2 = 0;
    a3 = 10 * (qf - q0) / tf^3;
    a4 = -15 * (qf - q0) / tf^4;
    a5 = 6 * (qf - q0) / tf^5;

    % Compute position, velocity, and acceleration
    pos = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    vel = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    acc = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end
