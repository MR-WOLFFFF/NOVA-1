% Denavit-Hartenberg Parameters for the RPR Robot
% Each row corresponds to a link: [a_i, alpha_i, d_i, theta_i]
% a_i = link length, alpha_i = twist angle, d_i = link offset, theta_i = joint angle
DH_table = [
    0, pi/2, 45, pi/4;    % Link 1 (Revolute joint, theta1 = pi/4)
    0, 0, 25 + 50, 0;     % Link 2 (Prismatic joint, rho = 50 mm)
    30, pi/2, 0, pi/3     % Link 3 (Revolute joint, theta2 = pi/3)
];

% Number of links
num_links = size(DH_table, 1);

% Initialize overall transformation matrix as identity matrix
T_total = eye(4);

% Initialize arrays for visualization
x_points = [0]; % X-coordinates of frames
y_points = [0]; % Y-coordinates of frames
z_points = [0]; % Z-coordinates of frames

% Loop through each link to compute transformation matrices
fprintf('Transformation Matrices for Each Link:\n');
for i = 1:num_links
    % Extract DH parameters for the current link
    a = DH_table(i, 1);        % Link length
    alpha = DH_table(i, 2);    % Twist angle
    d = DH_table(i, 3);        % Link offset
    theta = DH_table(i, 4);    % Joint angle
    
    % Compute the transformation matrix for the current link
    T = [
        cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
        0,           sin(alpha),            cos(alpha),            d;
        0,           0,                     0,                     1
    ];
    
    % Display the transformation matrix for the current link
    fprintf('Link %d Transformation Matrix:\n', i);
    disp(T);
    
    % Update the overall transformation matrix
    T_total = T_total * T;
    
    % Store the position of the frame for visualization
    x_points = [x_points, T_total(1, 4)];
    y_points = [y_points, T_total(2, 4)];
    z_points = [z_points, T_total(3, 4)];
end

% Display the final transformation matrix from base to end-effector
fprintf('Final Transformation Matrix (T_0T_n):\n');
disp(T_total);

% Visualization of the robot's configuration
figure;
plot3(x_points, y_points, z_points, '-o', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Robot Configuration');
hold on;

% Annotate frames
for i = 1:length(x_points)
    text(x_points(i), y_points(i), z_points(i), sprintf('Frame %d', i-1), 'FontSize', 10, 'Color', 'b');
end
hold off;
