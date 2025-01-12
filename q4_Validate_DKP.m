% DH Parameters for the RPR Robot
alpha = [pi/2, 0, pi/2];          % Twist angles
d = [45, 25 + 50, 0];            % Link offsets (rho = 50 mm for prismatic joint)
theta = [pi/4, 0, pi/3];         % Joint angles (theta1 = pi/4, theta2 = pi/3)
a = [0, 0, 30];                  % Link lengths

% Call the q4_DenaHart function
[T_total, transformations] = q4_DenaHart(alpha, d, theta, a);

% Display the individual transformation matrices
fprintf('Transformation Matrices for Each Link:\n');
for i = 1:length(transformations)
    fprintf('Link %d Transformation Matrix:\n', i);
    disp(transformations{i});
end

% Display the final transformation matrix
fprintf('Final Transformation Matrix (T_0T_n):\n');
disp(T_total);

% Visualization
x_points = [0]; % X-coordinates of frames
y_points = [0]; % Y-coordinates of frames
z_points = [0]; % Z-coordinates of frames
current_T = eye(4);

for i = 1:length(transformations)
    current_T = current_T * transformations{i};
    x_points = [x_points, current_T(1, 4)];
    y_points = [y_points, current_T(2, 4)];
    z_points = [z_points, current_T(3, 4)];
end

figure;
plot3(x_points, y_points, z_points, '-o', 'LineWidth', 2, 'MarkerSize', 8);
grid on;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Robot Configuration Validation');
