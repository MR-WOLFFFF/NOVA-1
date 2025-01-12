% Denavit-Hartenberg Parameters for the RPR Robot
DH_table = [
    0, pi/2, 45, pi/4;    % Link 1 (Revolute joint, theta1 = pi/4)
    0, 0, 25 + 50, 0;     % Link 2 (Prismatic joint, rho = 50 mm)
    30, pi/2, 0, pi/3     % Link 3 (Revolute joint, theta2 = pi/3)
];

% Extract DH parameters into separate arrays
a = DH_table(:, 1);
alpha = DH_table(:, 2);
d = DH_table(:, 3);
theta = DH_table(:, 4);

% Call the DenaHart function to compute the transformation matrices
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

% Visualization of the robot's configuration
x_points = [0];
y_points = [0];
z_points = [0];
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

hold on;
for i = 1:length(x_points)
    text(x_points(i), y_points(i), z_points(i), sprintf('Frame %d', i-1), 'FontSize', 10, 'Color', 'b');
end
hold off;
