% DH Parameters for the RPR Robot
alpha = [pi/2, 0, pi/2];          % Twist angles
d = [45, 25 + 50, 0];            % Link offsets (rho = 50 mm for prismatic joint)
theta = [pi/4, 0, pi/3];         % Joint angles (theta1 = pi/4, theta2 = pi/3)
a = [0, 0, 30];                  % Link lengths

% Compute Transformation Matrices
[T_total, transformations] = q4_DenaHart(alpha, d, theta, a);

% Extract Z-axes and origins for each frame
z_axes = [0; 0; 1]; % Base frame z-axis
origins = [0; 0; 0]; % Base frame origin
for i = 1:length(transformations)
    T = transformations{i};
    z_axes = [z_axes, T(1:3, 3)];
    origins = [origins, T(1:3, 4)];
end

% Initialize Jacobian Matrix
n = length(theta); % Number of joints
J_v = zeros(3, n); % Linear velocity Jacobian
J_w = zeros(3, n); % Angular velocity Jacobian

% Compute Jacobian Columns
for i = 1:n
    % Extract z-axis and origin of the current frame
    z_i = z_axes(:, i);
    o_i = origins(:, i);
    o_n = origins(:, end); % End-effector origin

    if i == 2 % Prismatic joint
        J_v(:, i) = z_i;    % Linear velocity
        J_w(:, i) = [0; 0; 0]; % No angular velocity
    else % Revolute joint
        J_v(:, i) = cross(z_i, o_n - o_i); % Linear velocity
        J_w(:, i) = z_i;                  % Angular velocity
    end
end

% Combine Linear and Angular Jacobian
Jacobian = [J_v; J_w];

% Display Results
fprintf('Jacobian Matrix:\n');
disp(Jacobian);
fprintf('Jacobian Matrix Size: %dx%d\n', size(Jacobian));
