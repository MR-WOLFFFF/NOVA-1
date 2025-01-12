function [theta1, rho, theta2] = inverse_kinematics(x_e, y_e, z_e, a3, d1)
    % Inputs:
    %   x_e, y_e, z_e: Cartesian coordinates of the end-effector
    %   a3: Link length of the third link
    %   d1: Fixed offset of the first joint
    % Outputs:
    %   theta1, rho, theta2: Joint variables (angles in radians, rho in mm)

    % Check if the z-coordinate is consistent with d1
    if abs(z_e - d1) > 1e-6
        error('Invalid z-coordinate: does not match the fixed offset d1');
    end

    % Compute theta1 using atan2
    theta1 = atan2(y_e, x_e);

    % Compute rho (prismatic joint extension)
    % First, calculate the projection of the end-effector position onto the XY plane
    projection_xy = sqrt(x_e^2 + y_e^2);

    % Solve for theta2 using the law of cosines
    c2 = (projection_xy^2 - a3^2) / (2 * projection_xy * a3);
    theta2 = acos(c2);

    % Compute rho considering the projection_xy
    rho = projection_xy - a3 * cos(theta2);

    % Return the joint variables
    theta1 = mod(theta1, 2*pi); % Ensure theta1 is within [0, 2*pi]
    theta2 = mod(theta2, 2*pi); % Ensure theta2 is within [0, 2*pi]
end

% Example Usage
x_e = 50; % Desired end-effector x-coordinate (mm)
y_e = 50; % Desired end-effector y-coordinate (mm)
z_e = 45; % Desired end-effector z-coordinate (mm)
a3 = 30;  % Length of link 3 (mm)
d1 = 45;  % Offset of link 1 (mm)

[theta1, rho, theta2] = inverse_kinematics(x_e, y_e, z_e, a3, d1);

% Display the results
fprintf('Joint Variables:\n');
fprintf('Theta1: %.2f rad (%.2f degrees)\n', theta1, rad2deg(theta1));
fprintf('Rho: %.2f mm\n', rho);
fprintf('Theta2: %.2f rad (%.2f degrees)\n', theta2, rad2deg(theta2));
