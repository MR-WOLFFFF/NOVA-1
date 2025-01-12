function J = Jacobian_RPR(theta1, rho, theta2)
    % Input parameters:
    % theta1: Angle of the first revolute joint (in radians)
    % rho: Extension of the prismatic joint (in mm)
    % theta2: Angle of the second revolute joint (in radians)

    % Link lengths (as per the RPR robot configuration)
    L1 = 45; % Fixed base length along z-axis
    L2 = rho + 25; % Prismatic joint with additional offset
    L3 = 30; % Fixed link length for revolute joint

    % Position of the end-effector (derived from direct kinematics)
    x = cos(theta1) * (L3 * cos(theta2));
    y = sin(theta1) * (L3 * cos(theta2));
    z = L1 + rho - (L3 * sin(theta2));
    
    % Partial derivatives for Jacobian matrix (Linear Velocity Part Jv)
    Jv = [
        -L3 * sin(theta1) * cos(theta2), 0, -L3 * cos(theta1) * sin(theta2);
         L3 * cos(theta1) * cos(theta2), 0, -L3 * sin(theta1) * sin(theta2);
          0,                          1, -L3 * cos(theta2)
    ];
    
    % Angular velocity part (Jo) for the revolute joints
    % First revolute joint rotates around z-axis
    % Second revolute joint rotates around y-axis
    Jo = [
        0, 0, cos(theta1);
        0, 0, sin(theta1);
        1, 0, 0;
    ];

    % Full Jacobian matrix combining linear and angular velocity parts
    J = [
        Jv;
        Jo
    ];
end

% Example Usage
theta1 = pi/4; % Example: 45 degrees in radians
rho = 50;      % Example: 50 mm prismatic extension
theta2 = pi/6; % Example: 30 degrees in radians

% Compute Jacobian
J = Jacobian_RPR(theta1, rho, theta2);
disp('Jacobian Matrix:');
disp(J);
