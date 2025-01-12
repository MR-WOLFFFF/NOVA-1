function [T_total, transformations] = q4_DenaHart(alpha, d, theta, a)
    % DenaHart: Computes the DH transformation matrices for a robotic arm.
    % Inputs:
    %   alpha: Array of twist angles (in radians)
    %   d: Array of link offsets (in mm)
    %   theta: Array of joint angles (in radians)
    %   a: Array of link lengths (in mm)
    % Outputs:
    %   T_total: Final transformation matrix (T_0T_n)
    %   transformations: Array of individual transformation matrices (T_i)

    % Number of joints/links
    n = length(alpha);

    % Initialize the overall transformation matrix
    T_total = eye(4);

    % Initialize cell array to store intermediate transformations
    transformations = cell(1, n);

    % Loop through each joint/link to compute the transformation matrix
    for i = 1:n
        % Compute the transformation matrix for the current link
        T = [
            cos(theta(i)), -sin(theta(i)) * cos(alpha(i)),  sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
            sin(theta(i)),  cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
            0,              sin(alpha(i)),                cos(alpha(i)),                d(i);
            0,              0,                            0,                            1
        ];

        % Store the transformation matrix
        transformations{i} = T;

        % Update the total transformation matrix
        T_total = T_total * T;
    end
end
