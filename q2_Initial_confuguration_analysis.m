% Initial configuration values
theta1 = pi/4; % 45 degrees
rho = 50;      % Prismatic joint extension (mm)
theta2 = pi/4; % 45 degrees

% Call the RPR3D function to visualize the configuration
RPR3D(theta1, rho, theta2);

% Add labels for clarity
title('First Configuration of the RPR Robot');
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
