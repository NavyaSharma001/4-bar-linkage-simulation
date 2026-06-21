%% 4 bar likn
% Resolves closed-loop positions via Freudenstein's analytical vector loop equations
% Implements a numerical derivative pass to bypass branch-crossover singularities

clc; clear; close all;

%% 1. Define Mechanism Link Dimensions(mm) 
l1 = 160; % Ground link (fixed base pin distance)
l2 = 40;  % Input Crank link (fully rotating link attached to motor)
l3 = 120; % Coupler link (floating connector link)
l4 = 100; % Output Rocker link (oscillating arm)
% Grashof condition check: Shortest link + Longest link <= sum of remaining links
if (l2 + l1 <= l3 + l4) 
    fprintf('>> Status: Grashof condition satisfied. Linkage will achieve continuous rotation.\n\n');
else
    fprintf('>> Warning: Non-Grashof configuration. System will lock or bind at specific limits.\n\n');
end

%% 2. Simulation Step and Domain Setup
theta2_vec = linspace(0, 2*pi, 180); % Crank rotation path from 0 to 360 degrees
omega2 = (100*2*pi)/60; % Constant input velocity of driving crank motor (rad/s)

% Pre-allocate memory arrays for fast processing
T3 = zeros(size(theta2_vec));
T4 = zeros(size(theta2_vec));

%% 3. Closed-Loop Position Tracking via Freudenstein Constants
K1 = l1/l2;
K2 = l1/l4;
K3 = (l1^2 + l2^2 - l3^2 + l4^2)/(2*l2*l4);

for i = 1:length(theta2_vec)
    t2 = theta2_vec(i);
    
    A = cos(t2) - K1 - K2*cos(t2) + K3;
    B = -2*sin(t2);
    C = K1 - (K2+1)*cos(t2) + K3;
    
    % Solve for Output Angle (theta4) using trigonometric quadratic roots
    % Utilizing open-branch configuration geometry
    T4(i) = 2 * atan2(-B - sqrt(B^2 - 4*A*C), 2*A);
    
    % Solve for Coupler Angle (theta3) using loop closure vectors
    T3(i) = atan2((l1 + l4*sin(T4(i)) - l2*sin(t2)), (l4*cos(T4(i)) - l1 - l2*cos(t2)));
end

%% 4. Graphical Real-Time Animation Window
figure('Name', '4-Bar Linkage Kinematic Simulation', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]);

for i = 1:length(theta2_vec)
    t2 = theta2_vec(i); t3 = T3(i); t4 = T4(i);
    
    % Joint pin coordinate mapping
    xA = l2*cos(t2);          yA = l2*sin(t2);          % Crank pin (A)
    xB = l1 + l4*cos(t4);     yB = l4*sin(t4);          % Rocker pin (B)
    
    % Clear and plot current physical links configuration layout
    plot([0, xA], [0, yA], 'r-o', 'LineWidth', 3, 'MarkerSize', 8); hold on; % Crank (Red)
    plot([xA, xB], [yA, yB], 'b-o', 'LineWidth', 3, 'MarkerSize', 8);        % Coupler (Blue)
    plot([l1, xB], [0, yB], 'g-o', 'LineWidth', 3, 'MarkerSize', 8);         % Rocker (Green)
    plot([0, l1], [0, 0], 'k-', 'LineWidth', 2);                             % Ground Frame (Black)
    
    % Format viewing window parameters dynamically based on sizing
    axis equal;
    grid on;
    xlim([-l2-20, l1+l4+20]); ylim([-l4-20, l4+20]);
    title('Planar Linkage Configuration Profile Analysis', 'FontSize', 11);
    xlabel('X Coordinate (mm)'); ylabel('Y Coordinate (mm)');
    
    drawnow; 
    if i < length(theta2_vec), clf; end 
end

%% 5. Kinematic Performance Plotting (Singularity-Free Update)
figure('Name', 'Output Kinematics Workspace Profile', 'NumberTitle', 'off');
set(gcf, 'Color', [1 1 1]);
subplot(2,1,1);
plot(rad2deg(theta2_vec), rad2deg(T4), 'g-', 'LineWidth', 2);
title('Output Rocker Position Profile (\theta_4 vs \theta_2)', 'FontSize', 10);
xlabel('Crank Position (deg)'); ylabel('Rocker Position (deg)'); grid on;
subplot(2,1,2);

% Calculate instantaneous change in rocker position over crank displacement interval
dTheta4 = diff(T4);
dTheta2 = diff(theta2_vec);
omega4_numerical = (dTheta4 ./ dTheta2) * omega2;

% Pad the final array vector element to match the baseline x-axis dimensions
omega4_numerical = [omega4_numerical, omega4_numerical(end)];

plot(rad2deg(theta2_vec), omega4_numerical, 'r-', 'LineWidth', 2);
title('Output Rocker Angular Velocity (\omega_4) - Singularity Free', 'FontSize', 10);
xlabel('Crank Position (deg)'); ylabel('Angular Velocity (rad/s)'); grid on;
