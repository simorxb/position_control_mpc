%% Parameters
m = 10;
k = 0.5;

%% State space model
A = [-k/m 0; 1 0];
B = [1/m; 0];
C = [0 1];
D = 0;

% Create state-space model
object_sys = ss(A, B, C, D);

%% Create MPC object
MV = struct('Min', -10, 'Max', 10, 'ScaleFactor', 10);
OV = struct('Min', -2, 'Max', 2, 'ScaleFactor', 2);
Weights = struct('MV', 0, 'MVRate', 0.1, 'OV', 1);
Ts = 0.1;
mpcobj = mpc(object_sys, Ts, 10, 2, Weights, MV, OV);

%% Simulate controller
Tsim = 5;                       % seconds
steps = round(Tsim/Ts);         % simulation iterations
r = 1*ones(steps, 1);           % reference signal
[y,t,u] = sim(mpcobj,steps,r);

%% Plot (black background)
% Create figure with black background
figure('Color', 'k');

% First subplot: Position
ax1 = subplot(2,1,1);

stairs(t, y(:,1), 'm-', 'LineWidth', 2); % Feedback in white
hold on;
stairs(t, r(:,1), 'c--', 'LineWidth', 2); % Setpoint in red dashed
hold off;

legend('Feedback', 'Setpoint', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12); % Legend text in white
ylabel('Position (m)', 'Color', 'w', 'FontSize', 12); % Y-axis label in white
title('Position', 'Color', 'w', 'FontSize', 12); % Title in white
grid on;
ax1.Color = 'k';
ax1.GridColor = 'w';
ax1.GridAlpha = 0.3;
ax1.XColor = 'w';
ax1.YColor = 'w';

% Second subplot: Control Effort
ax2 = subplot(2,1,2);

stairs(t, u, 'g-', 'LineWidth', 2); % Control effort in white

ylabel('Force (N)', 'Color', 'w', 'FontSize', 12); % Y-axis label in white
xlabel('Time (s)', 'Color', 'w', 'FontSize', 12); % X-axis label in white
title('Control Effort', 'Color', 'w', 'FontSize', 12); % Title in white
grid on;
ax2.Color = 'k';
ax2.GridColor = 'w';
ax2.GridAlpha = 0.3;
ax2.XColor = 'w';
ax2.YColor = 'w';
