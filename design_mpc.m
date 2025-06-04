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

%% Simulate with various constraints on control effort

% Define simulation parameters
Tsim = 6;                        % Total simulation time in seconds
steps = round(Tsim / Ts);        % Number of simulation steps
r = ones(steps, 1);              % Reference signal

% Define different MV constraint scenarios
mv_constraints = [
    5, -5;
    10, -10;
    20, -20
    ];

% Define colors for plotting
colors = {'m', 'c', 'g'};        % Colors for different scenarios
labels = {'±5 N', '±10 N', '±20 N'};  % Labels for legend

% Initialize cell arrays to store simulation results
y_all = cell(1, length(mv_constraints));
u_all = cell(1, length(mv_constraints));
t_all = cell(1, length(mv_constraints));

% Loop over each MV constraint scenario
for i = 1:length(mv_constraints)
    % Clone the original MPC object to avoid modifying it
    mpc_temp = mpcobj;

    % Set MV constraints
    mpc_temp.MV(1).Max = mv_constraints(i, 1);
    mpc_temp.MV(1).Min = mv_constraints(i, 2);

    % Run simulation
    [y, t, u] = sim(mpc_temp, steps, r);

    % Store results
    y_all{i} = y;
    u_all{i} = u;
    t_all{i} = t;
end

% Plotting
figure('Color', 'k');

% First subplot: Position
ax1 = subplot(2,1,1);
hold on;
for i = 1:length(mv_constraints)
    stairs(t_all{i}, y_all{i}(:,1), '-', 'Color', colors{i}, 'LineWidth', 2);
end
stairs(t_all{1}, r, 'r--', 'LineWidth', 2);  % Reference signal
hold off;
legend([labels, {'Setpoint'}], 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12);
ylabel('Position (m)', 'Color', 'w', 'FontSize', 12);
title('Position', 'Color', 'w', 'FontSize', 12);
grid on;
ax1.Color = 'k';
ax1.GridColor = 'w';
ax1.GridAlpha = 0.3;
ax1.XColor = 'w';
ax1.YColor = 'w';

% Second subplot: Control Effort
ax2 = subplot(2,1,2);
hold on;
for i = 1:length(mv_constraints)
    stairs(t_all{i}, u_all{i}, '-', 'Color', colors{i}, 'LineWidth', 2);
end
hold off;
legend(labels, 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12);
ylabel('Force (N)', 'Color', 'w', 'FontSize', 12);
xlabel('Time (s)', 'Color', 'w', 'FontSize', 12);
title('Control Effort', 'Color', 'w', 'FontSize', 12);
grid on;
ax2.Color = 'k';
ax2.GridColor = 'w';
ax2.GridAlpha = 0.3;
ax2.XColor = 'w';
ax2.YColor = 'w';

%% Simulate with various constraints on position

% Define simulation parameters
Tsim = 6;                        % Total simulation time in seconds
steps = round(Tsim / Ts);        % Number of simulation steps
r = ones(steps, 1);              % Reference signal

% Define different OV constraint scenarios
ov_constraints = [
    1.15, -1.15;
    1.05, -1.05;
    1.02, -1.02
    ];

t_constraint = [0 Tsim];

% Define colors for plotting
colors = {'m', 'c', 'g'};        % Colors for different responses
colors_constraints = {'b', 'y', 'w'};        % Colors for different constraints
labels = {'Response with 1.15 m Constraint', 'Response with 1.05 m Constraint', 'Response with 1.02 m Constraint'};  % Labels for legend (response)
labels_constraints = {'1.15 m Constraint', '1.05 m Constraint', '1.02 m Constraint'};  % Labels for legend (constraint)

% Initialize cell arrays to store simulation results
y_all = cell(1, length(ov_constraints));
u_all = cell(1, length(ov_constraints));
t_all = cell(1, length(ov_constraints));

% Loop over each OV constraint scenario
for i = 1:length(ov_constraints)
    % Clone the original MPC object to avoid modifying it
    mpc_temp = mpcobj;

    % Set OV constraints
    mpc_temp.OV(1).Max = ov_constraints(i, 1);
    mpc_temp.OV(1).Min = ov_constraints(i, 2);

    % Make MPC more aggressive to increase overshoot
    mpc_temp.Weights.ManipulatedVariablesRate = 0;

    % Run simulation
    [y, t, u] = sim(mpc_temp, steps, r);

    % Store results
    y_all{i} = y;
    u_all{i} = u;
    t_all{i} = t;
end

% Plotting
figure('Color', 'k');

% First subplot: Position
ax1 = subplot(2,1,1);
hold on;
for i = 1:length(ov_constraints)
    stairs(t_all{i}, y_all{i}(:,1), '-', 'Color', colors{i}, 'LineWidth', 2);
end
for i = 1:length(ov_constraints)
    stairs(t_constraint, [ov_constraints(i,1) ov_constraints(i,1)], '--', 'Color', colors_constraints{i}, 'LineWidth', 2);
end
stairs(t_all{1}, r, 'r--', 'LineWidth', 2);  % Reference signal
hold off;
legend([labels, labels_constraints, {'Setpoint'}], 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 10, Location='best');
ylabel('Position (m)', 'Color', 'w', 'FontSize', 10);
title('Position', 'Color', 'w', 'FontSize', 12);
grid on;
ax1.Color = 'k';
ax1.GridColor = 'w';
ax1.GridAlpha = 0.3;
ax1.XColor = 'w';
ax1.YColor = 'w';

% Second subplot: Control Effort
ax2 = subplot(2,1,2);
hold on;
for i = 1:length(ov_constraints)
    stairs(t_all{i}, u_all{i}, '-', 'Color', colors{i}, 'LineWidth', 2);
end
hold off;
legend(labels, 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 10, Location='best');
ylabel('Force (N)', 'Color', 'w', 'FontSize', 10);
xlabel('Time (s)', 'Color', 'w', 'FontSize', 10);
title('Control Effort', 'Color', 'w', 'FontSize', 12);
grid on;
ax2.Color = 'k';
ax2.GridColor = 'w';
ax2.GridAlpha = 0.3;
ax2.XColor = 'w';
ax2.YColor = 'w';

%% Run Simulink model 
% Run Simulink simulation
Fd_step = 0;
simIn = Simulink.SimulationInput('model');
simIn = simIn.setVariable('Fd_step', Fd_step);
simIn = simIn.setVariable('np', 0.0005);
simIn = simIn.setModelParameter('StopTime', '5');
out = sim(simIn);

% Get signals from simulation
t_z = out.logsout.get('z').Values.Time;
z = out.logsout.get('z').Values.Data;          % Position 
t_z_meas = out.logsout.get('z_meas').Values.Time;
z_meas = out.logsout.get('z_meas').Values.Data;  % Measured position
t_cost = out.logsout.get('cost').Values.Time;
cost = out.logsout.get('cost').Values.Data;    % Cost function value
t_ref = out.logsout.get('ref').Values.Time;
ref = out.logsout.get('ref').Values.Data;      % Reference signal
t_F = out.logsout.get('F').Values.Time;
F = out.logsout.get('F').Values.Data;          % Control force

%% Plot (black background) - from Simulink
% Create figure with black background
figure('Color', 'k');

% First subplot: Position
ax1 = subplot(2,1,1);

stairs(t_z, z, 'm-', 'LineWidth', 2); % Feedback in magenta
hold on;
stairs(t_z_meas, z_meas, 'y-', 'LineWidth', 2); % Feedback in magenta
stairs(t_ref, ref, 'c--', 'LineWidth', 2); % Setpoint in cyan dashed
hold off;

legend('Feedback', 'Measured Feedback', 'Setpoint', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
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

stairs(t_F, F, 'g-', 'LineWidth', 2); % Control effort in white

ylabel('Force (N)', 'Color', 'w', 'FontSize', 12); % Y-axis label in white
xlabel('Time (s)', 'Color', 'w', 'FontSize', 12); % X-axis label in white
title('Control Effort', 'Color', 'w', 'FontSize', 12); % Title in white
grid on;
ax2.Color = 'k';
ax2.GridColor = 'w';
ax2.GridAlpha = 0.3;
ax2.XColor = 'w';
ax2.YColor = 'w';

%% Run Simulink model 
% Run Simulink simulation with force disturbance
Fd_step = 2;
simIn = Simulink.SimulationInput('model');
simIn = simIn.setVariable('Fd_step', Fd_step);
simIn = simIn.setVariable('np', 0);
simIn = simIn.setModelParameter('StopTime', '20');
out = sim(simIn);

% Get signals from simulation
t_z = out.logsout.get('z').Values.Time;
z = out.logsout.get('z').Values.Data;          % Position 
t_z_meas = out.logsout.get('z_meas').Values.Time;
z_meas = out.logsout.get('z_meas').Values.Data;  % Measured position
t_cost = out.logsout.get('cost').Values.Time;
cost = out.logsout.get('cost').Values.Data;    % Cost function value
t_ref = out.logsout.get('ref').Values.Time;
ref = out.logsout.get('ref').Values.Data;      % Reference signal
t_F = out.logsout.get('F').Values.Time;
F = out.logsout.get('F').Values.Data;          % Control force
t_Fd = out.logsout.get('Fd').Values.Time;
Fd = out.logsout.get('Fd').Values.Data;          % Force disturbance

%% Plot (black background) - from Simulink
% Create figure with black background
figure('Color', 'k');

% First subplot: Position
ax1 = subplot(2,1,1);

stairs(t_z, z, 'm-', 'LineWidth', 2); % Feedback in magenta
hold on;
stairs(t_ref, ref, 'c--', 'LineWidth', 2); % Setpoint in cyan dashed
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

stairs(t_F, F, 'g-', 'LineWidth', 2); % Control effort in green
hold on;
stairs(t_Fd, Fd, 'r-', 'LineWidth', 2); % Force disturbance in magenta
hold off;

legend('Control effort', 'Disturbance', 'TextColor', 'w', 'Color', 'k', 'EdgeColor', ...
    [0.5 0.5 0.5], 'LineWidth', 1, 'FontSize', 12); % Legend text in white

ylabel('Force (N)', 'Color', 'w', 'FontSize', 12); % Y-axis label in white
xlabel('Time (s)', 'Color', 'w', 'FontSize', 12); % X-axis label in white
title('Control Effort', 'Color', 'w', 'FontSize', 12); % Title in white
grid on;
ax2.Color = 'k';
ax2.GridColor = 'w';
ax2.GridAlpha = 0.3;
ax2.XColor = 'w';
ax2.YColor = 'w';