%%% ECE 443 Final Project %%%
clc; clear all; close all;

% Simulation parameters
N = 25; % Number of agents
L = 5; % Size of environment
T = 500; % Number of time steps
timeStep = 0.01; % Time step

%% 2D Flocking in Free Space %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader

p_xg = L*(rand - 0.5); % Random initial velocity x component
p_yg = L*(rand - 0.5); % Random initial velocity y component
x_g(1) = L*(rand - 0.5); % Random initial x position for gamma leader
y_g(1) = L*(rand - 0.5); % Random initial y position for gamma leader

% p_xg = 2;
% p_yg = 0;
% x_g(1) = -3;
% y_g(1) = 0;

% Beta agent parameters
% c1_b = 3;
% c2_b = 3;

% Ms = [0, 1, -1; 0, 0, 0; 0.3, 0.0, 0.3]; % split and rejoin obstacle matrix
% Ms = [0, 0, 0; 1.25, 0, -1.25; 0.5, 0.0, 0.5];

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
for i = 1:N 
    x(1,i) = L*(rand - 0.5);
    y(1,i) = L*(rand - 0.5);

    % x(1,i) = -4*(rand + 0.4);
    % y(1,i) = 4*(rand)-2.5;
end

% Prepare the figure for real-time drawing
figure;
axis([-L L -L L]);
axis square;
hold on;

% Simulation Loop
ui_alpha = zeros(2, N);
ui_gamma = zeros(2, N);
ui_beta = zeros(2, N);

for k = 1:T-1
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;

    for i = 1:N
        q_i = [x(k,i); y(k,i)];
        p_i = [p_x(k,i); p_y(k,i)];
        ui_alpha(:, i) = zeros(2, 1); % Reset alpha forces for this agent
        ui_gamma(:, i) = zeros(2, 1); % Reset gamma forces for this agent
        ui_beta(:, i) = zeros(2, 1); % Reset beta forces for this agent

        for j = 1:N
            q_j = [x(k,j); y(k,j)];
            p_j = [p_x(k,j); p_y(k,j)];
            distance = norm(q_j - q_i);
            if distance < r
                n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                a_ij = rho_h(h, distance / r);
                ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
            end
        end

        % % Beta agent obstacle avoidance
        % for n = 1:length(Ms)
        %     midpoint = [Ms(1,n); Ms(2,n)];
        %     radius = Ms(3,n);
        % 
        %     mu_b = mu(midpoint, q_i, radius);
        %     a_kb = a_k(q_i, midpoint);
        % 
        %     q_ik = mu_b * q_i + (1 - mu_b) * midpoint; % Interpolated position relative to obstacle
        %     p_ik = mu_b * (eye(2) - a_kb * a_kb.') * p_i; % Adjusted velocity
        % 
        %     b_ik = rho_h(h, norm(q_ik - q_i) / d); % Bumping function applied to the normalized distance
        %     n_ik = sigma_epsilon(epsilon, (q_ik - q_i)); % Normalized interaction vector
        % 
        %     ui_beta(:,i) = ui_beta(:,i) + c1_b * phi_beta(norm(q_ik - q_i), d, h) * n_ik + c2_b * b_ik * (p_ik - p_i);
        % end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k)];
        p_g = [p_xg; p_yg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i); %+ ui_beta(:, i);

        % Update positions and velocities
        x(k+1, i) = x(k, i) + timeStep * p_x(k, i);
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
    end
end

% Animation
vidObj = VideoWriter('2d_flocking_free_space.avi');
open(vidObj);
for k = 1:T
    clf;
    hold on;
    % Draw each agent with smaller dots
    for i = 1:N
        rectangle('Position', [x(k,i)-0.075, y(k,i)-0.075, 0.15, 0.15], 'FaceColor', 'b', 'Curvature', [1, 1]);
    end
    % Draw the gamma leader in red with a smaller dot
    rectangle('Position', [x_g(k)-0.075, y_g(k)-0.075, 0.15, 0.15], 'FaceColor', 'r', 'Curvature', [1, 1]);
    
    % % Draw obstacles as black circles
    % for n = 1:size(Ms, 2)
    %     obstacle_x = Ms(1, n);
    %     obstacle_y = Ms(2, n);
    %     obstacle_radius = Ms(3, n);
    %     viscircles([obstacle_x, obstacle_y], obstacle_radius, 'EdgeColor', 'k');
    % end

    axis([-5 5 -5 5]);
    axis square;
    title('2D Flocking in Free Space');
    hold off;

    currFrame = getframe(gcf);
    writeVideo(vidObj, currFrame);
end

close(vidObj);

%% 2D Flocking Split/Rejoin Maneuver %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader

% p_xg = L*(rand - 0.5); % Random initial velocity x component
% p_yg = L*(rand - 0.5); % Random initial velocity y component
% x_g(1) = L*(rand - 0.5); % Random initial x position for gamma leader
% y_g(1) = L*(rand - 0.5); % Random initial y position for gamma leader

p_xg = 2;
p_yg = 0;
x_g(1) = -3;
y_g(1) = 0;

% Beta agent parameters
c1_b = 3;
c2_b = 3;

Ms = [0, 1, -1; 0, 0, 0; 0.3, 0.0, 0.3]; % split and rejoin obstacle matrix
% Ms = [0, 0, 0; 1.25, 0, -1.25; 0.5, 0.0, 0.5]; % squeeze obstacle matrix

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
for i = 1:N 
    % x(1,i) = L*(rand - 0.5);
    % y(1,i) = L*(rand - 0.5);

    x(1,i) = -4*(rand + 0.4);
    y(1,i) = 4*(rand)-2.5;
end

% Prepare the figure for real-time drawing
figure;
axis([-L L -L L]);
axis square;
hold on;

% Simulation Loop
ui_alpha = zeros(2, N);
ui_gamma = zeros(2, N);
ui_beta = zeros(2, N);

for k = 1:T
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;

    for i = 1:N
        q_i = [x(k,i); y(k,i)];
        p_i = [p_x(k,i); p_y(k,i)];
        ui_alpha(:, i) = zeros(2, 1); % Reset alpha forces for this agent
        ui_gamma(:, i) = zeros(2, 1); % Reset gamma forces for this agent
        ui_beta(:, i) = zeros(2, 1); % Reset beta forces for this agent

        for j = 1:N
            q_j = [x(k,j); y(k,j)];
            p_j = [p_x(k,j); p_y(k,j)];
            distance = norm(q_j - q_i);
            if distance < r
                n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                a_ij = rho_h(h, distance / r);
                ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
            end
        end

        % Beta agent obstacle avoidance
        for n = 1:length(Ms)
            midpoint = [Ms(1,n); Ms(2,n)];
            radius = Ms(3,n);

            mu_b = mu(midpoint, q_i, radius);
            a_kb = a_k(q_i, midpoint);

            q_ik = mu_b * q_i + (1 - mu_b) * midpoint; % Interpolated position relative to obstacle
            p_ik = mu_b * (eye(2) - a_kb * a_kb.') * p_i; % Adjusted velocity

            b_ik = rho_h(h, norm(q_ik - q_i) / d); % Bumping function applied to the normalized distance
            n_ik = sigma_epsilon(epsilon, (q_ik - q_i)); % Normalized interaction vector

            ui_beta(:,i) = ui_beta(:,i) + c1_b * phi_beta(norm(q_ik - q_i), d, h) * n_ik + c2_b * b_ik * (p_ik - p_i);
        end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k)];
        p_g = [p_xg; p_yg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i) + ui_beta(:, i);

        % Update positions and velocities
        x(k+1, i) = x(k, i) + timeStep * p_x(k, i);
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
    end
end

% Animation
vidObj2 = VideoWriter('2d_flocking_split_join.avi');
open(vidObj2);
for k = 1:T
    clf;
    hold on;
    % Draw each agent with smaller dots
    for i = 1:N
        rectangle('Position', [x(k,i)-0.075, y(k,i)-0.075, 0.15, 0.15], 'FaceColor', 'b', 'Curvature', [1, 1]);
    end
    % Draw the gamma leader in red with a smaller dot
    rectangle('Position', [x_g(k)-0.075, y_g(k)-0.075, 0.15, 0.15], 'FaceColor', 'r', 'Curvature', [1, 1]);
    
    % Draw obstacles as black circles
    for n = 1:size(Ms, 2)
        obstacle_x = Ms(1, n);
        obstacle_y = Ms(2, n);
        obstacle_radius = Ms(3, n);
        viscircles([obstacle_x, obstacle_y], obstacle_radius, 'EdgeColor', 'k');
    end

    axis([-5 5 -5 5]);
    axis square;
    title('2D Flocking Split/Rejoin Maneuver');
    hold off;

    currFrame = getframe(gcf);
    writeVideo(vidObj2, currFrame);
end

close(vidObj2);

%% 2D Flocking Squeeze Maneuver %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader

% p_xg = L*(rand - 0.5); % Random initial velocity x component
% p_yg = L*(rand - 0.5); % Random initial velocity y component
% x_g(1) = L*(rand - 0.5); % Random initial x position for gamma leader
% y_g(1) = L*(rand - 0.5); % Random initial y position for gamma leader

p_xg = 2;
p_yg = 0;
x_g(1) = -3;
y_g(1) = 0;

% Beta agent parameters
c1_b = 3;
c2_b = 3;

%Ms = [0, 1, -1; 0, 0, 0; 0.3, 0.0, 0.3]; % split and rejoin obstacle matrix
Ms = [0, 0, 0; 1.25, 0, -1.25; 0.5, 0.0, 0.5]; % squeeze obstacle matrix

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
for i = 1:N 
    % x(1,i) = L*(rand - 0.5);
    % y(1,i) = L*(rand - 0.5);

    x(1,i) = -4*(rand + 0.4);
    y(1,i) = 4*(rand)-2.5;
end

% Prepare the figure for real-time drawing
figure;
axis([-L L -L L]);
axis square;
hold on;

% Simulation Loop
ui_alpha = zeros(2, N);
ui_gamma = zeros(2, N);
ui_beta = zeros(2, N);

for k = 1:T
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;

    for i = 1:N
        q_i = [x(k,i); y(k,i)];
        p_i = [p_x(k,i); p_y(k,i)];
        ui_alpha(:, i) = zeros(2, 1); % Reset alpha forces for this agent
        ui_gamma(:, i) = zeros(2, 1); % Reset gamma forces for this agent
        ui_beta(:, i) = zeros(2, 1); % Reset beta forces for this agent

        for j = 1:N
            q_j = [x(k,j); y(k,j)];
            p_j = [p_x(k,j); p_y(k,j)];
            distance = norm(q_j - q_i);
            if distance < r
                n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                a_ij = rho_h(h, distance / r);
                ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
            end
        end

        % Beta agent obstacle avoidance
        for n = 1:length(Ms)
            midpoint = [Ms(1,n); Ms(2,n)];
            radius = Ms(3,n);

            mu_b = mu(midpoint, q_i, radius);
            a_kb = a_k(q_i, midpoint);

            q_ik = mu_b * q_i + (1 - mu_b) * midpoint; % Interpolated position relative to obstacle
            p_ik = mu_b * (eye(2) - a_kb * a_kb.') * p_i; % Adjusted velocity

            b_ik = rho_h(h, norm(q_ik - q_i) / d); % Bumping function applied to the normalized distance
            n_ik = sigma_epsilon(epsilon, (q_ik - q_i)); % Normalized interaction vector

            ui_beta(:,i) = ui_beta(:,i) + c1_b * phi_beta(norm(q_ik - q_i), d, h) * n_ik + c2_b * b_ik * (p_ik - p_i);
        end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k)];
        p_g = [p_xg; p_yg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i) + ui_beta(:, i);

        % Update positions and velocities
        x(k+1, i) = x(k, i) + timeStep * p_x(k, i);
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
    end
end

% Animation
vidObj3 = VideoWriter('2d_flocking_split_join.avi');
open(vidObj3);
for k = 1:T
    clf;
    hold on;
    % Draw each agent with smaller dots
    for i = 1:N
        rectangle('Position', [x(k,i)-0.075, y(k,i)-0.075, 0.15, 0.15], 'FaceColor', 'b', 'Curvature', [1, 1]);
    end
    % Draw the gamma leader in red with a smaller dot
    rectangle('Position', [x_g(k)-0.075, y_g(k)-0.075, 0.15, 0.15], 'FaceColor', 'r', 'Curvature', [1, 1]);
    
    % Draw obstacles as black circles
    for n = 1:size(Ms, 2)
        obstacle_x = Ms(1, n);
        obstacle_y = Ms(2, n);
        obstacle_radius = Ms(3, n);
        viscircles([obstacle_x, obstacle_y], obstacle_radius, 'EdgeColor', 'k');
    end

    axis([-5 5 -5 5]);
    axis square;
    title('2D Flocking Split/Rejoin Maneuver');
    hold off;

    currFrame = getframe(gcf);
    writeVideo(vidObj3, currFrame);
end

close(vidObj3);

%% 3D Flocking in Free Space %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader
z_g = zeros(T, 1); % Initial z positions for gamma leader
p_xg = 2; % Constant x velocity
p_yg = 0; % No y velocity
p_zg = 0; % No z velocity
x_g(1) = -1;
y_g(1) = 0;
z_g(1) = 0;

% Beta agent parameters
c1_b = 3;
c2_b = 3;

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
z = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
p_z = zeros(T, N);
for i = 1:N 
    x(1,i) = -4 * (rand + 0.4);
    y(1,i) = 4 * (rand) - 2.5;
    z(1,i) = 2 * (rand) - 1;
end

% Prepare the figure for real-time drawing
fig = figure;
set(gca, 'xlim', [-L L], 'ylim', [-L L], 'zlim', [-L L]);
axis square;
grid on;
hold on;
view(3); % Set the view to 3D
title('3D Flocking Simulation', 'FontSize', 14, 'FontWeight', 'bold');
% Initialize the VideoWriter object
vidObj4 = VideoWriter('3d_flocking_free_space.avi');
open(vidObj4);

% Simulation Loop
ui_alpha = zeros(3, N);
ui_gamma = zeros(3, N);
ui_beta = zeros(3, N);
ui = zeros(3, N); % Initialize total force matrix

for k = 1:T-1
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;
    z_g(k+1) = z_g(k) + timeStep * p_zg;

    % Update agents' positions
    scatter3_hdl = scatter3(x(k,:), y(k,:), z(k,:), 36, 'b', 'filled');
    scatter3_hdl_g = scatter3(x_g(k), y_g(k), z_g(k), 36, 'r', 'filled');

    % Logic for movement, forces, interactions
    for i = 1:N
        q_i = [x(k,i); y(k,i); z(k,i)];
        p_i = [p_x(k,i); p_y(k,i); p_z(k,i)];
        ui_alpha(:, i) = zeros(3, 1);
        ui_gamma(:, i) = zeros(3, 1);
        ui_beta(:, i) = zeros(3, 1);

        for j = 1:N
            if i ~= j
                q_j = [x(k,j); y(k,j); z(k,j)];
                p_j = [p_x(k,j); p_y(k,j); p_z(k,j)];
                distance = norm(q_j - q_i);
                if distance < r
                    n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                    a_ij = rho_h(h, distance / r);
                    ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
                end
            end
        end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k); z_g(k)];
        p_g = [p_xg; p_yg; p_zg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i); % + ui_beta(:, i);

        % Update positions and velocities
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        x(k+1, i) = x(k, i) + timeStep * p_x(k+1, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k+1, i);
        p_z(k+1, i) = p_z(k, i) + timeStep * ui(3, i);
        z(k+1, i) = z(k, i) + timeStep * p_z(k+1, i);
    end

    drawnow; % Ensure plot updates before capturing the frame
    frame = getframe(fig); % Capture the current figure
    writeVideo(vidObj4, frame); % Write frame to video

    delete(scatter3_hdl); % Remove previous frame's agent markers
    delete(scatter3_hdl_g); % Remove previous frame's gamma leader marker
end

close(vidObj4); % Close the VideoWriter object

%% 3D Flocking Split/Rejoin %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader
z_g = zeros(T, 1); % Initial z positions for gamma leader
p_xg = 2; % Constant x velocity
p_yg = 0; % No y velocity
p_zg = 0; % No z velocity
x_g(1) = -3;
y_g(1) = 0;
z_g(1) = 0;

% Beta agent parameters
c1_b = 3;
c2_b = 3;
Ms = [0, 1, -1; 0, 0, 0; 1.25, 0.0, 1.25]; % Obstacle matrix

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
z = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
p_z = zeros(T, N);
for i = 1:N 
    x(1,i) = -4 * (rand + 0.4);
    y(1,i) = 4 * (rand) - 2.5;
    z(1,i) = 2 * (rand) - 1;
end

% Prepare the figure for real-time drawing
fig = figure;
set(gca, 'xlim', [-L L], 'ylim', [-L L], 'zlim', [-L L]);
axis square;
grid on;
hold on;
view(3); % Set the view to 3D
title('3D Flocking Split/Rejoin', 'FontSize', 14, 'FontWeight', 'bold');

% Initialize the VideoWriter object
vidObj5 = VideoWriter('3d_flocking_split_rejoin.avi');
open(vidObj5);

% Simulation Loop
ui_alpha = zeros(3, N);
ui_gamma = zeros(3, N);
ui_beta = zeros(3, N);

% Pre-create obstacle spheres for efficiency
obstacle_spheres = gobjects(size(Ms, 2), 1);
for n = 1:size(Ms, 2)
    [X, Y, Z] = sphere;
    obstacle_spheres(n) = surf(X * Ms(3,n) + Ms(1,n), Y * Ms(3,n) + Ms(2,n), Z * Ms(3,n) + 0, 'EdgeColor', 'none', 'FaceColor', 'k');
end

for k = 1:T-1
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;
    z_g(k+1) = z_g(k) + timeStep * p_zg;

    % Update agents' positions
    scatter3_hdl = scatter3(x(k,:), y(k,:), z(k,:), 36, 'b', 'filled');
    scatter3_hdl_g = scatter3(x_g(k), y_g(k), z_g(k), 36, 'r', 'filled');

    % Logic for movement, forces, interactions
    for i = 1:N
        q_i = [x(k,i); y(k,i); z(k,i)];
        p_i = [p_x(k,i); p_y(k,i); p_z(k,i)];
        ui_alpha(:, i) = zeros(3, 1);
        ui_gamma(:, i) = zeros(3, 1);
        ui_beta(:, i) = zeros(3, 1);

        for j = 1:N
            if i ~= j
                q_j = [x(k,j); y(k,j); z(k,j)];
                p_j = [p_x(k,j); p_y(k,j); p_z(k,j)];
                distance = norm(q_j - q_i);
                if distance < r
                    n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                    a_ij = rho_h(h, distance / r);
                    ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
                end
            end
        end

        % Beta agent obstacle avoidance
        for n = 1:length(Ms)
            midpoint = [Ms(1,n); Ms(2,n); 0]; % Adjust for 3D by specifying z position
            radius = Ms(3,n);

            mu_b = mu(midpoint, q_i, radius);
            a_kb = a_k(q_i, midpoint);

            q_ik = mu_b * q_i + (1 - mu_b) * midpoint; % Interpolated position relative to obstacle
            p_ik = mu_b * (eye(3) - a_kb * a_kb.') * p_i; % Adjusted velocity

            b_ik = rho_h(h, norm(q_ik - q_i) / d);
            n_ik = sigma_epsilon(epsilon, (q_ik - q_i));

            ui_beta(:, i) = ui_beta(:, i) + c1_b * phi_beta(norm(q_ik - q_i), d, h) * n_ik + c2_b * b_ik * (p_ik - p_i);
        end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k); z_g(k)];
        p_g = [p_xg; p_yg; p_zg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i) + ui_beta(:, i);

        % Update positions and velocities
        x(k+1, i) = x(k, i) + timeStep * p_x(k, i);
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
        z(k+1, i) = z(k, i) + timeStep * p_z(k, i);
    end

    drawnow; % Ensure plot updates before capturing the frame
    frame = getframe(fig); % Capture the current figure
    writeVideo(vidObj5, frame); % Write frame to video

    delete(scatter3_hdl); % Remove previous frame's agent markers
    delete(scatter3_hdl_g); % Remove previous frame's gamma leader marker
end

close(vidObj5); % Close the VideoWriter object

%% 3D Flocking Squeeze %%
% Alpha agent parameters
h = 0.5;
r = 1.2; % Interaction radius of agents
a = 3;
b = 5;
c = (abs(a-b))/(sqrt(4*a*b));
d = 1.1;
epsilon = 0.9;

% Gamma agent parameters
c1_g = 2;
c2_g = 2;
x_g = zeros(T, 1); % Initial x positions for gamma leader
y_g = zeros(T, 1); % Initial y positions for gamma leader
z_g = zeros(T, 1); % Initial z positions for gamma leader
p_xg = 2; % Constant x velocity
p_yg = 0; % No y velocity
p_zg = 0; % No z velocity
x_g(1) = -3;
y_g(1) = 0;
z_g(1) = 0;

% Beta agent parameters
c1_b = 3;
c2_b = 3;
Ms = [0, -1.5, 0, 1; 0, 1.5, 0, 1];

% Assign agents random initial positions
x = zeros(T, N);
y = zeros(T, N);
z = zeros(T, N);
p_x = zeros(T, N);
p_y = zeros(T, N);
p_z = zeros(T, N);
for i = 1:N 
    x(1,i) = -4 * (rand + 0.4);
    y(1,i) = 4 * (rand) - 2.5;
    z(1,i) = 2 * (rand) - 1;
end

% Prepare the figure for real-time drawing
fig = figure;
set(gca, 'xlim', [-L L], 'ylim', [-L L], 'zlim', [-L L]);
axis square;
grid on;
hold on;
view(3); % Set the view to 3D
title('3D Flocking Squeeze', 'FontSize', 14, 'FontWeight', 'bold');

% Initialize the VideoWriter object
vidObj6 = VideoWriter('3d_flocking_squeeze.avi');
open(vidObj6);

% Simulation Loop
ui_alpha = zeros(3, N);
ui_gamma = zeros(3, N);
ui_beta = zeros(3, N);

% Pre-create obstacle spheres for efficiency
obstacle_spheres = gobjects(size(Ms, 1), 1);
for n = 1:size(Ms, 1)
    [X, Y, Z] = sphere;
    obstacle_spheres(n) = surf(X * Ms(n,4) + Ms(n,1), Y * Ms(n,4) + Ms(n,2), Z * Ms(n,4) + Ms(n,3), 'EdgeColor', 'none', 'FaceColor', 'k');
end

for k = 1:T-1
    % Update gamma leader's position
    x_g(k+1) = x_g(k) + timeStep * p_xg;
    y_g(k+1) = y_g(k) + timeStep * p_yg;
    z_g(k+1) = z_g(k) + timeStep * p_zg;

    % Update agents' positions
    scatter3_hdl = scatter3(x(k,:), y(k,:), z(k,:), 36, 'b', 'filled');
    scatter3_hdl_g = scatter3(x_g(k), y_g(k), z_g(k), 36, 'r', 'filled');

    % Logic for movement, forces, interactions
    for i = 1:N
        q_i = [x(k,i); y(k,i); z(k,i)];
        p_i = [p_x(k,i); p_y(k,i); p_z(k,i)];
        ui_alpha(:, i) = zeros(3, 1);
        ui_gamma(:, i) = zeros(3, 1);
        ui_beta(:, i) = zeros(3, 1);

        for j = 1:N
            if i ~= j
                q_j = [x(k,j); y(k,j); z(k,j)];
                p_j = [p_x(k,j); p_y(k,j); p_z(k,j)];
                distance = norm(q_j - q_i);
                if distance < r
                    n_ij = sigma_epsilon(epsilon, (q_j - q_i));
                    a_ij = rho_h(h, distance / r);
                    ui_alpha(:, i) = ui_alpha(:, i) + (phi_alpha(distance, r, h, a, b, c, d) * n_ij) + (a_ij * (p_j - p_i));
                end
            end
        end

        % Beta agent obstacle avoidance
        for n = 1:size(Ms, 1)
            midpoint = Ms(n, 1:3)'; % 3D midpoint of the obstacle
            radius = Ms(n, 4);

            mu_b = mu(midpoint, q_i, radius);
            a_kb = a_k(q_i, midpoint);

            q_ik = mu_b * q_i + (1 - mu_b) * midpoint; % Interpolated position relative to obstacle
            p_ik = mu_b * (eye(3) - a_kb * a_kb.') * p_i; % Adjusted velocity

            b_ik = rho_h(h, norm(q_ik - q_i) / d);
            n_ik = sigma_epsilon(epsilon, (q_ik - q_i));

            ui_beta(:, i) = ui_beta(:, i) + c1_b * phi_beta(norm(q_ik - q_i), d, h) * n_ik + c2_b * b_ik * (p_ik - p_i);
        end

        % Calculate the influence from the gamma leader
        q_g = [x_g(k); y_g(k); z_g(k)];
        p_g = [p_xg; p_yg; p_zg];
        ui_gamma(:, i) = -c1_g * sigma_epsilon(1, (q_i - q_g)) - c2_g * (p_i - p_g);

        % Aggregate forces for each agent
        ui(:, i) = ui_alpha(:, i) + ui_gamma(:, i) + ui_beta(:, i);

        % Update positions and velocities
        x(k+1, i) = x(k, i) + timeStep * p_x(k, i);
        p_x(k+1, i) = p_x(k, i) + timeStep * ui(1, i);
        y(k+1, i) = y(k, i) + timeStep * p_y(k, i);
        p_y(k+1, i) = p_y(k, i) + timeStep * ui(2, i);
        z(k+1, i) = z(k, i) + timeStep * p_z(k, i);
    end

    drawnow; % Ensure plot updates before capturing the frame
    frame = getframe(fig); % Capture the current figure
    writeVideo(vidObj6, frame); % Write frame to video

    delete(scatter3_hdl); % Remove previous frame's agent markers
    delete(scatter3_hdl_g); % Remove previous frame's gamma leader marker
end

close(vidObj6); % Close the VideoWriter object

%% Helper Functions %%
function rho_h = rho_h(h, z)
    if z >= 0 && z < h
        rho_h = 1;
    elseif z >= h && z <= 1
        rho_h = 0.5*(1 + cos(pi*((z-h)/(1-h))));
    else
        rho_h = 0;
    end
end

function sigma_epsilon = sigma_epsilon(epsilon, z)
    sigma_epsilon = z / sqrt(1 + epsilon * (norm(z)^2)); 
end

function phi = phi(a, b, c, z)
    phi = 0.5 * ((a + b) * sigma_epsilon(1, z) * (z + c) + (a - b));
end

function phi_alpha = phi_alpha(z, r, h, a, b, c, d)
    phi_alpha = rho_h(h, (z / r)) * phi(a, b, c, (z - d));
end

function phi_beta = phi_beta(z, d, h)
    phi_beta = rho_h(h, (z / d)) * (sigma_epsilon(1, z - d) - 1);
end

function mu = mu(y, q, r)
    mu = r / norm(q-y);
end

function a_k = a_k(q, y)
    a_k = (q-y) / norm(q-y);
end
