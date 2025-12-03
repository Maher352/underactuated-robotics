% run_LQR.m
% Use LQR to drive the cart to the predicted ball landing position.

clear; clc; close all;

%% Parameters
params.mc = 1.0;    % cart mass [kg]
params.b  = 0.1;    % viscous friction [N.s/m]
params.g  = 9.81;   % gravity [m/s^2]

%% Initial conditions for the ball
xb0   = 0.5;        % initial horizontal position of the ball [m]
yb0   = 1.0;        % initial height of the ball [m]
vxb0  = 0.5;        % initial horizontal velocity of the ball [m/s]
vyb0  = -0.2;       % initial vertical velocity of the ball [m/s]

%% Initial conditions for the cart
xc0    = 0.0;       % cart initial position [m]
xcdot0 = 0.0;       % cart initial velocity [m/s]
y_catch = 0.0;      % height of the cart [m]

%% Actuator Constraints
params.u_max = 15;          % maximum control force [N]
params.u_min = -15;         % minimum control force [N]

%% Cart and ball dimensions
cart_w = 0.3;   % cart width
cart_h = 0.1;   % cart height
ball_r = 0.05;  % ball radius

%% 1) Predict ball landing time and horizontal position

[t_hit, x_land, valid] = predict_landing(xb0, yb0, vxb0, vyb0, params.g, y_catch);

if ~valid
    fprintf('Ball never reaches y = %.2f m.\n', y_catch);
    return;
end

fprintf('Predicted landing time:  t_hit = %.4f s\n', t_hit);
fprintf('Predicted landing x-pos: x_land = %.4f m\n', x_land);

x_ref = x_land;

%% 2) Design LQR for the cart dynamics

mc = params.mc;
b  = params.b;

% Linearized cart dynamics: x = [x_c; xcdot]
% using the dynamics equation:
% x_cdot = xcdot
% xcdotdot = (u - b*xcdot) / mc

A_cart = [0     1;
          0  -b/mc];
B_cart = [0;
          1/mc];

% LQR weights (can be tuned)
Q = diag([100, 10]);   % cost on cart position error and velocity
R = 0.1;               % cost on control 

K = lqr(A_cart, B_cart, Q, R);

disp('LQR gain K (u = -K * [x_c - x_ref; xcdot]):');
disp(K);

%% 3) Simulate full cart+ball system with LQR controller

x0 = [xc0;
      xcdot0;
      xb0;
      yb0;
      vxb0;
      vyb0];

t_final = t_hit + 0.2;
tspan   = [0 t_final];

odefun = @(t, x) closed_loop_dynamics(t, x, K, x_ref, params);

[t, X] = ode45(odefun, tspan, x0);

x_c   = X(:,1);
x_b   = X(:,3);
y_b   = X(:,4);

%% 4) Evaluate cart position at impact time

x_b_hit = interp1(t, x_b, t_hit);
x_c_hit = interp1(t, x_c, t_hit);
err = x_c_hit - x_b_hit;

fprintf('\nAt t_hit:\n');
fprintf('  Ball x_b(t_hit)  = %.4f m\n', x_b_hit);
fprintf('  Cart x_c(t_hit)  = %.4f m\n', x_c_hit);
fprintf('  Cart error       = %.4e m\n', err);

% Acceptable position error for the cart center based on object dimensions
err_margin = cart_w/2 - ball_r;     % [m]
x_ok_upper = x_land + err_margin;   
x_ok_lower = x_land - err_margin;   

% ---- Catch checker ----
if abs(err) <= err_margin
    caught = true;
    fprintf('Result: BALL CAUGHT; |error| = %.4f <= %.4f (max allowed)\n', ...
            abs(err), err_margin);
else
    caught = false;
    fprintf('Result: BALL MISSED; |error| = %.4f > %.4f (max allowed)\n', ...
            abs(err), err_margin);
end

%% 5) Plot results

figure;

% (1) Ball trajectory (x vs y)
subplot(1,2,1);
plot(x_b, y_b, 'LineWidth', 1.5); hold on;
yline(y_catch, '--');
plot(x_land, y_catch, 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
xlabel('x [m]');
ylabel('y [m]');
title('Ball trajectory');
legend('Ball path', 'Catch height', 'Predicted landing', 'Location', 'best');
grid on;

% (2) x vs time (cart and ball)
subplot(1,2,2);
plot(t, x_b, 'LineWidth', 1.5); hold on;
plot(t, x_c, 'LineWidth', 1.5);
xline(t_hit, '--');
yline(x_ok_upper, ':', 'LineWidth', 1.2);
yline(x_ok_lower, ':', 'LineWidth', 1.2);
xlabel('t [s]');
ylabel('x [m]');
title('Horizontal positions vs time');
legend('Ball x_b(t)', 'Cart x_c(t)', 't_{hit}', 'Acceptable band', 'Location', 'best');
grid on;

%% 8) Animation

% Figure setup for animation
figure('Color','w');
hold on;

% Axes limits
xmin = min([x_c; x_b]) - 0.5;
xmax = max([x_c; x_b]) + 0.5;
ymin = min([y_b; y_catch]) - 0.5;
ymax = max([yb0; y_catch]) + 0.5;

axis([xmin xmax ymin ymax]);
axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Cart-Ball LQR Catch Animation');

% Draw track
plot([xmin xmax], [y_catch y_catch], 'k-', 'LineWidth', 1);

% Draw vertical line at predicted landing x (for reference)
plot([x_land x_land], [ymin ymax], 'k--');


hCart = rectangle('Position',[x_c(1)-cart_w/2, y_catch, cart_w, cart_h], ...
                  'FaceColor',[0.8 0.2 0.8], 'EdgeColor','k');
hBall = rectangle('Position',[x_b(1)-ball_r, y_b(1)-ball_r, 2*ball_r, 2*ball_r], ...
                  'Curvature',[1 1], 'FaceColor',[0.3 0.8 0.2], ...
                  'EdgeColor','k');

% Animate
idx_stop = find(t <= t_hit, 1, 'last');

step = max(1, floor(idx_stop/300));  % ~300 frames max

for k = 1:step:idx_stop
    xc = x_c(k);
    xb = x_b(k);
    yb = y_b(k);

    % Update cart and ball
    set(hCart, 'Position', [xc - cart_w/2, y_catch, cart_w, cart_h]);
    set(hBall, 'Position', [xb - ball_r,  yb - ball_r, 2*ball_r, 2*ball_r]);

    drawnow;
    pause(0.01);
end

fprintf('\nSimulation and Animation complete. \n');

%% ===== Helper functions =====
function xdot = closed_loop_dynamics(t, x, K, x_ref, params)
    % x = [x_c; xcdot; x_b; y_b; xbdot; ybdot]

    mc = params.mc;
    b  = params.b;
    g  = params.g;

    x_c    = x(1);
    xcdot  = x(2);
    x_b    = x(3);
    y_b    = x(4);
    xbdot  = x(5);
    ybdot  = x(6);

    e = [x_c - x_ref;
        xcdot];

    u = -K * e;

    % saturate control input to actuator limits
    if isfield(params, 'u_max')
        u = min(u, params.u_max);
    end
    if isfield(params, 'u_min')
        u = max(u, params.u_min);
    end

    % Cart dynamics
    xcdotdot = (u - b*xcdot) / mc;

    % Ball dynamics
    xbdotdot = 0;
    ybdotdot = -g;

    xdot = [xcdot;
            xcdotdot;
            xbdot;
            ybdot;
            xbdotdot;
            ybdotdot];
end
