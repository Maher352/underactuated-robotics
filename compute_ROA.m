% compute_ROA.m
% ROA w.r.t. ball initial position (xb0, yb0)

%   Blue = cart catches the ball (|error| <= err_margin)
%   Orange   = cart misses the ball

clear; clc; close all;

%% Parameters (same as in run_traj_LQR.m)
params.mc = 1.0;    % cart mass [kg]
params.b  = 0.1;    % viscous friction [N.s/m]
params.g  = 9.81;   % gravity [m/s^2]

%% Fixed initial velocities for the ball
vxb0  = 0.0;        % initial horizontal velocity of the ball [m/s]
vyb0  = 0.0;        % initial vertical velocity of the ball [m/s]

%% Fixed initial conditions for the cart
xc0    = 0.0;       % cart initial position [m]
xcdot0 = 0.0;       % cart initial velocity [m/s]
y_catch = 0.0;      % height of the cart [m]

%% Actuator Constraints
params.u_max = 15;          % maximum control force [N]
params.u_min = -15;         % minimum control force [N]

%% Cart and ball dimensions
cart_w = 0.3;   % cart width
ball_r = 0.05;  % ball radius

% Acceptable position error for the cart center
err_margin = cart_w/2 - ball_r;   % [m]

%% LQR design for the cart (linearized model)

mc = params.mc;
b  = params.b;

% Linearized cart dynamics: x = [x_c; xcdot]
A_cart = [0     1;
          0  -b/mc];
B_cart = [0;
          1/mc];

% LQR weights (same as in your script)
Q = diag([50, 5]);   % cost on cart position error and velocity
R = 10;              % cost on control 
K = lqr(A_cart, B_cart, Q, R);

fprintf('LQR gain K:\n'); disp(K);

%% Grid of initial ball positions (xb0, yb0)

xb0_vals = -2.5 : 0.05 : 2.5;   % horizontal range [-2.5, 2.5]
yb0_vals =  0.0 : 0.05 : 5.0;   % vertical range [0, 5]

Nx = numel(xb0_vals);
Ny = numel(yb0_vals);

catch_map = zeros(Ny, Nx);   % initialy 0

%% Main sweep over grid

for iy = 1:Ny
    for ix = 1:Nx

        xb0 = xb0_vals(ix);
        yb0 = yb0_vals(iy);

        % 1) Predict ball landing time and horizontal position
        [t_hit, x_land, valid] = predict_landing(xb0, yb0, vxb0, vyb0, params.g, y_catch);

        % If no valid landing, treat as miss
        if ~valid
            caught = false;
        else
            % 2) Build cubic reference trajectory x_ref(t) for this landing
            T  = t_hit;
            a0 = xc0;
            a1 = xcdot0;

            M   = [T^2   T^3;
                   2*T  3*T^2];
            rhs = [x_land - a0 - a1*T;
                   -a1];

            coeffs = M \ rhs;
            a2 = coeffs(1);
            a3 = coeffs(2);

            traj.a0 = a0;
            traj.a1 = a1;
            traj.a2 = a2;
            traj.a3 = a3;
            traj.T  = T;

            % 3) Simulate full cart+ball system with LQR tracking
            % State: [x_c; xcdot; x_b; y_b; xbdot; ybdot]
            x0 = [xc0;
                  xcdot0;
                  xb0;
                  yb0;
                  vxb0;
                  vyb0];

            t_final = t_hit + 0.2;
            tspan  = [0 t_final];

            odefun = @(t,x) closed_loop_dynamics(t, x, K, params, traj);

            try
                [t_sim, X_sim] = ode45(odefun, tspan, x0);
            catch
                % If the integrator fails, treat as miss
                caught = false;
                catch_map(iy, ix) = 0;
                continue;
            end

            x_c_sim = X_sim(:,1);
            x_b_sim = X_sim(:,3);

            % 4) Evaluate positions at impact time
            x_b_hit = interp1(t_sim, x_b_sim, t_hit, 'linear', 'extrap');
            x_c_hit = interp1(t_sim, x_c_sim, t_hit, 'linear', 'extrap');
            err     = x_c_hit - x_b_hit;

            % 5) Catch checker
            caught = (abs(err) <= err_margin);
        end

        catch_map(iy, ix) = double(caught); 

    end
end

%% Plot ROA in (xb0, yb0) space

figure;
imagesc(xb0_vals, yb0_vals, catch_map);   
set(gca, 'YDir', 'normal');              

colormap([1 0.64 0.06; 0.53 0.81 0.92]);                % 0 = orange (miss), 1 = blue (catch)
colorbar('Ticks',[0,1], 'TickLabels',{'Miss','Catch'});

xlabel('x_b(0) [m]');
ylabel('y_b(0) [m]');
title('ROA in Ball Initial Position Space (x_b(0), y_b(0))');
grid on;

%% ===== Helper functions (same as in your main script) =====

function [xr, vr, ar] = ref_traj(t, traj)
    % Cubic reference trajectory x_ref(t)
    a0 = traj.a0; a1 = traj.a1; a2 = traj.a2; a3 = traj.a3;

    xr = a0 + a1*t + a2*t^2 + a3*t^3;          % position
    vr =      a1   + 2*a2*t + 3*a3*t^2;        % velocity
    ar =             2*a2   + 6*a3*t;          % acceleration
end

function xdot = closed_loop_dynamics(t, x, K, params, traj)
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

    % Reference trajectory at time t
    [x_ref, v_ref, a_ref] = ref_traj(t, traj);

    % Tracking error
    e = [x_c - x_ref;
         xcdot - v_ref];

    % Feedforward term to follow desired acceleration
    u_ff = mc * a_ref + b * v_ref;

    % LQR feedback on tracking error
    u = u_ff - K * e;

    % Saturate control input to actuator limits
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
