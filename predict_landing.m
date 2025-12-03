function [t_hit, x_land, valid] = predict_landing(xb0, yb0, vxb0, vyb0, g, y_catch)
    % Predict time and x-position where the ball hits y = y_catch
    % Returns valid = false if there is no real positive solution

    % Quadratic coefficients: (1/2)g t^2 - vy0 t - (yb0 - y_catch) = 0
    a = 0.5 * g;
    b = -vyb0;
    c = -(yb0 - y_catch);

    discriminant = b^2 - 4*a*c;

    if discriminant < 0
        t_hit = NaN;
        x_land = NaN;
        valid = false;
        return;
    end

    sqrtD = sqrt(discriminant);
    t1 = (-b + sqrtD)/(2*a);
    t2 = (-b - sqrtD)/(2*a);

    % We want the smallest positive time
    candidates = [t1, t2];
    candidates = candidates(candidates > 0);

    if isempty(candidates)
        t_hit = NaN;
        x_land = NaN;
        valid = false;
        return;
    end

    t_hit = min(candidates);
    x_land = xb0 + vxb0 * t_hit;
    valid = true;
end
