function Controllers()
close all;
    
    %% Better Unicycle
%     veh = BetterUnicycle;
%     u = @(t,x)constantRadiusBetterUnicycle(t,x,veh);
%     x0 = [0; 0; 0; 0; 0];
    
    %% Smooth Differential Drive 
%     veh = SmoothDifferentialDrive;
%     u = @(t,x)constantRadiusSmoothDiffDrive(t,x,veh);
%     x0 = [0; 0; 0; 0; 0];

    %% Better Unicycle - Input/output go to goal
    % Set maximum values
    p_max = 0.5; % Maximum desired position deviation
    p_dot_max = 0.1; % Maximum desired velocity deviation
    p_ddot_max = 0.25; % Maximum desired acceleration deviation
    
    % Calculate point control gains
    A = [zeros(2) eye(2); zeros(2,4)]; 
    B = [zeros(2); eye(2)];
    Q = diag([1/p_max^2, 1/p_max^2, 1/p_dot_max^2, 1/p_dot_max^2]);
    R = diag([1/p_ddot_max^2, 1/p_ddot_max^2]);
    K = lqr(A, B, Q, R);
    
    % Create the vehicle and desired values
    veh = BetterUnicycle;    
    q_d = [5; -3; 0; 0];
    eps = 1.0;
    
    % Create the controller and initial conditions
    u = @(t,x)goToGoalApproximateDiffeomorphismUnicycle(t, x, veh, q_d, eps, K);
    x0 = [0; 0; 0; 0; 0];

    
    %% Simulate
    % Select the integration mode
    integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    %integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
    dt = 0.1;
    tf = 10;
    [tmat, xmat] = integrator(t0, dt, tf, x0, veh, u);
    
    % Plot the velocities
    veh.plotVelocitiesAndInput(tmat, xmat, u);
    
    % Plot the state
    figure;
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
       pause(dt);
    end   
        
end

function [tmat, xmat] = integrateODE(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Integrate forward in time
    [tmat, xmat] = ode45(@(t,x)veh.kinematics(t, x, u(t,x)), [t0:dt:tf], x0);
    
    % Transpose the outputs
    tmat = tmat';
    xmat = xmat';    
end

function [tmat, xmat] = integratorEuler(t0, dt, tf, x0, veh, u)
% Input parameters
%   t0: initial time
%   dt: time step for return data
%   tf: final time
%   x0: initial state
%   veh: instantiation of VehicleKinematics class
%   u: function handle which takes input arguments of (t,x)

    % Initialize state data
    tmat = [t0:dt:tf]';
    len = length(tmat);
    xmat = zeros(veh.dimensions, len);
    xmat(:,1) = x0;
    
    % Loop through and calculate the state
    x = x0;
    for k = 1:len
        % Calculate state update equation
        t = tmat(k);
        xdot = veh.kinematics(t, x, u(x,t));
        
        % Update the state
        x = x + dt * xdot;
        
        % Store the state
        xmat(:,k) = x;
    end

end

function u = constantRadiusBetterUnicycle(t, x, veh)
    % Simple feedback control to calculate inputs
    u_v = 0;
    u_w = 0;
    u = [u_v; u_w];
end

function u = constantRadiusSmoothDiffDrive(t, x, veh)
    % Simple feedback control
    ur = 0;
    ul = 0;
    
    u = [ur; ul];
end

function u = goToGoalApproximateDiffeomorphismUnicycle(t, x, veh, q_d, eps, K)
    % Get states
    x_pos = x(veh.x_ind);
    y_pos = x(veh.y_ind);
    [v, w] = veh.getVelocities(t, x, 0);
    th = x(veh.th_ind);
    c = cos(th);
    s = sin(th);
    
    % Form espilon variables
    w_hat_e = [0 -eps*w; w/eps 0];
    R_e = [c -eps*s; s eps*c];
    R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
    
    % Calculate current values of espilon state
    q_eps = [x_pos; y_pos] + eps * [c; s];
    q_eps_dot = R_e*[v; w];
    q = [q_eps; q_eps_dot];
    
    % Calculate point control
    u_point = -K*(q - q_d);
    
    % Calculate the control inputs
    u = R_e_inv*u_point - w_hat_e*[v; w];    
end