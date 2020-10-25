function Controllers()
close all;
    
    %% Better Unicycle
%     vdmax = 0.6*1.0; % Maximum desired velocity 
%     wdmax =3*0.1; % Maximum desired angular velocity
%     amax = 0.1; % Maximum acceleration
%     alphamax = 0.05; % Maxiumum angular acceleration
%     
%     % Calculate point control gains
%     A = [zeros(2)]
%     B = [eye(2)]
%     Q = [1/vdmax^2 0;0 1/wdmax^2]
%     R = [1/amax^2 0; 0 1/alphamax^2]
%     K = lqr(A, B, Q, R)
%     
%     veh = BetterUnicycle;
%     
%     u = @(t,x)constantRadiusBetterUnicycle(t,x,veh, K);
%     x0 = [0; 0; 0; 0; 0];

    
    %% Smooth Differential Drive 
%     vdmax = .1*1.0; % Maximum desired velocity 
%     wdmax = 2*0.1; % Maximum desired angular velocity
%     armax = 0.1; % Maxiumum angular acceleration
%     almax = 0.1; % Maxiumum angular acceleration
%     
%     veh = SmoothDifferentialDrive;
%     r = veh.r;
%     L = veh.L;
%     % Calculate point control gains
%     A = [zeros(2)]
%     M = [r/2 r/2;r/L -r/L]
%     Q = [1/vdmax^2 0;0 1/wdmax^2]
%     R = [1/armax^2 0; 0 1/almax^2]
%     K = lqr(A, M, Q, R)
% 
%     u = @(t,x)constantRadiusSmoothDiffDrive(t,x,veh, K);
%     x0 = [0; 0; 0; 0; 0];

    %% Smooth Bicycle Drive 
    vdmax = 1.0; % Maximum deviation velocity 
    wdmax = 0.1; % Maximum deviation angular velocity
    admax = 0.1; % Maxiumum acceleration deviation
    alphadmax = 0.1; % Maxiumum angular acceleration deviation
    
    veh = SmoothBicycleDrive;
    r = veh.r;
    L = veh.L;
    % Calculate point control gains
    A = [zeros(2)];
    B = [eye(2)];
    Q = [1/vdmax^2 0;0 1/wdmax^2];
    R = [1/admax^2 0; 0 1/alphadmax^2];
    K = lqr(A, B, Q, R);

    u = @(t,x)constantRadiusSmoothBicycle(t,x,veh, K);
    x0 = [0; 0; 0; 0; 0; 0];

        %% Better Unicycle
%     vdmax = 0.6*1.0; % Maximum desired velocity 
%     wdmax =3*0.1; % Maximum desired angular velocity
%     amax = 0.1; % Maximum acceleration
%     alphamax = 0.05; % Maxiumum angular acceleration
%     
%     % Calculate point control gains
%     A = [zeros(2)]
%     B = [eye(2)]
%     Q = [1/vdmax^2 0;0 1/wdmax^2]
%     R = [1/amax^2 0; 0 1/alphamax^2]
%     K = lqr(A, B, Q, R)
%     
%     veh = BetterUnicycle;
%     
%     u = @(t,x)constantRadiusBetterUnicycle(t,x,veh, K);
%     x0 = [0; 0; 0; 0; 0];

    
    %% Better Differential - Input/output go to goal
    % Set maximum values
%     p_max = 0.5; % Maximum desired position deviation
%     p_dot_max = 0.1; % Maximum desired velocity deviation
%     p_ddot_max = 0.1; % Maximum desired acceleration deviation
%     
%     veh = SmoothDifferentialDrive;
%     r = veh.rad;
%     L = veh.L;
% %     R_eps =[cos(theta) -eps*sin(
%     
%     % Calculate point control gains
% 
%     A = [zeros(2) eye(2); zeros(2,4)]; 
%     M = [r/2 r/2;r/L -r/L];
%     B = [zeros(2); eye(2)];%
%     Q = diag([1/p_max^2, 1/p_max^2, 1/p_dot_max^2, 1/p_dot_max^2]);
%     R = diag([1/p_ddot_max^2, 1/p_ddot_max^2]);
%     K = lqr(A, B, Q, R);
%     
%     % Create the vehicle and desired values   
%     q_d = [4; 3;0; 0;];
%     eps = 0.10;
%     
%     % Create the controller and initial conditions
%     u = @(t,x)goToGoalApproximateDiffeomorphismDifferential(t, x, veh, q_d, eps, K);
%     x0 = [0; 0; 0; 0; 0];

    %% Better Unicycle - Input/output go to goal
    % Set maximum values
%     p_max = 0.5; % Maximum desired position deviation
%     p_dot_max = 0.1; % Maximum desired velocity deviation
%     p_ddot_max = 0.25; % Maximum desired acceleration deviation
%     
%     % Calculate point control gains
%     A = [zeros(2) eye(2); zeros(2,4)]; 
%     B = [zeros(2); eye(2)];
%     Q = diag([1/p_max^2, 1/p_max^2, 1/p_dot_max^2, 1/p_dot_max^2]);
%     R = diag([1/p_ddot_max^2, 1/p_ddot_max^2]);
%     K = lqr(A, B, Q, R);
%     
%     % Create the vehicle and desired values
%     veh = BetterUnicycle;    
%     q_d = [5; -3; 0; 0];
%     eps = 1.0;
%     
%     % Create the controller and initial conditions
%     u = @(t,x)goToGoalApproximateDiffeomorphismUnicycle(t, x, veh, q_d, eps, K);
%     x0 = [0; 0; 0; 0; 0];

    
    %% Simulate
    % Select the integration mode
%     integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
    dt = 0.1;
    tf =25;
    [tmat, xmat] = integrator(t0, dt, tf, x0, veh, u);
    
    % Plot the velocities
    veh.plotVelocitiesAndInput(tmat, xmat, u);
    
    % Plot the state
    figure;
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
%        pause(dt);
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
        xdot = veh.kinematics(t, x, u(t,x));
        
        % Update the state
        x = x + dt * xdot;
        
        % Store the state
        xmat(:,k) = x;
    end

end

function u = constantRadiusBetterUnicycle(t, x, veh, K)
    % Simple feedback control to calculate inputs
    vd = 2.5;
    wd = 0.4;
    
    [v,w] = veh.getVelocities(t, x, 0); 
    x = [v; w];
    xd = [vd; wd];
    z = x - xd;
    
    u = -K * z;
end

function u = constantRadiusSmoothDiffDrive(t, x, veh, K)
    % Simple feedback control
%     ur = 0;
%     ul = 0;
% %     u[1] = velosity of left wheel
% %     u[2] = velosity of right wheel
    vd = 2.5;
    wd = 0.4;
    
    [v,w] = veh.getVelocities(t, x, 0); 
    x = [v; w];
    xd = [vd; wd];
    z = x - xd;
    
    u = -K * z;   
end

%% Simple Smooth Car / Bicycle
function u = constantRadiusSmoothBicycle(t, x, veh, K)
%     u = [10; .2]; % .7854 corresponds to a steering angle that will 
%                      %     produce w = 1 for L and v = 1

    vd = 2.5;
    wd = 0.4;
    
    L = veh.L;
    
%     [v,w] = veh.getVelocities(t, x, 0); 
%     x = [v; w];
%     xd = [vd; wd];
%     z = x - xd;
%     
%     u = -K * z;
% 
%     vd = 2.5;
%     wd = 0.4;
    
    [v,w] = veh.getVelocities(t, x, 0); 
    x = [v; w];
    xd = [vd; wd];
    z = x - xd;
    
    u_t = -K * z;
    a = u_t(1);
    alpha = u_t(2);
    phidot = -L*(alpha/a)*csc(L*w/v)^2;
    u = [a, phidot];

end

function u = goToGoalApproximateDiffeomorphismDifferential(t, x, veh, q_d, eps, K)
    % Get states
    x_pos = x(veh.x_ind);
    y_pos = x(veh.y_ind);
    [wr, wl, v, w] = veh.getVelocities(t, x, 0);
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
    u_point = -K*(q - q_d) ;
    
    % Calculate the control inputs
    aVec = R_e_inv*u_point - w_hat_e*[v; w]; 
    a = aVec(1);
    alpha = aVec(2);
    alpha_r  = (alpha*veh.L+2*a)/(2*veh.rad);
    alpha_l = (2/veh.rad)*a - alpha_r;
    u = [alpha_r; alpha_l] ;
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