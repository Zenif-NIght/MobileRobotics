
function prob3_3()
    close all;    
%     %% Better Unicycle
%         veh = BetterUnicycle;
%         u = @(t,x)constantRadiusBetterUnicycle(t,x,veh);
%         x0 = [0; 0; 0; 0; 0];    
    %% Continuous Steering Bicycle - LaValle (13.47)
    %     veh = ContinuousSteeringBicycle;
    %     u = @(t,x)constantRadiusContinuousSteeringBicycle(t,x,veh);
    %     x0 = [0; 0; 0; 0; 0];    
    %% Continuous Steering Bicycle - LaValle (13.48)
    %     veh = ContinuousSteeringBicycle13_48;
    %     u = @(t,x)constantRadiusContinuousSteeringBicycle13_48(t,x,veh);
    %     x0 = [0; 0; 0; 0; 0; 0];    
%     %% Smooth Differential Drive
%         veh = SmoothDifferentialDrive;
%         u = @(t,x)constantRadiusSmoothDiffDrive(t,x,veh);
%         x0 = [0; 0; 0; 0; 0];    
    %% Smooth Differential Drive - State feedback - Place    
%     % Calculate gain    
%     A = zeros(2);
%     B = [1/8 1/8; 1/4 -1/4];
%     poles = [-1; -2];
%     K = place(A, B, poles);
%     vd = 5;
%     wd = 0.5;
%     veh = SmoothDifferentialDrive;
%     u = @(t,x)constantRadiusSmoothDiffDriveStateFeedback(t,x,veh, K, vd, wd);
%     x0 = [0; 0; 0; 0; 0];
    %% Smooth Differential Drive - State feedback - LQR%
    % Calculate gain
        A = zeros(2);               
%     State matrices \dot{x} = Ax + Bu
        B = [1/8 1/8; 1/4 -1/4];
        Q = [10 0; 0 10];
%     LQR matrices: x'Qx + u'Ru
        R = [1 0; 0 1];
        K = lqr(A, B, Q, R);
        vd = 5;
        wd = 0.5;
        
        veh = SmoothDifferentialDrive;
        u = @(t,x)constantRadiusSmoothDiffDriveStateFeedback(t,x,veh, K, vd, wd);
        x0 = [0; 0; 0; 0; 0];    
    %% Simulate    
    % Select the integration mode    
%     integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);    
    integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);    
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
        veh.plotState(tmat(k), xmat(:,k));       pause(dt);    
    end
end

function [tmat, xmat] = integrateODE(t0, dt, tf, x0, veh, u)
    % Input parameters
    %   t0: initial time
    %   dt: time step for return data
    %   tf: final time%   x0: initial state
    %   veh: instantiation of VehicleKinematics class
    %   u: function handle which takes input arguments of (t,x)    
    % Integrate forward in time    [tmat, xmat] = ode45(@(t,x)veh.kinematics(t, x, u(t,x)), [t0:dt:tf], x0);
    % Transpose the outputs    tmat = tmat';    xmat = xmat';
end

function [tmat, xmat] = integratorEuler(t0, dt, tf, x0, veh, u)
    % Input parameters
    %   t0: initial time
    %   dt: time step for return data
    %   tf: final time
    %   x0: initial state
    %   veh: instantiation of VehicleKinematics class%   u: function handle which takes input arguments of (t,x)    
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
function u = constantRadiusBetterUnicycle(t, x, veh)   
    % Define desired values   
    v_d = 5;  
    w_d = 0.5; 
    % Extract states   
    v = x(veh.v_ind);  
    w = x(veh.w_ind);  
    % Simple feedback control to calculate inputs 
    u_v = -(v-v_d);    
    u_w = -(w-w_d);    
    u = [u_v; u_w];
end
        
function u = constantRadiusContinuousSteeringBicycle(t, x, veh)  
    % Define desired values 
    v_d = 5;   
    phi_d = 0.0997;
    % Extract states  
    v = x(veh.v_ind); 
    phi = x(veh.phi_ind);  
    % Simple feedback control to calculate inputs 
    u_v = -(v-v_d);  
    phi_err = -(phi-phi_d); 
    phi_err = atan2(sin(phi_err), cos(phi_err)); 
    % Angle trick to get error angle between -pi and pi 
    u_phi = phi_err;   
    u = [u_v; u_phi];
end
function u = constantRadiusContinuousSteeringBicycle13_48(t, x, veh)  
        % Define desired values 
        v_d = 5;    
        phi_d = 0.0997; 
        % Extract states   
        v = x(veh.v_ind);   
        phi = x(veh.phi_ind); 
        phi_dot = x(veh.phi_dot_ind);   
        % Simple feedback control to calculate inputs   
        u_v = -(v-v_d);   
        phi_err = -(phi-phi_d);  
        phi_err = atan2(sin(phi_err), cos(phi_err));
        % Angle trick to get error angle between -pi and pi  
        u_phi = 5*phi_err- phi_dot; 
        u = [u_v; u_phi];
    end

% function u = constantRadiusBetterUnicycle(t,x,veh)
%             % Define desired values    
%     wrd = 21;  
%     wld = 19;  
% 
%     % Extract states 
%     wr = x(veh.ind_wr); 
%     wl = x(veh.ind_wl);  
%     % Simple feedback control  
%     ur = -(wr-wrd);  
%     ul = -(wl-wld); 
%     u = [ur; ul];
%     end
    
function u = constantRadiusSmoothDiffDrive(t, x, veh)
    % Define desired values    
    wrd = 21;  
    wld = 19;  

    % Extract states 
    wr = x(veh.ind_wr); 
    wl = x(veh.ind_wl);  
    % Simple feedback control  
    ur = -(wr-wrd);  
    ul = -(wl-wld); 
    u = [ur; ul];
    end

function u = constantRadiusSmoothDiffDriveStateFeedback(t, x, veh, K, vd, wd)  
    % Form the velocity state  
    [v, w] = veh.getVelocities(t, x, 0); 
    x_vel = [v; w];   
    % Define desired values 
    xd = [vd; wd];   
    % Define shifted state  
    z = x_vel - xd;   
    % Calculate control 
    u = -K*z;
end

