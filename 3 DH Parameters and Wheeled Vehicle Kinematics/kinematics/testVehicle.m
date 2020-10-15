function testVehicle()
    close all; 
    % Create the vehicle
%     veh = SimpleUnicycle;
%     veh = SimpleBicycle;

    veh = BetterUnicycle;
%       veh = DifferentialDive;
    
    % Select the control
%     u = @(t,x)constantRadiusUnicycle(t,x);
%     u = @(t,x)constantRadiusBicycle(t,x);

%      u =@(t,x)constantDifferentialDive(t,x);
     u =@(t,x)BetterUnicycleDynamic(t, x)
    
    % Select the integration mode
%      integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
    dt = 0.1;
    tf = 10;
    x0 = zeros(veh.dimensions,1); % most of the time this is it [0; 0; 0];
    x0(2) =-5;
    [tmat, xmat, umat] = integrator(t0, dt, tf, x0, veh, u);
    
    % Plot the state
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
       pause(dt);
    end
    hold off
    figure('Name','Stuff vs Time');
    plot(tmat, xmat(4,:))
    hold on
%     figure('Name','Rotational velocity vs Time');
    plot(tmat, xmat(5,:))
    legend('Translational velocity','Rotational velocity')
    hold off
    
    figure('Name',(' Inputs  vs Time'));
    for iput = length(umat(1))
        plot(tmat, umat(iput,:))
        hold on
        
    end
    legend("Input 1  vs Time1","Input 2  vs Time2")
    hold off
    2+2;
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

function [tmat, xmat, umat] = integratorEuler(t0, dt, tf, x0, veh, u)
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
    umat = zeros(length(u(1,1)), len);
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
        umat(:,k) = u(x,t);
    end

end



%% Simple Car / Bicycle
function u = constantRadiusBicycle(t, x)
    u = [1; 0.7854]; % .7854 corresponds to a steering angle that will 
                     %     produce w = 1 for L and v = 1
end


%%  Simple Unicycle
function u = constantRadiusUnicycle(t, x)
    u = [1; 1];
end


%% Differential Drive NEW
function u = constantDifferentialDive(t,x)%(t, x)
%     u[1] = velosity of left wheel
%     u[2] = velosity of right wheel
    u = [1; .2];
end


%% Better Unicycle NEW
function u = BetterUnicycleDynamic(t, x)
    %    u1 = acceleration
    %    u2 = alpha
        u = [2.5;.5]; % 1
end

