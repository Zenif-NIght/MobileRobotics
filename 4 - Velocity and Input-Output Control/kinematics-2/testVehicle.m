function testVehicle()

    % Create the vehicle
%     veh = SimpleUnicycle;
%     u = @(t,x)constantRadiusUnicycle(t,x);

    veh = DifferentialDrive;
    u = @(t,x)constantDifferentialDive(t,x,veh);

%     veh = SimpleBicycle;
%     u = @(t,x)constantRadiusBicycle(t,x);
    
    % Select the integration mode
    %integrator = @(t0, dt, tf, x0, veh, u) integrateODE(t0, dt, tf, x0, veh, u);
    integrator = @(t0, dt, tf, x0, veh, u) integratorEuler(t0, dt, tf, x0, veh, u);
    
    % Integrate the state
    t0 = 0; 
    dt = 0.1;
    tf = 10;
    x0 = zeros(veh.dimensions,1);
    [tmat, xmat, umat, xdotmat] = integrator(t0, dt, tf, x0, veh, u);
    close all; 
    
    %% Plot the state
    subplot(2,2,1)
    for k = 1:length(tmat)
       veh.plotState(tmat(k), xmat(:,k));
       pause(dt);
    end
    title('position trajectory, Drive Path')


    subplot(2,2,2)
    plot(tmat, xdotmat(veh.th_ind+1,:))
    title('phi vs time xdotmat(veh.th_ind+1,:) ')

    subplot(2,2,3)
    plot(tmat, xdotmat(veh.th_ind,:))
    title('Angluler velocity vs time xdotmat(veh.th_ind,:) ')
    
    subplot(2,2,4)
    hold on
    plot(tmat, umat(1,:))
    plot(tmat, umat(2,:))
    hold off
    legend('Linerar velocity','Angluler velocity')
    title('Control vs. Time u=ul,ur')

    
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

function [tmat, xmat, umat, xdotmat] = integratorEuler(t0, dt, tf, x0, veh, u)
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
    vList = zeros(1, len);
    xmat(:,1) = x0;
    
    % Loop through and calculate the state
    x = x0;
    for k = 1:len
        % Calculate state update equation
        t = tmat(k);
        xdot = veh.kinematics(t, x, u(t,u));
        
        % Update the state
        x = x + dt * xdot;
        
        % Store the state
        xmat(:,k) = x;
        umat(:,k) = u(t,x);
        xdotmat(:,k) = xdot;
    end

end

function u = constantRadiusUnicycle(t, x)
    u = [2.5; 0.4];
end

%% Differential Drive NEW
function u = constantDifferentialDive(t,x,veh)%(t, x)
%     u[1] = velosity of left wheel
%     u[2] = velosity of right wheel
    r = veh.rad;
    L = veh.L;
    M = [[r/2,r/2];[r/L,-r/L]];
    v= 2.5;
    w= 0.4;
    solution = (M^(-1))*[v;w];

    u =solution;

%     u = 10*[1; .9];
end

function u = constantRadiusBicycle(t, x)
    v= 2.5;
    w= 0.4;
    u = [v; w]; 
end

