classdef BetterUnicycle < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    properties
        a_ind = 1; % First input
        alpha_ind = 2; % Second input
        
    end
    methods
        function obj = BetterUnicycle()
            obj = obj@VehicleKinematics(3);
            obj.plot_path = true;
            obj.c = 'g';
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            % Time derivative: [xdot; ydot; theta_dot] = [vcos(theta); vsin(theta); omega]
            a = u(obj.a_ind);
            alpha = u(obj.alpha_ind);
                        
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(3,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}            
        end        
    end 
end