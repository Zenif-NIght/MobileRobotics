classdef BetterUnicycle < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    properties
        a_ind = 1; % First input
        alpha_ind = 2; % Second input
        
        % other x vector indeces
        s_ind = 4;
        w_ind = 5;
        
    end
    methods
        function obj = BetterUnicycle()
            obj = obj@VehicleKinematics(5);
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
            s = x(obj.s_ind);
            w = x(obj.w_ind);
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = s * cos(theta); % \dot{x}
            xdot(obj.y_ind) = s * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}
            xdot(obj.s_ind) = a;
            xdot(obj.w_ind) = alpha;
        end        
    end 
end