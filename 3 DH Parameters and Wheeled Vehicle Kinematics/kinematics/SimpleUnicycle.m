classdef SimpleUnicycle < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities

    properties
        v_ind = 1; % First input
        w_ind = 2; % Second input

    end
    methods
        function obj = SimpleUnicycle()
            obj = obj@VehicleKinematics(5);
            obj.plot_path = true;
            obj.c ='r';
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            % Time derivative: [xdot; ydot; theta_dot] = [vcos(theta); vsin(theta); omega]
            v = u(obj.v_ind);
            w = u(obj.w_ind);
                        
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(3,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta}            
        end        
    end 
end

