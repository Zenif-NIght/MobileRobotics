classdef steeringCar < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    properties
        a_ind = 1; % First input
        alpha_ind = 2; % Second input
        
        % other x vector indeces
        phi_ind = 4;
        w_ind = 5;
        L=1;
        
    end
    methods
        function obj = steeringCar()
            obj = obj@VehicleKinematics(5);
            obj.plot_path = true;
            obj.c = 'g';
            
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            % Time derivative: [xdot; ydot; theta_dot] = [vcos(theta); vsin(theta); omega]
            a = u(obj.a_ind);
%             alpha = u(obj.alpha_ind);
                        
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            s = 1;%x(obj.s_ind);
            w = x(obj.w_ind);
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = cos(theta); % \dot{x}
            xdot(obj.y_ind) = sin(theta); % \dot{y}
            xdot(obj.th_ind) =tan(obj.phi_ind)/obj.L ; % \dot{theta}
            xdot(obj.phi_ind) = w;
            xdot(obj.w_ind) = a;
        end        
    end 
end