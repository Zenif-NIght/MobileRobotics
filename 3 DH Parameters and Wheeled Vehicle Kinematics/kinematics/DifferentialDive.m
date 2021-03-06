classdef DifferentialDive < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    properties
        vl_ind = 1; % First input
        vr_ind = 2; % Second input
        ro = 1; % radesus of the well
        L = 1;

        
    end
    methods
        function obj = DifferentialDive()
            obj = obj@VehicleKinematics(3);
            obj.plot_path = true;
            obj.c = 'magenta';
            
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            % Time derivative: [xdot; ydot; theta_dot] = [vcos(theta); vsin(theta); omega]
            vl = u(obj.vl_ind);
            vr = u(obj.vr_ind);
                        
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(3,1);
            xdot(obj.x_ind) = (vr+vl)* 0.5 * cos(theta); % \dot{x}
            xdot(obj.y_ind) = (vr+vl)* 0.5 * sin(theta); % \dot{y}
            xdot(obj.th_ind) = (vr/obj.L) - (vl/obj.L); % \dot{theta}            
        end        
    end 
end