classdef SmoothBicycleDrive < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        v_ind = 4; % velocity of bicycle
        w_ind = 5; % angular velocity of bicycle (just to store)
        phi_ind = 6; % steering angle of bicycle
        % Vehicle properties
        rad = 0.25;
        L = 1;

        % Plotting properties
        h_wheel = [];
        h_wheel_line = [];
    end
    
    methods
        function obj = SmoothBicycleDrive()
%             obj = obj@DifferentialDrive();
            obj = obj@VehicleKinematics(6);
            obj.dimensions = 6;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the bicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; phi]
            
            % Extract bicycle states
            v = x(obj.v_ind);
            w = x(obj.w_ind);
            phi = x(obj.phi_ind);
            
            % Extract inputs
            a = u(1); % Translational acceleration
            phiDot = u(2); % Steering angle Rotational velocity
            
            % Calculate dynamics
            alpha = a*(sec(phi)^2)/obj.L;
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta} 
            xdot(obj.v_ind) = a; % \dot{v}\
            xdot(obj.w_ind) = alpha;
            xdot(obj.phi_ind) = phiDot; % \dot{phi} 
        end   
        
        function [v, w] = getVelocities(obj, t, x, u)
            % Calculate velocities
            v = x(obj.v_ind); % Translational velocity
            w = x(obj.w_ind); % Rotational velocity
        end
        
        function plotState(obj, t, x)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % This function plots the state of the robot.  Uses
            % a triangular plot to plot position and orientation
            %
            % In addition to the standard vehicle plot, this plots
            % the location of the front wheel
            %
            % t: time of plot
            % x: state of the vehicle            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Plot the vehicle
            plotState@VehicleKinematics(obj, t, x);
            
            % Calculate the front wheel location
            q = [x(obj.x_ind); x(obj.y_ind)];
            v = [cos(x(obj.th_ind)); sin(x(obj.th_ind))];
            q2 = q+v;
            
            % Plot the front wheel location
            if isempty(obj.h_wheel)
               % Plot a line to the front wheel
               obj.h_wheel_line = plot([q(1) q2(1)], [q(2) q2(2)], obj.c, 'linewidth', 2);
               
               % Plot the position of the front wheel
               obj.h_wheel = plot(q2(1), q2(2), ['o' obj.c], 'linewidth', 2);
            else
                % Update a line to the front wheel
                set(obj.h_wheel_line, 'xdata', [q(1) q2(1)], 'ydata', [q(2) q2(2)]);
                
                % Update the position of the front wheel
                set(obj.h_wheel, 'xdata', q2(1), 'ydata', q2(2));                
            end
            
        end
    end 
end

