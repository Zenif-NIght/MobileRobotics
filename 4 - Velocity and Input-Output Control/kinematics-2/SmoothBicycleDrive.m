classdef SmoothBicycleDrive < VehicleKinematics
    %SimpleUnicycle Implements a unicycle with direct control over the
    %velocities
    
    properties
        ind_a = 4 % velocity of right wheel
        ind_phi = 5 % Rotational angle of left wheel

        rad = 0.25;
        % Vehicle properties
        L = 1

        % Plotting properties
        h_wheel = [];
        h_wheel_line = [];
    end
    
    methods
        function obj = SmoothBicycleDrive()
%             obj = obj@DifferentialDrive();
            obj = obj@VehicleKinematics(5);
            obj.dimensions = 5;
        end
        
        function xdot = kinematics(obj, t, x, u)
            %kinematics Gives the unicycle dynamics
            %   [x; y; theta] = [vcos(theta); vsin(theta); omega]
            %   u = [v; omega]
            
%             % Extract inputs
%             ur = u(1); % Rotational velocity of right wheel
%             ul = u(2); % Rotational velocity of left wheel
%             
%             % Extract wheel angular velocities
%             wr = x(obj.ind_wr);
%             wl = x(obj.ind_wl);
%             
%             % Calculate velocities
%             v = obj.rad/2*(wr+wl); % Translational velocity
%             w = obj.rad/obj.L*(wr-wl); % Rotational velocity

                        % Extract inputs
            a = u(1); % Translational velocity
            phiDot = u(2); % Rotational velocity
            
            % Calculate dynamics
            theta = x(obj.th_ind);  % Orientation
            xdot = zeros(obj.dimensions,1);
            xdot(obj.x_ind) = v * cos(theta); % \dot{x}
            xdot(obj.y_ind) = v * sin(theta); % \dot{y}
            xdot(obj.th_ind) = w; % \dot{theta} 
            xdot(obj.ind_a) = a; % \dot{wr}
            xdot(obj.ind_phi) = phiDot; % \dot{wl}            
        end   
        
        function [v, w] = getVelocities(obj, t, x, u)
            % Extract wheel angular velocities
            wr = x(obj.ind_a);
            wl = x(obj.ind_phiDot);
            
            % Calculate velocities
            v = obj.rad/2*(wr+wl); % Translational velocity
            w = obj.rad/obj.L*(wr-wl); % Rotational velocity
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

