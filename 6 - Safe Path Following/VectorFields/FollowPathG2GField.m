classdef FollowPathG2GField < VectorField
    %FollowPathField Field used to follow a path without avoiding obstacles
    
    properties (SetAccess = public, GetAccess = public)
        traj % Instance of InterpolatedTrajectory used for defining
             % the goal position
    end
    
    properties (SetAccess=private)
        % Vector fields
        go_to_goal % Vector field to help go to a goal position
    end
    
    methods
        function obj = FollowPathG2GField(x_vec, y_vec, v_max, traj)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   v_max - Defines the maximum velocity
            %   traj - Instance of InterpolatedTrajectory used for defining
            %   the goal position
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.traj = traj;
            
            % Create helper vector fields
            obj.go_to_goal = GoToGoalField(x_vec, y_vec, traj.q_mat(:,1), v_max);
        end
        
        function g = getVector(obj, t, x, ~)
        % getVector will return a go-to-goal vector given the position x
        %
        % Inputs:
        %   obj: class reference used to access class variables (see lines
        %         5-10) 
        %   t: Current time
        %   x: 2D position for calculating the vector
            
            % Get the current goal position
            x_g = obj.traj.getPoint(t);
            
            % Calculate the difference vector from vehicle position to goal
            g = obj.go_to_goal.x_g - x;
            
            % Scale the magnitude of the resulting vector using a smooth
            % convergence
            dist = norm(g);
            v_g = obj.go_to_goal.v_max * (1- exp(-dist^2/obj.go_to_goal.sig_sq));
            
            % Check distance prior to dividing by zero
            if dist > 0 % Avoid dividing by zero
                g = v_g/dist * g; % Dividing by dist is dividing by the norm
            else
                g = [0;0];
            end
            obj.go_to_goal.x_g = x_g;
        end
        
        function setObstacles(obj, ~, ~)
        %setObstacles Does nothing
        
            % This is left blank as you are not using obstacles with this
            % field
        end
    end
end

