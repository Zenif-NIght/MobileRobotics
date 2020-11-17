classdef FollowPathG2GOrbitAvoidField < VectorField
    %FollowPathG2GOrbitAvoidField Field used to follow a path while avoiding obstacles
    
    properties (SetAccess = public, GetAccess = public)
        v_max % Max velocity
        
        traj % Instance of InterpolatedTrajectory used for defining
             % the goal position
        
        % Obstacle variables
        q_o_mat % 2 x n_obs matrix where each column is a different obstacle measurement
                % and there are n_obs measurements
        orien % 1 x n_obs vector of orientations of the obstacle points relative to the vehicle
              % (i.e.,
              %     0:      Straight in front of the vehicle
              %     pi/2:   Directly left of the vehicle
              %     -pi/2:  Directly right of the vehicle
              %     +/- pi: Directly behind the vehicle
        n_obs % Number of obstacle points
    end
    
    properties (SetAccess=private)
        % Vector fields
        go_to_goal % Vector field to help go to a goal position
        orbit_avoid % Obstacle avoidance using orbit avoid
        
        % Weights on the vector fields
        w_g2g = 0; % Weight on the go-to-goal vector field
        w_avoid = 0; % Weight on the obstacle avoidance vector field
        
        % Convergence variables
        S = 0 % Sphere of influence
        R = 0 % Radius of max effect
        k_conv = 0 % Convergence gain for orbit
    end
    
    methods
        function obj = FollowPathG2GOrbitAvoidField(x_vec, y_vec, v_max, v_d, traj)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   v_max - Defines the maximum velocity
            %   v_d - Defines the desired velocity
            %   traj - Instance of InterpolatedTrajectory used for defining
            %   the goal position
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.traj = traj;
            obj.v_max = v_max;
            
            % Initialize the obstacle readings to very far away
            obj.q_o_mat = 1000000 .* [1; 1];
            obj.orien = 0;
            
            % Create helper vector fields
            obj.go_to_goal = GoToGoalField(x_vec, y_vec, traj.q_mat(:,1), v_max);
            obj.orbit_avoid = OrbitAvoidField(x_vec, y_vec, obj.q_o_mat, obj.R, v_d, obj.k_conv, obj.S);            
        end
        
        function g = getVector(obj, t, x, ~)
        % getVector will return a go-to-goal vector given the position x
        %
        % Inputs:
        %   obj: class reference used to access class variables (see lines
        %         5-10) 
        %   t: Current time
        %   x: 2D position for calculating the vector
            
            %%%TODO: Implement
            g = [0;0];
        end
        
        function setObstacles(obj, q_o_mat, orien)
        %setObstacles Updates the obstacle position readings
        %
        % Inputs:
        %   q_o_mat: See the class property definition of q_o_mat
        %   orien: See the class property definition of orien
            % Check the size of the inputs
            obj.n_obs = size(q_o_mat,2);
            assert( obj.n_obs == length(orien), 'The orientation inputs must be the same number as obstacles');
            
            % Store the inputs
            obj.q_o_mat = q_o_mat;
            obj.orien = orien;
        end
    end
end

