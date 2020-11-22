classdef FollowPathG2GAvoidField < VectorField
    %FollowPathG2GAvoidField Field used to follow a path while avoiding obstacles
    %using a basic repulsive field
    
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
        obs_avoid % Obstacle avoidance vector field
        
        % Weights on the vector fields
        w_g2g = 2; % Weight on the go-to-goal vector field
        w_avoid = 2; % Weight on the obstacle avoidance vector field
        
        % Convergence variables
        S = 2 % Sphere of influence
        R = 0.5 % Radius of max effect 
    end
    
    methods
        function obj = FollowPathG2GAvoidField(x_vec, y_vec, v_max, traj)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   v_max - Defines the maximum velocity
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
            obj.obs_avoid = AvoidObstacle(x_vec, y_vec, obj.q_o_mat, v_max);
            obj.obs_avoid.S = obj.S; % Set the avoidance parameters
            obj.obs_avoid.R = obj.R;
        end
        
        function g = getVector(obj, t, x, th)
        % getVector will return a go-to-goal vector given the position x
        %
        % Inputs:
        %   obj: class reference used to access class variables (see lines
        %         5-10) 
        %   t: Current time
        %   x: 2D position for calculating the vector
            
            %%% TODO: implement
            g = [0;0];
            for k = 1:obj.n_obs
                q_o = obj.q_o_mat(:,k)
                if isinf(sum(q_o))
                    continue;
                end
                
%################# From the Orbit Avoif Field #################
                g = x - obj.obs_avoid.x_o;%go_to_goal.x_g;
            
                % Scale the magnitude of the resulting vector (Sphere of
                % influence)
                dist = norm(g);
                scale_sphere = 1;
                if dist > obj.S
                    scale_sphere = 0;
                elseif dist > obj.R
                    scale_sphere = (obj.S - dist) / (obj.S - obj.R);
                end

                % Scale the magunitude of the resulting vector by the error
                % orientation
                th_g = atan2(-g(2), -g(1)); % -g as we want to know the heading to the obstacle
%                 th = tan(obj.go_to_goal.x_g(2)/obj.go_to_goal.x_g(1));
                th_e = th-th_g;
                th_e = abs(atan2(sin(th_e), cos(th_e))); % Error in orientation from pointing to the obstacle
                scale_orien = 1; % Scaling due to orientation difference
                if th_e > obj.obs_avoid.S
                    scale_orien = 0;
                elseif dist > obj.R
                    scale_orien = (obj.obs_avoid.S - th_e) / (obj.obs_avoid.S - obj.obs_avoid.R);            
                end

                % Create the velocity scaled by the influence factors
                v_g = obj.v_max * scale_sphere * scale_orien;

                % Output g
                if dist > 0 % Avoid dividing by zero
                    g = v_g/dist * g; % Dividing by dist is dividing by the norm
                else % Choose a random position if you are on top of the obstacle
                    g = rand(2,1);
                end

            end 
            
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

