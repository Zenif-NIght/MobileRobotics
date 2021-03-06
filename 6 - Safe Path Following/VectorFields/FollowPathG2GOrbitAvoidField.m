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
        w_g2g = 2; % Weight on the go-to-goal vector field
        w_avoid = 2; % Weight on the obstacle avoidance vector field
        
        % Convergence variables
        S = 2 % Sphere of influence
        R = 0.5 % Radius of max effect
        k_conv = 0.8 % Convergence gain for orbit
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
            
%################# From the Orbit Avoif Field #################

            % Calculate the vector pointing from the vehicle to the obstacle
            o_m_x = obj.obs_avoid.x_o - x;

            % Calculate the orientation difference between vehicle's
            % orientation and the vector pointing to the obstacle
            th_o = atan2(o_m_x(2), o_m_x(1)); % heading to the obstacle
            th_e = th_o - th; % Difference in orientation
            th_e = atan2(sin(th_e), cos(th_e)); 

            % Calculate the rotation based on the angle to the obstacle
            w_act = abs(obj.w);
            if th_e < 0
                w_act = -w_act;
            end

            % Get the effective radius of the vecicle from the center
            xhat = x - obj.obs_avoid.x_o; % Note that this is just -o_m_x
            r_eff_sq = xhat'*xhat;

            % Calculate convergence gain
            if r_eff_sq > obj.rad^2
                gam = 0; % Don't attract vehicle to the orbit
            else
                gam = obj.k_conv*(obj.rad^2 - r_eff_sq);            
            end

            % Create the orbit vector
            A = [gam, -w_act; w_act, gam];
            g = A*xhat;   

            % Scale the vector field base on the sphere of influence
            dist = norm(xhat);
            scale_sphere = 1;
            if dist > obj.S
                scale_sphere = 0;
            elseif dist > obj.rad
                scale_sphere = (obj.obs_avoid.S - dist) / (obj.obs_avoid.S - obj.rad);
            end

            % Scale the vector field by the orientation influence
            scale_orien = 1;
            if abs(th_e) > pi/2
                scale_orien = 0;
            end

            % Calculate the vector scaled by the sphere of influence and
            % the orientation scale
            g = g * scale_sphere*scale_orien;            

            % Threshold the vector field to be less than or equal to the
            % desired velocity
            v_g = norm(g);
            if v_g > obj.v_d
               g = obj.v_d/v_g * g; 
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

