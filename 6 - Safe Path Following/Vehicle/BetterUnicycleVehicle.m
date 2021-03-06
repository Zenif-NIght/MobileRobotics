classdef BetterUnicycleVehicle < Vehicle
   
    properties
        % Properties for path control (using point control method)
        eps_path = 1.0 % Initial epsilon for controlling a point to a path
        eps_path_min = 0.2 % Minimum value for eps_path
        use_dim_eps = false; % true => use a diminishing epsilon, false will use a constant epsilon of eps_path
        K_point_ctrl % Feedback matrix for point control, used with feedback on 
                     % K_point_ctrl*(q - q_des), where q is the
                     % position and velocity of a point
                     
        % Properties for point-method velocity control
        eps_vel = 0.2; % Epsilon to be used for velocity control
        K_point_vel % Feedback matrix for point control of velocity, used with feedback on 
                     % K_point_vel*(q - q_des), where q is the velocity of a point                     
                     
        % Properties for velocity control 
        K_vel % Feedback matrix for velocity control where the state is the 
              % translational and rotational velocities
              
        % Gains for control
        k_wd = 2; % Gain for the desired rotational velocity
        vd_field_max = 5; % Maximum desired velocity from a vector field
        
        planner;
        u_init;
        v_d;
        w_d;
        
    end
    
    properties(Constant)
        % Additional state variables
        v_ind = 4;
        w_ind = 5;
        
        % Input indices
        u_a_ind = 1; % Translational acceleration input
        u_alpha_ind = 2; % Rotational acceleration input
    end
    
    methods
        function obj = BetterUnicycleVehicle(varargin)
            % Get the initial state
            x0 = [0 0 0 0 0]'; % default to the zero state
            if nargin > 0
                x0 = varargin{1}; 
            end
            
            % Initialize the kinematics and the vehicle
            kin = BetterUnicycle;
            q_ind = [kin.x_ind; kin.y_ind];
            obj = obj@Vehicle(kin, x0, q_ind);  
            
            
            % Calculate feedback matrix for point control
            A = [zeros(2) eye(2); zeros(2,4)]; 
            B = [zeros(2); eye(2)];
            Q = diag([1, 1, 1, 1]);
            R = diag([1, 1]);
            obj.K_point_ctrl = lqr(A, B, Q, R);
            
            % Calculate feedback matrix for velocity point control
            A = zeros(2);
            B = eye(2);
            Q = diag([1, 1]);
            R = diag([.1, .1]);
            obj.K_point_vel = lqr(A, B, Q, R);
            
            % Calculate feedback matrix for velocity control
            A = zeros(2);
            B = eye(2);
            Q = diag([100, 100]);
            R = diag([1, 1]);
            obj.K_vel = lqr(A, B, Q, R);
            
        end
        
        function u = velocityControl(obj, vd, wd, varargin)
            % Get the state
            if nargin > 5
                x = varargin{1};
            else
                x = obj.x;
            end
            
            % Extract states
            v = x(obj.kinematics.v_ind);
            w = x(obj.kinematics.w_ind);
            
            % Calculate new state
            z = [v - vd; w - wd];
            
            % Calculate control
            u = -obj.K_vel * z;
        end
        
        function u = pathControl(obj, t, q_des, qd_des, qdd_des, varargin)
            %pathControl will calculate the desired control to track a path
            %  defined by the inputs:
            %   t: Time
            %   q_des: Desired position
            %   qd_des: Desired velocity vector of a point mass
            %   qdd_des: Desired acceleration vector of a point mass
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %
            % Note that the current time t is used to calcualte the value
            % for epsilon in the epsilon point control
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get the state
            if nargin > 5
                x = varargin{1};
            else
                x = obj.x;
            end
            
            % Calculate epsilon
            eps = obj.getEpsilon(t);
            
            % Get states
            x_pos = x(obj.kinematics.x_ind);
            y_pos = x(obj.kinematics.y_ind);
            [v, w] = obj.kinematics.getVelocities(t, x, 0);
            th = x(obj.kinematics.th_ind);
            c = cos(th);
            s = sin(th);
                
            % Form espilon variables
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e = [c -eps*s; s eps*c];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
            
            % Calculate current values of espilon state
            q_eps = [x_pos; y_pos] + eps * [c; s];
            q_eps_dot = R_e*[v; w];
            q = [q_eps; q_eps_dot];
            
            % Calculate point control
            u_point = -obj.K_point_ctrl*(q - [q_des; qd_des]) + qdd_des;
            
            % Calculate the control inputs
            u = R_e_inv*u_point - w_hat_e*[v; w];            
        end
        
        %%%%%%%%%%%%%%%  Vector Field Following Controls %%%%%%%%%%%%%%%%
        function u = vectorFieldControl(obj, t, g, control_type, varargin)
            %vectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs:
            %   t: Time
            %   g: vector to be followed
            %   varargin{1}: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)            %
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get the state
            if nargin > 3
                x = varargin{1};
            else
                x = obj.x;
            end
            
            if control_type == VECTOR_FOLLOWING_TYPE.VELOCITY
                u = obj.velocityVectorFieldControl(t, g, x);
            elseif control_type == VECTOR_FOLLOWING_TYPE.POINT
                u = obj.pointVelocityVectorFieldControl(t, g, x);
            else
                u = [0; 0];
            end
        end
                
        function u = velocityVectorFieldControl(obj, t, g_function, x)
            %vectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs:
            %   t: Time
            %   g_function: function handle for obtaining vector.
            %   g_function is a function of (t, x) and returns the vector
            %   x: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Calculate the current vector
            g = g_function(t, x(obj.q_ind), x(obj.th_ind));
            
            % Calculate the desired velocity
            vd = norm(g);
            vd = min(vd, obj.vd_field_max); % Threshold the desired velocity
            
            % Calculate the desired orientation
            th_d = atan2(g(2), g(1));
            
            % Calculate the error in orientation
            th = x(obj.th_ind);
            th_e = th-th_d;
            th_e = atan2(sin(th_e), cos(th_e)); % Adjust to ensure between -pi and pi
            
            % Calculate the desired rotational velocity
            wd = -obj.k_wd*th_e;
            
            % Use velocity control to follow the vector field
            u = obj.velocityControl(vd, wd, x);
        end
        
        function u = pointVelocityVectorFieldControl(obj, t, g_function, x)
            %pointVelocityVectorFieldControl will calculate the desired control to follow 
            % a vector field with the following inputs (using point control):
            %   t: Time
            %   g_function: function handle for obtaining vector.
            %   g_function is a function of (t, x) and returns the vector
            %   x: the state
            %
            %  With the output
            %    u: control input to the system (u_v, u_omega)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Get necessary states
            [v, w] = obj.kinematics.getVelocities(t, x, 0);
            th = x(obj.th_ind);
            c = cos(th);
            s = sin(th);
            
            % Form espilon variables
            eps = obj.eps_vel;
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e = [c -eps*s; s eps*c];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];
            
            % Calculate current velocity of espilon state
            q_eps = x(obj.q_ind) + eps*[c; s];
            q_eps_dot = R_e*[v; w];
            
            % Calculate the vector field for the espilon point
            g = g_function(t, q_eps, th);
            
            % Restrict the velocity of the vector field
            v_g = norm(g);
            if v_g > obj.vd_field_max
                g = obj.vd_field_max/v_g * g;
            end
            
            % Calculate point control
            u_point = -obj.K_point_vel*(q_eps_dot - g);
            
            % Calculate the commanded acceleration values            
            u = R_e_inv*u_point - w_hat_e*[v;w];            
        end
        
        
        function [q_er, qdot_er, qddot_er, qdddot_er] = epislonTrajectory(obj, t, traj, eps)
            % Calculate Reference Trajectory
            [q_r, qdot_r, qddot_r, qdddot_r,qddddot_r] = traj(t);

            % Calculate Reference Trajectory Parameters
            psi_r = atan2(qdot_r(2),qdot_r(1));
            v_r = norm(qdot_r);
            w_r = (qdot_r(1)*qddot_r(2) - qdot_r(2)*qddot_r(1))/v_r^2;
            v = [v_r; w_r];
            a_r = (qdot_r(1)*qddot_r(1) + qdot_r(2)*qddot_r(2))/v_r;
            alpha_r = (qdot_r(1)*qdddot_r(2) - qdot_r(2)*qdddot_r(1))/v_r^2 - 2*a_r*w_r/v_r;
            a = [a_r; alpha_r];
            
            

            % Create Algebraic relations to epsilon trajectory
            R_er = [cos(psi_r) -eps*sin(psi_r); sin(psi_r) eps*cos(psi_r)];
            w_hat_r = [0 -eps*w_r; w_r/eps 0];
            
            % Create Epsilon Trajectory
            q_er = q_r + eps*[cos(psi_r);sin(psi_r)];
            qdot_er = R_er*v;
            qddot_er = R_er*w_hat_r*v + R_er*a;

        end

        %%%%%%%%%%%%%%%%%%%%%% Control Functions %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function u = TrackTrajectoryApproximateDiffeomorphism(obj, t, x, traj)
            eps = obj.eps_vel;
            % Get states
            x_pos = x(obj.x_ind);
            y_pos = x(obj.y_ind);
            [v, w] = obj.getVelocities(t, x, 0);
            th = x(obj.th_ind);
            c = cos(th);
            s = sin(th);

            % Form espilon variables
            w_hat_e = [0 -eps*w; w/eps 0];
            R_e = [c -eps*s; s eps*c];
            R_e_inv = [1 0; 0 1/eps] * [c s; -s c];

            % Calculate current values of espilon state
            q_eps = [x_pos; y_pos] + eps * [c; s];
            q_eps_dot = R_e*[v; w];
            q = [q_eps; q_eps_dot];

            % Get epsilon trajectory
            [qd, qd_dot, qd_ddot] = obj.epislonTrajectory(t,traj,eps);

            % Calculate point control
            u_point = -obj.K_point_ctrl*(q - [qd; qd_dot]) + qd_ddot;

            % Calculate the control inputs
            u = R_e_inv*u_point - w_hat_e*[v; w];
        end
        
        function [v, w] = getVelocities(obj, t, x, u)
            v = x(obj.v_ind);
            w = x(obj.w_ind);
        end
    end
    
    methods (Access=protected)
        function eps = getEpsilon(obj, t)
            if obj.use_dim_eps
                eps = obj.eps_path*exp(-t);
                eps = max(obj.eps_path_min, eps);
            else
                eps = obj.eps_path;
            end
        end
    end
end

