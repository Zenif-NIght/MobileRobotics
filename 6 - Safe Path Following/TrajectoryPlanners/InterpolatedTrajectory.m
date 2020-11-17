classdef InterpolatedTrajectory < handle
    %InterpolatedTrajectory Creates a trajectory interpolation that can be
    %used for path following
    
    properties (SetAccess=private)
        q_mat % The trajectory data
        t_mat % The time to reach each point
        resolution % The resolution of the time vector (i.e. spacing between time elements)
        num_points % Number of points in the trajectory
    end
    
    methods
        function obj = InterpolatedTrajectory(path, vel_des, resolution, time_init)
            %InterpolatedTrajectory Creates a trajectory to be followed
            %
            % Inputs:
            %   path: 2xN matrix where each column is a point along the
            %         path
            %   vel_des: The desired velocity to travel along the path
            %   resolution: The time resolution for creating an
            %               interpolating path
            %   time_init: The time to be at the initial position
            
            % Process input path variables
            [d, N] = size(path); 
            assert(d == 2, 'Path must be two dimensional');
            assert(N > 1, 'Path must consist of at least two elements');
            assert(vel_des > 0, 'Velocity must be greater than zero');
            assert(resolution > 0, 'Resolution must be greater than zero');
            obj.resolution = resolution;
            
            % Initialize variables to define the trajectory
            t_path = zeros(1, N); % Time vector along the path
            t_path(1) = time_init;
            
            % Loop through and create the desired time at each point in the
            % path
            for k = 2:N
                % Calculate the distance between the kth and k-1 point
                q_km1 = path(:,k-1);
                q_k = path(:,k);
                dist = norm(q_k-q_km1); % Distance
                
                % Calculate the travel time between points (distance = vel*del_time)
                del_time = dist/vel_des;
                
                % Calculate the time to arrive at the kth point
                t_path(k) = t_path(k-1) + del_time;                
            end
            
            % Create an evenly spaced trajectory by interpolating between
            % the first and final points
            obj.t_mat = time_init:resolution:t_path(end); % Time indices of the trajectory
            x_vec = interp1(t_path, path(1,:), obj.t_mat); % x-values along the trajectory
            y_vec = interp1(t_path, path(2,:), obj.t_mat); % y-values along the trajectory
            obj.q_mat = [x_vec; y_vec]; 
            obj.num_points = length(obj.t_mat);
        end
        
        function q = getPoint(obj,t)
        %getPoint Will return the point for a given time, t. If t is
        %outside the bounds of the trajectory then the terminal point will
        %be returned
            % Convert time index to matrix index (note, no interpolation is
            % performed) (1 is the lowest index found)
            ind = uint64((t - obj.t_mat(1)) / obj.resolution) + 1; % "+1" for 1 indexing
            
            % Check violation of last time
            if ind > obj.num_points
                q = obj.q_mat(:, end);            
            % Return the trajectory position
            else                
                q = obj.q_mat(:,ind);
            end
        end
    end
end

