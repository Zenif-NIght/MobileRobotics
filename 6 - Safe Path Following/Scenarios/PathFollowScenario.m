classdef PathFollowScenario < VectorFieldScenario
    %PathFollowScenario Uses a vector field approach to follow a desired
    %path
    
    properties
        % Sensor
        n_sensors; % Stores the number of sensors
        
        % Plotting properties
        h_goal_point % Handle to the current goal point
        h_plan % Handle to the current plan
    end
    
    methods
        function obj = PathFollowScenario(field, veh, world, control_type)
        %PathFollowScenario constructor
        %
        % Inputs:
        %   field: Instance of the vector field. Must have a setObstacles()
        %          which takes in a vector of obstacle positions and
        %          orientation from the vehicle
        %   veh: Instance of the Vehicle class
        %   world: Instance of the Polygon world class
        %   control_type: The type of vector field control to be used - 
        %                 instance of the VECTOR_FOLLOWING_TYPE
        
            
            % Create the scenario
            obj = obj@VectorFieldScenario(field, veh, world, control_type);
            
            % Store object variables
            obj.n_sensors = veh.sensor.n_lines;
        end
        
        function u = control(obj, t, x)
            % Get obstacle avoidance readings into the vector fields
            if ~isempty(obj.vehicle.xo_latest) && ~isempty(obj.vehicle.yo_latest)
                % Create a matrix of detected positions
                q_obs = [obj.vehicle.xo_latest'; obj.vehicle.yo_latest'];
                
                % Store the detected positions within the vector field
                obj.vector_field.setObstacles(q_obs, obj.vehicle.sensor.orien_nom);
            else
                warning('No sensor readings yet received');
            end
            
            % Get the control
            u = control@VectorFieldScenario(obj, t, x);
        end
        
        function initializeStatePlot(obj)
           initializeStatePlot@VectorFieldScenario(obj);
           
           % Plot the desired plan
           q_plan = obj.vector_field.traj.q_mat;
           obj.h_plan = plot(q_plan(1,:), q_plan(2,:), 'g', 'linewidth', 3);
           
           % Plot the goal
           obj.h_goal_point = plot(q_plan(1,1), q_plan(1,2), 'bo', 'linewidth', 3);           
        end
    end
    
    methods (Access=protected)
        function plotWorld(obj, t)
            % Update the vector field
            obj.vector_field.plotVectorField(t);
            
            % Update the goal location
            q = obj.vector_field.traj.getPoint(t);
            set(obj.h_goal_point, 'xdata', q(1), 'ydata', q(2));
        end
    end
end

