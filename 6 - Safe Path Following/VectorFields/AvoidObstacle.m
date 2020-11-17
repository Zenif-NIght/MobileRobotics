classdef AvoidObstacle < VectorField
    %AvoidObstacle Basic vector field pointing away from an obstacle point
    
    properties (SetAccess = public, GetAccess = public)
        x_o % Obsactle position
        v_max % Max velocity
        
        % Convergence variables
        S = 5 % Sphere of influence
        R = 1 % Radius of max effect        
    end
    
    methods
        function obj = AvoidObstacle(x_vec, y_vec, x_o, v_max)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_o - Defines a 2D obstacle position for the vector field
            %   v_max - Maximum velocity
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            obj.x_o = x_o;
            obj.v_max = v_max;
        end
        
        function g = getVector(obj, ~, x, ~)
        % getVector: calculate obstacle avoidance vector
        %
        % Inputs:
        %   obj: class reference used to access class variables (see lines
        %         5-10) 
        %   x: 2D position for calculating the vector
            % Calculate difference vector to the obstacle
            g = x - obj.x_o;
            
            % Scale the magnitude of the resulting vector
            dist = norm(g); % Magnitude
            scale = 1; % Default scalie if within R
            if dist > obj.S % Zero influence outside S
                scale = 0;
            elseif dist > obj.R % Linear interpolation between R and S
                scale = (obj.S - dist) / (obj.S - obj.R);
            end
            v_g = obj.v_max * scale; % Resulting magnitude of the velocity
            
            % Output g (careful not to divide by zero)
            if dist > 0 % Avoid dividing by zero
                g = v_g/dist * g; % Dividing by dist is dividing by the norm
            else % Choose a random position if you are on top of the obstacle
                g = rand(2,1);
            end
        end
    end
end

