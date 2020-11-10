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
            %%% Todo: implement
            g = [0;0];
        end
    end
end

