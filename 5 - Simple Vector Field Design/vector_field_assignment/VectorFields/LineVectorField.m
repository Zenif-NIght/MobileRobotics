classdef LineVectorField < VectorField
    %LineVectorField Vector field causing vehicle to move to a line
    
    properties (SetAccess = protected, GetAccess = public)
        % Line variables
        x_l % A point on the line
        psi_l % The orientation of the line
        R_lc % The rotation matrix to rotate a cartesian point to line frame
        R_cl % The rotation matrix to rotate a line point to the cartesian frame
    end
    
    properties(Access = public)        
        % Vector field variables
        slope % Slope of the sigmoid function defining the line
        v_d % Desired velocity / length of the vector field line        
    end
    
    methods
        function obj = LineVectorField(x_vec, y_vec, x_l, psi_l, slope, v_d)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_l - Defines a 2D point along the line
            %   psi_l - Defines the orientation of the line wrt the x-axis
            %   slope - Defines the slope of the sigmoid function
            %   v_d - Desired velocity
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Store the convergence variables
            obj.slope = slope;
            obj.v_d = v_d;
            
            % Set the line variables
            obj.setLineParameters(x_l, psi_l);            
        end
        
        function setLineParameters(obj, x_l, psi_l)
        %setLineParameters Updates the point along the line and the
        %orientation of the line
            % Store variables
            obj.x_l = x_l;
            obj.psi_l = psi_l;            
            
            %%% TODO: create the correct rotation matrices (should not
            %%% simply be the identity matrices)
            obj.R_cl = eye(2); % Rotation from line frame to cartesian frame
            obj.R_lc = eye(2); % Rotation from cartesian frame to line frame
        end
        
        function g = getVector(obj, ~, x, ~)
        % getVector will return a go-to-goal vector given the position x
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