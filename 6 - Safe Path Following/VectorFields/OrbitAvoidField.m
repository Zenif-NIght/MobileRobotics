classdef OrbitAvoidField < VectorField
    %OrbitAvoidField Creates a vector field moving the vehicle around an
    %obstacle position using an orbit
    
    properties
        x_o % 2D center position of the orbit (typically updated to use the obstacle position as the center of the orbit)
        rad % radius of the orbit
        w % frequency of the orbit (calculated using v = wr relationship)
        k_conv % gain on convergence to the orbit
        v_d % Desired speed of the orbit 
        
        S % Sphere of influence
    end
    
    methods
        function obj = OrbitAvoidField(x_vec, y_vec, x_o, rad, v_d, k_conv, S)
            %Inputs:
            %   x_vec & y_vec - Used for plotting. A grid of quivers will
            %     be plotted using the ranges specified in x_vec and y_vec
            %   x_o - Defines a 2D center position for the vector field
            %   rad - radius of the orbit
            %   v_d - Desired speed of the orbit 
            %   k_conv - gain on convergence to the orbit
            %   S - Sphere of influence
            
            % Create the object variable
            obj = obj@VectorField(x_vec, y_vec);
            
            % Store the variables which describe the orbit
            obj.x_o = x_o;
            obj.rad = rad;
            obj.k_conv = k_conv;
            obj.S = S;
            
            % Calculate the rotational velocity
            obj.v_d = abs(v_d);
            obj.w = obj.v_d/obj.rad;
        end
        
        function g = getVector(obj, ~, x, th)
        % getVector will return a go-to-goal vector given the position x
        %
        % Inputs:
        %   obj: class reference used to access class variables (see lines
        %         5-12) 
        %   x: 2D position for calculating the vector
        %   th: Current orientation of the vehicle
        
            % Calculate the vector pointing from the vehicle to the obstacle
            o_m_x = obj.x_o - x;
            
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
            xhat = x - obj.x_o; % Note that this is just -o_m_x
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
                scale_sphere = (obj.S - dist) / (obj.S - obj.rad);
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
        
        function h = plotVectorField(obj, t)
            % Plot the circular radius
            circle(obj.x_o, obj.rad, 20, 'b', []);
            
            % Create the plot of the actual vector field
            h = plotVectorField@VectorField(obj, t);
        end
    end
end

