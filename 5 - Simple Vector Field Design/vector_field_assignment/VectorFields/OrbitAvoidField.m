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

        
            % Calculate the difference between the vector and obstacle
            xhat = (x - obj.x_o);
            
            % Calculate the orientation difference from direction towards
            % obstacle
            th_e =  th - atan2(xhat(1),xhat(2))  ;
            
            % Calculate the rotation based on the angle to the obstacle
            w_act = abs(obj.w);
            if th_e > 0 % th_e is the difference between the vehicle orientation and the obstacle
                w_act = -w_act;
            end
            
            % Get the effective radius of the vecicle from the center
            rad_e = obj.v_d/w_act;
            
            % Calculate convergence gain
                % Don't attract vehicle to the orbit
             gam = obj.k_conv*(rad_e^2 - (xhat')*xhat);
                        
            % Create the orbit vector
            A = [gam, w_act; -w_act, gam];
            g = A*xhat;   
            
            % Scale the vector field base on the sphere of influence
            d = norm(g); % TODO might need to look in to this 
            k_i = 0;
            if d > obj.S
                k_i = 0;
            elseif rad_e < d && d <= obj.S
                k_i = (obj.S - d)/(obj.S - rad_e);
            elseif d<= rad_e
                k_i = 0;
            else
                k_i ="k_i ERROR";
            end
            g = g * k_i;
            
            % Scale the vector field by the orientation influence
            k_o =0;
            if abs(th) <= pi/2
                k_o = 1;
            else
                k_o = 0;
            end
            g = g * k_o;
            
            % Calculate the vector scaled by the sphere of influence and
            % the orientation scale 
            
            % Threshold the vector field to be less than or equal to the
            % desired velocity
            
            
            
        end
        
        function h = plotVectorField(obj, t)
            % Plot the circular radius
            circle(obj.x_o, obj.rad, 20, 'b', []);
            
            % Create the plot of the actual vector field
            h = plotVectorField@VectorField(obj, t);
        end
    end
end

