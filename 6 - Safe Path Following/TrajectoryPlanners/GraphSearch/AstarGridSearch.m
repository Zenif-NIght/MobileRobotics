classdef AstarGridSearch < DijkstraGridSearch
    %AstarGridSearch Creates a grid search using the A-star algorithm
    
    methods
        function initializeSearch(obj, start_ind, ind_goal)
            initializeSearch@DijkstraGridSearch(obj, start_ind, ind_goal);
            assert(obj.n_goal == 1, "Only implemented for a single goal location");
        end
        
        function tc = getTotalCost(obj, ind, c)
        % getTotalCost returns the total cost at the given index
        %
        % Total cost is calculates as the cost-to-come + a hueristic of the
        % cost-to-go
        %
        % Inputs:
        %   ind: index of the grid point in question
        %   c: cost-to-come
            % Convert the array index to row and columns
            [r_i, c_i] = ind2sub(obj.grid_size, ind);
            [r_g, c_g] = ind2sub(obj.grid_size, obj.ind_goal);
        
            % Calculate the distance between locations
            diff = [r_i - r_g; c_i - c_g];
            c2g = norm(diff);
            
            % Output the total cost
            tc = c + c2g;
        end
    end
end

