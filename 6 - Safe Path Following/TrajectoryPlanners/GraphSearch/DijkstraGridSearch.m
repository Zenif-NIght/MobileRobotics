classdef DijkstraGridSearch < ForwardGridSearch
    %DijkstraGridSearch Performs a forward Dijkstra search over a grid
    %world
    
    properties(Access=protected)
        c2c % n_rows x n_cols matrix of cost-to-come
        edge_cost % Cost of each edge from an index to its neighbor
    end
    
    methods
        function obj = DijkstraGridSearch(grid)
        %DijkstraGridSearch Construct an instance of this class
        %   Inputs:
        %       grid: n_rows x n_cols grid where 0 corresponds to free
        %       and otherwise the grid is occupied
            % Initialize the class
            obj = obj@ForwardGridSearch(grid);
            
            % Create the cost to come matrix
            obj.c2c = zeros(size(grid));
            
            % Initialize the edge costs
            obj.edge_cost = zeros(1,8);
            obj.edge_cost([obj.L, obj.U, obj.R, obj.D]) = 1; % Distance from one cell to another - straight
            obj.edge_cost([obj.LU, obj.RU, obj.RD, obj.LD]) = 1.41; %sqrt(2); % Distance from one cell to a diagonal cell
        end
        
        function initializeQueue(obj, start_ind)
        %initializeQueue creates a fifo queue and adds the start_ind as the
        %first node
            % Create the queue as a java linked list
            obj.queue = PriorityQueue(2);
            
            % Store the initial starting location
            obj.queue.insert([start_ind 0]); % Zero stands for zero-cost-to-come
        end
        
        function ind = pop(obj)
        %pop Takes the first element off of the list
            try
                ind_cost = obj.queue.remove(); % index and cost vector
                ind = ind_cost(1);
            catch
                ind = -1;
            end
        end
        
        function addIndexToQueue(obj, ind_new, ind_parent, dir_new)
        %addIndexToQueue Adds the index to the queue and stores the index
        %of the parent in the parent mapping
        % Inputs:
        %   ind_new: Index of the new location
        %   ind_parent: Index of the parent node
        %   dir_new: Direction of ind_new from the parent (Move type
        %   enumeration)
        
            % Calculate and store cost-to-come to new node
            c = obj.edge_cost(dir_new) + obj.c2c(ind_parent);
            obj.c2c(ind_new) = c;
            
            % Adds the index as the final element to implement a FIFO
            tc = obj.getTotalCost(ind_new, c);
            obj.queue.insert([ind_new, tc]);
            
            % Adds the parent mapping
            obj.parent_mapping(ind_new) = ind_parent;
        end
        
        function resolveDuplicate(obj, ind_duplicate, ind_poss_parent, dir_duplicate)
        %resolveDuplicate for the Dijkstra search checks to see if the new
        %path the the node has a lower cost-to-come than the previous path
        %to the node
        % Inputs:
        %   ind_duplicate: index that has been seen again
        %   ind_poss_parent: The possible parent that was seen in index
        %   duplicate
        %   dir_duplicate is the direction of the duplicate from the parent
        %   (Move type enumeration)
            % Calculate the possible cost-to-come from the new potential parent
            c_poss = obj.edge_cost(dir_duplicate) + obj.c2c(ind_poss_parent);
            
            % Compare the cost-to-come with the previous cost-to-come
            if c_poss < obj.c2c(ind_duplicate)
                % Remove the previous entry in the queue
                obj.queue.remove([ind_duplicate, obj.c2c(ind_duplicate)]);
                
                % Store the new cost to come
                obj.addIndexToQueue(ind_duplicate, ind_poss_parent, dir_duplicate);
            end
        end
        
        function stop = stoppingCondition(obj, ind, goal_found)
        %stop if goal found
            stop = goal_found;
        end
        
        function tc = getTotalCost(obj, ind, c)
        % getTotalCost returns the total cost at the given index
        %
        % Default is just to return the cost to come
        %
        % Inputs:
        %   ind: index of the grid point in question
        %   c: cost-to-come
            tc = c;
        end
    end
end

