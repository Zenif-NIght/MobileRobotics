classdef ForwardGridSearch < handle
    %ForwardGridSearch Implements a grid search algorithm performing a
    %forward search
    
    % Properties set in the constructor that cannot be modified
    properties (SetAccess=private)
        grid % Stores the grid being searched over
        grid_size % size of the grid being searched [n_rows, n_cols]
        ind_max % Maximum index in the grid             
    end
    
    % Search specific variables
    properties (SetAccess=protected)
        ind_goal % Index of goal states
        ind_start % Starting index
        n_goal % Number of goal states 
        parent_mapping % Stores a map from each index to its parent node
        visited % Stores whether each element has been visited
    end
    
    properties(Access=protected)
        queue % Storage queue for processing next location
    end
    
    properties(Constant)
        % grid location states
        UNVISITED = 0 % Indicates that a location has not yet been visited
        DEAD = 1 % State has been visited and so has all neighbors
        ALIVE = 2 % State has been encountered, but has unvisited next states   
        
        % Move type enumeration
        L = 1; % Left
        LU = 2; % Diagonal, left and up
        U = 3; % Up
        RU = 4; % Diagonal, right and up
        R = 5; % Right
        RD = 6; % Diagonal, right and down
        D = 7; % Down
        LD = 8; % Diagonal, left and down
    end
    
    methods(Abstract)
        % Initializes the queue with the starting index
        initializeQueue(obj, start_ind);
        
        % Implements a function that gets the next index to be investigated from the queue
        % returns -1 if there is no element on the queue
        ind = pop(obj);
        
        % Adds the index to the queue and store parent
        % Inputs:
        %   ind_new: Index of the new location
        %   ind_parent: Index of the parent node
        %   dir_new: Direction of ind_new from the parent (Move type
        %   enumeration)
        addIndexToQueue(obj, ind_new, ind_parent, dir_new);
        
        % resolves duplicate sighting of the index
        % Inputs:
        %   ind_duplicate: index that has been seen again
        %   ind_poss_parent: The possible parent that was seen in index
        %   duplicate
        %   dir_duplicate is the direction of the duplicate from the parent
        %   (Move type enumeration)
        resolveDuplicate(obj, ind_duplicate, ind_poss_parent, dir_duplicate);
        
        % returns true if stopping condition reached, false otherwise
        stop = stoppingCondition(ind, goal_found);
    end
    
    methods
        function obj = ForwardGridSearch(grid)
            %ForwardGridSearch Construct an instance of this class
            %   Inputs:
            %       grid: n_rows x n_cols grid where 0 corresponds to free
            %       and otherwise the grid is occupied
            
            % Store the input grid and calculate metadata
            obj.grid = grid;
            obj.grid_size = size(obj.grid);
            obj.ind_max = sub2ind(obj.grid_size, obj.grid_size(1), obj.grid_size(2));
        end
        
        function initializeSearch(obj, ind_start, ind_goal)
        %initializeSearch Initializes a search from the start index to the
        %goal index
            % Add the starting index to the queue
            obj.initializeQueue(ind_start);
            obj.ind_start = ind_start;
            
            % Store the goal indices
            obj.ind_goal = ind_goal;
            obj.n_goal = length(obj.ind_goal);
            
            % Create a mapping for parents
            obj.parent_mapping = containers.Map('KeyType', 'uint64', 'ValueType', 'uint64');
            
            % Create a grid for indicating whether a node has been visited
            obj.visited = false(obj.grid_size);
        end
        
        function [plan, success] = getPlan(obj, end_index)
            plan = [end_index];
            index = end_index;
            
            % Loop through and trace the parents up the tree
            while index ~= obj.ind_start
                if obj.parent_mapping.isKey(index)
                    index = obj.parent_mapping(index); % Update the index to the start
                    plan = [index plan]; % Prepend the index
                else
                    warning('Start not found');
                    plan = [];
                    success = false;
                    return;
                end
            end
            success = true;
        end
        
        function ind_end = fullSearch(obj, start_ind, ind_goal)
        %fullSearch Performs a search from the starting index to the goal
        %index.
        %
        % Inputs:
        %   start_ind: Starting index for the search within the grid
        %   ind_goal: vector of possible goal indices
        %
        % Outputs:
        %   ind_end: the ending index of the search, -1 indicates that the
        %   search was not successful
            % Initialize the search
            obj.initializeSearch(start_ind, ind_goal);
        
            % Initialize the end index indicating that search not yet
            % successful
            ind_end = -1;
            
            % Perform search
            ind = start_ind;
            goal_found = false;
            while ~obj.stoppingCondition(ind, goal_found)
                [ind, goal_found] = obj.step();
                
                % If search was successful, then store the final index
                if goal_found
                    ind_end = ind;                    
                end
            end
        end
        
        function [ind, succ] = step(obj)
            %step Creates one step through the while loop
            %
            % Outputs: 
            %   ind: index of the node that was visited
            %        -1 => there are no elements on the list
            %   succ: true => goal reached, false => it was not
            
            % Get the next element to be evaluated
            succ = false; % Initialize success to be not true
            ind = obj.pop();
            if ind < 0
                return;
            end
            
            % Check to see if final goal reached
            succ = ~isempty(find(obj.ind_goal == ind, 1));
            
            % Get the neighbors of ind
            [ind_neighbors, direction] = obj.getNeighboringNodes(ind);
            
            % Loop through all neighbors
            for i = 1:length(ind_neighbors)
                % Extract neighbor information
                ind_i = ind_neighbors(i); % ith neighbor
                dir_i = direction(i); % Direction of ith neighbor
                
                % Resolve duplicate
                if obj.visited(ind_i)
                    obj.resolveDuplicate(ind_i, ind, dir_i);
                    
                else % => not yet visited 
                    % Mark as visited
                    obj.visited(ind_i) = true;
                    
                    % Insert the node
                    obj.addIndexToQueue(ind_i, ind, dir_i);
                end
            end
        end
        
        function [ind_neigh, dir] = getNeighboringNodes(obj, ind)
        %getNeighboringNodes returns the nodes that neighbor the given
        %index
        %
        % Input:
        %   ind: scalar index of the location in question
        %
        % Outputs:
        %   ind_neigh: indices of the neighbors of ind
        %   dir: vector indicating directionality of the neighbors. Each
        %   element is of "Move type enumeration"
        
            % Dermine if a top or a bottom index
            row_ind = mod(ind, obj.grid_size(1)); % 0 corresponds to bottom, 1 corresponds to top
            bottom = false; % Initialize top and bottom to false
            top = false;
            if row_ind == 0
                bottom = true;
            elseif row_ind == 1
                top = true;
            end
            
            % Create the directions vector
            dir = [obj.U, obj.LU, obj.RU, obj.D, obj.LD, obj.RD, obj.L, obj.R];
                
        
            % Initialize the neighbor indices
            ind_neigh = -1 .* ones(1, 8);
            k = 1;
            
            % Add in the neighbor above
            if ~top
                ind_neigh(k) = ind-1; k = k+1; % Up
                ind_neigh(k) = ind-1-obj.grid_size(1); k = k+1;% Left-Up diagonal
                ind_neigh(k) = ind-1+obj.grid_size(1); k = k+1;% Right-Up diagonal
            end
            
            % Add in the neighbor below
            if ~bottom
                ind_neigh(k) = ind+1; k = k+1; % Down
                ind_neigh(k) = ind+1-obj.grid_size(1); k=k+1;% Left-Down diagonal
                ind_neigh(k) = ind+1+obj.grid_size(1); k=k+1;% Right-Down diagonal
            end
            
            % Add in the neighbor to the left
            ind_neigh(k) = ind-obj.grid_size(1); k = k+1;
            
            % Add in the neighbor to the right
            ind_neigh(k) = ind+obj.grid_size(1); k = k+1;            
        
            % Check to make sure they aren't too small or too large 
            ind_valid = ind_neigh > 0 & ind_neigh <= obj.ind_max;
            ind_neigh = ind_neigh(ind_valid);
            dir = dir(ind_valid);
            
            % Check to make sure they are not obstacle locations
            ind_free = obj.grid(ind_neigh) == OccupancyGrid.FREE;
            ind_neigh = ind_neigh(ind_free);
            dir = dir(ind_free);
        end        
    end
end

