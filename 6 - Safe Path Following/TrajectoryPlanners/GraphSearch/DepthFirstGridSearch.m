classdef DepthFirstGridSearch < ForwardGridSearch
    %DepthFirstGridSearch Performs a depth first search on the grid
    
    methods
        function initializeQueue(obj, start_ind)
        %initializeQueue creates a fifo queue and adds the start_ind as the
        %first node
            % Create the queue as a java linked list
            obj.queue = java.util.LinkedList;
            
            % Store the initial starting location
            obj.queue.addLast(start_ind);
        end
        
        function ind = pop(obj)
        %pop Takes the first element off of the list
            try
                ind = obj.queue.removeLast();
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
        
            % Adds the index as the final element to implement a FIFO
            obj.queue.addLast(ind_new);
            
            % Adds the parent mapping
            obj.parent_mapping(ind_new) = ind_parent;
        end
        
        function resolveDuplicate(obj, ind_duplicate, ind_poss_parent, dir_duplicate)
        %resolveDuplicate for the Breadth-first search does nothing
        % Inputs:
        %   ind_duplicate: index that has been seen again
        %   ind_poss_parent: The possible parent that was seen in index
        %   duplicate
        %   dir_duplicate is the direction of the duplicate from the parent
        %   (Move type enumeration)
        
        end
        
        function stop = stoppingCondition(ind, goal_found)
        %stop if goal found
            stop = goal_found;
        end
    end
end

