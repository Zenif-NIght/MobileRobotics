function testGraphSearch()
%testGraphSearch Creates an occupance grid and searches from a start
%position to an end position
close all

    %% Create an occupancy grid
    % Create the world
    world = PolygonWorld1;
    %world = PolygonWorld_Non_convex;
    
    % Create an occupancy grid
    res = 0.25;
    xlim = [0 22];
    ylim = [-5 10];
    grid = OccupancyGrid(res, xlim, ylim);
    
    % Fill the world with points
    x_vec = xlim(1):res:xlim(2);
    y_vec = ylim(1):res:ylim(2);
    [X, Y] = meshgrid(x_vec, y_vec);
    [n_rows, n_cols] = size(X);
    for row = 1:n_rows
        for col = 1:n_cols
            if world.insideObstacles(X(row, col), Y(row, col))
                grid.setOccupied([X(row, col); Y(row, col)]);
            end
        end
    end
    
    % Plot the world
    figure;
    world.plotWorld(gca);
    
    % Plot the occupancy grid
    figure('Color', 'white', 'units', 'inches', 'Position', [1 1, 8, 5]);
    hold on;
    plotter = OccupancyPlotter(grid);
    plotter.plot_grid = true;
    plotter.initializePlot(0);
    plotter.plot(0);
    ax_occ = plotter.ax_handle;
    
    %% Demonstrate the search
    % Get the starting and ending positions
    q_start = [1; 1];
    plot(ax_occ, q_start(1), q_start(2), 'go', 'linewidth', 3);
    q_goal = [20; 8];
    plot(ax_occ, q_goal(1, :), q_goal(2, :), 'ro', 'linewidth', 3);
    
    
    % Convert the starting and ending positions to grid indices
    ind_start = grid.positionToArrayIndex(q_start);
    ind_goal = grid.positionToArrayIndex(q_goal);
    
    % Create the planner
    %planner = BreadthFirstGridSearch(grid.grid);
    %planner = DepthFirstGridSearch(grid.grid);
    %planner = DijkstraGridSearch(grid.grid);
    %planner = AstarGridSearch(grid.grid);
    planner = GreedySearch(grid.grid);
   
    %% Plot the resulting plan
    % Perform a full search
    ind_end = planner.fullSearch(ind_start, ind_goal);
    
    % Extract the plan indices
    [plan, success] = planner.getPlan(ind_end);%ind);
    assert(success, 'Planner did not successfully complete');
    
    % Loop through and construct the resulting plan
    n_nodes = length(plan);
    q_plan = zeros(2,n_nodes);
    for k = 1:n_nodes
        q_plan(:,k) = grid.arrayIndexToPosition(plan(k));
    end
    
    % Plot the plan
    plot(ax_occ, q_plan(1,:), q_plan(2,:), 'go');
    
    % Calculate the path length
    path_len = 0; 
    for k = 1:(n_nodes-1)
        path_len = path_len + norm(q_plan(:,k) - q_plan(:,k+1));
    end
    disp(['Total path length: ' num2str(path_len)]);
    
    % Calculate a trajectory based on the plan
    vel_des = 0.25;
    resolution = 0.01;
    time_init = 0;
    traj = InterpolatedTrajectory(q_plan, vel_des, resolution, time_init);
    
    % Visualize the trajectory
    t_vec = time_init:.05:(path_len/vel_des);
    h_traj_point = plot(q_plan(1,1), q_plan(2,1), 'og', 'linewidth', 3);
    for k = 1:length(t_vec)
        q = traj.getPoint(t_vec(k));
        set(h_traj_point, 'xdata', q(1), 'ydata', q(2));
        pause(0.01);
    end
end

