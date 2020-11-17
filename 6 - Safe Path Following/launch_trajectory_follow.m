function launch_trajectory_follow()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies
    addpath Dependencies/intersections
    addpath Dependencies/PriorityQueue
    addpath Map
    addpath Scenarios
    addpath Sensors
    addpath TrajectoryPlanners
    addpath TrajectoryPlanners/GraphSearch
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle/Kinematics
    addpath World
    
    %%%%%%%%%%%%%%%%% Create a Trajectory %%%%%%%%%%%%%%%%%%%%%
    % Create the world
    world = PolygonWorld1;
    %world = PolygonWorld_Non_convex;
    
    % Create an occupancy grid for planning
    grid = createOccupancyGridFromWorld(world);
    
    % Create a trajectory
    x0 = [0;1;0;0;0]; % Initial state of the vehicle
    t0 = 0;                     % Initial time
    time_res = 0.01;            % Time resolution of the trajectory
    vd = 0.25;                  % Desired velocity
    q_start = x0(1:2);          % Starting position
    q_goal = [20; 8];           % Goal position
    traj = createDesiredTrajectory(q_start, q_goal, grid, t0, time_res, vd);
    
    %%%%%%%%%%%%%%%%% Simple Vector Field Problems %%%%%%%%%%%%%%%%%%%%%
    % Initialize the plotting variables (a meshgrid is used with x_vec and
    % y_vec for originating the arrows)
    x_vec = -2:1:20; % x values for plotting arrows in the vector field
    y_vec = -4:1:10; % y values for plotting arrows in the vector field
    
    % Create a vector field
    v_max = 1.0; % Maximum allowable velocity
%     field = FollowPathG2GField(x_vec, y_vec, v_max, traj);
    field = FollowPathG2GAvoidField(x_vec, y_vec, v_max, traj);
    %field = FollowPathG2GOrbitAvoidField(x_vec, y_vec, v_max, vd, traj);
    
    % Vector field scenario
    scenario = PathFollowScenario(field, BetterUnicycleVehicle(x0), world, VECTOR_FOLLOWING_TYPE.POINT );
    
    % Plot the trajectory to be followed
    %plot(traj.q_mat(1,:), traj.q_mat(2,:), 'go', 'linewidth', 2);
    
    % Run the scenario
    scenario.runScenario();
end

function grid = createOccupancyGridFromWorld(world)
%createOccupancyGridFromWorld Creates an output of type OccupancyGrid using
%the world object
%
% Inputs:
%   world: Instance of the PolygonWorld object
%
% Outputs:
%   grid: Instance of the OccupancyGrid object

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
end

function traj = createDesiredTrajectory(q_start, q_goal, grid, t0, time_res, vd)
%createDesiredTrajectory creates an instance of the InterpolatedTrajectory
%
% Inputs:
%   q_start: 2x1 starting position
%   q_goal: 2x1 ending position
%   grid: Instance of the OccupancyGrid class
%   t0: Initial time of trajectory
%   time_res: Time resolution of the trajectory
%   vd: Desired velocity along the path

    %% Create a plan from the start position to the end position
    % Convert the starting and ending positions to grid indices
    ind_start = grid.positionToArrayIndex(q_start);
    ind_goal = grid.positionToArrayIndex(q_goal);
    
    % Create a planner (Note, you could use any of the planners)
    %planner = BreadthFirstGridSearch(grid.grid);
    %planner = DepthFirstGridSearch(grid.grid);
    %planner = DijkstraGridSearch(grid.grid);
    planner = AstarGridSearch(grid.grid);
    %planner = GreedySearch(grid.grid);
    
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
    
    %% Calculate a trajectory based upon the plan
    traj = InterpolatedTrajectory(q_plan, vd, time_res, t0);
end


