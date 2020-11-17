classdef MultiMPCGoToGoal < MultiAgentScenario
    %MultiMPCGoToGoal Implements several agents which move towards a
    %desired goal location using MPC for a differentially flat system
    
    properties
        qd_mat % Stores the desired trajectory over time, used for plotting results,
               % The matrix has 2 x n_agents number of rows (2 for each
               % agent)
        goalPoints % an array of desired points for the vehicles  (e.g., [q0,q1,...,qn] 
                  % where qi = [x; y]
        agent_colors % n_agents x 3 matrix where each row represents the color of a different agent
        
        % Plotting flags for results
        plot_agent_traj_results = true; % true => each agent's actual trajectory will be overlayed onto the world plot
        
        % Storage for the desired differentially flat state
        x_diff_flat = {}
    end
    
    methods
        function obj = MultiMPCGoToGoal(world, goalPoints, x0)
            %MultiReferenceAvoidScenario Construct an instance of this class
            %   veh : instance to a vehicle class to be used to create the
            %   agents
            %   world: instance of the polygon world class
            %   goalPoints: an array of desired points for the vehicles (e.g., [q0,q1,...,qn] 
            %              where qi = [x; y]
            %   x0: Cell structure of initial states, one for each agent to
            %   be run (size n_agents)
            
            % Create a unique color for each agent
            n_agents = length(x0);
            agent_colors = distinguishable_colors(n_agents);
            
            % Create each agent and its corresponding plotter
            agents = cell(n_agents, 1);
            plotters = {};
            for i = 1:length(agents)
                % Create the agent
                agents{i} = MPCG2GAgent(x0{i}, goalPoints(i,:)', world);
                
                % Create a vehicle plotter
                plotters{end+1} = SingleAgentPlotter(@(t)agents{i}.vehicle.getConfiguration(t), agent_colors(i,:));
                plotters{end+1} = TrajectoryPlotter(@(t)agents{i}.getLatestTrajectory(), agent_colors(i,:));
                
                % Create a a plotter for the desired position
                plotters{end+1} = PositionPlotter(@(t)agents{i}.getLatestTrajectoryPoint(), agent_colors(i,:));
                
                %plotters{end+1} = TwoDRangePlotter(veh_i);
                
                % Initialize the occupancy grid plotter for agent 1
%                 if i == 1
%                     figure('units','normalized','outerposition',[0 0 1 1]);
%                     plotters{end+1} = OccupancyPlotter(agents{i}.map);
%                     plotters{end}.plot_grid = true;
%                     plotters{end}.initializePlot(0);
%                 end
            end
            
            % Initialize the object
            obj = obj@MultiAgentScenario(agents, world, plotters, true);
            
            % Store object variables
            obj.goalPoints = goalPoints;
            obj.agent_colors = agent_colors;
        end
        
        function initializeWorldPlot(obj, ax)
            initializeWorldPlot@MultiAgentScenario(obj, ax);
            
            % Plot the waypoints for the path
            hold on;
            plot(obj.goalPoints(:,1), obj.goalPoints(:,2), 'ro', 'linewidth', 5);
        end
        
        function plotResults(obj)
            % Plot the executed trajectory for each agent
            if obj.plot_agent_traj_results
                hold on;
                for i = 1:obj.n_agents
                    % Get state indices
                    x_ind = obj.state_ind{i}(1);
                    y_ind = obj.state_ind{i}(2);                    

                    % Plot the actual trajectory
                    plot(obj.xmat(x_ind, :), obj.xmat(y_ind, :), ':', 'linewidth', 2, 'color', obj.agent_colors(i,:));
                end
            end
            
            % Loop through and plot position error for each agent
            n_subplots = 6;
            plot_count = 1;
            lw_des = 3; % Line width for the desired plots
            lw_act = 2; % Line width for the actual plots
            fontsize = 18; % Fontsize for each plot
            for i = 1:obj.n_agents
                % Create a figure for the actual and desired positions
                figure('units', 'normalized', 'outerposition', [0 0 1 1])
                
                % Get the state indices
                x_ind = obj.state_ind{i}(1);
                y_ind = obj.state_ind{i}(2);
                v_ind = obj.state_ind{i}(BetterUnicycleVehicle.v_ind);
                w_ind = obj.state_ind{i}(BetterUnicycleVehicle.w_ind);
                
                
%                 % Plot the trajectory
%                 subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
%                 plot(obj.x_diff_flat{i}(1, :), obj.x_diff_flat{i}(2, :), 'r:', 'linewidth', 3); hold on;
%                 plot(obj.xmat(x_ind, :), obj.xmat(y_ind, :), 'b', 'linewidth', 2); hold on;
%                 axes equal
                
                % Plot the x position
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                plot(obj.tmat, obj.x_diff_flat{i}(1, :), 'r:', 'linewidth', lw_des); hold on;
                plot(obj.tmat, obj.xmat(x_ind, :), 'b', 'linewidth', lw_act);
                ylabel('x');
                set(gca, 'fontsize', fontsize);
                
                % Plot the y position
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                plot(obj.tmat, obj.x_diff_flat{i}(2, :), 'r:', 'linewidth', lw_des); hold on;
                plot(obj.tmat, obj.xmat(y_ind, :), 'b', 'linewidth', lw_act);
                ylabel('y');
                set(gca, 'fontsize', fontsize);
                
                % Plot the error vs time
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                diff = obj.x_diff_flat{i}(1:2, :) - obj.xmat([x_ind, y_ind], :);
                err = vecnorm(diff);
                plot(obj.tmat, err, 'r', 'linewidth', lw_act);
                ylabel('error');
                set(gca, 'fontsize', fontsize);
                
                % Plot the velocities over time
                [vd, wd, kd] = getVelocitiesFromTraj(obj.x_diff_flat{i});
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                plot(obj.tmat, vd, 'r:', 'linewidth', lw_des); hold on;
                plot(obj.tmat, obj.xmat(v_ind,:), 'b', 'linewidth', lw_act);
                ylabel('v');
                set(gca, 'fontsize', fontsize);
                
                % Plot the rotational velocity over time
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                plot(obj.tmat, wd, 'r:', 'linewidth', lw_des); hold on;
                plot(obj.tmat, obj.xmat(w_ind,:), 'b', 'linewidth', lw_act);
                ylabel('\omega');
                set(gca, 'fontsize', fontsize);
                
                % Plot the curvature over time
                subplot(n_subplots, 1, plot_count); plot_count = plot_count + 1;
                plot(obj.tmat, kd, 'r:', 'linewidth', lw_des); hold on;
                plot(obj.tmat, obj.xmat(w_ind,:)./obj.xmat(v_ind,:), 'b', 'linewidth', lw_act);
                ylabel('Curvature');
                set(gca, 'fontsize', fontsize);
                
                % Return the plot count to 1 for the next agent
                xlabel('time (s)'); 
                plot_count = 1;
            end
            
            
        end
        
        function initializeStateDataStorage(obj, steps)
        % initializeStateDataStorage initialization function to create
        % storage values - Default is to do nothing
        %
        % Inputs:
        %   steps: the number of steps in the simulation (corresponds to the same size as tmat 
            obj.x_diff_flat = cell(1, obj.n_agents);
            n_diff_flat_states = length(obj.agents{1}.x_flat_latest); % Number of differentially flat states
            for i = 1:obj.n_agents
                obj.x_diff_flat{i} = zeros(n_diff_flat_states, steps);
            end
        end
        
        function storeNewStateData(obj, t, k, x)
           %storeNewStateData stores the following data at each euler step
           %    * Vehicle state
           %
           % Inputs:
           %    t: time value
           %    k: simulation step
           %    x: state at time t
           
            % Store the state of each agent
            for i = 1:obj.n_agents
                obj.x_diff_flat{i}(:,k) = obj.agents{i}.x_flat_latest;
            end
        end
    end
end

function [v, w, k] = getVelocitiesFromTraj(traj)
    
    % Extract desired data
    qdot = traj(3:4, :);
    qddot = traj(5:6, :);
    
    % Initialize variables
    n_vals = size(traj, 2);
    v = zeros(1, n_vals);
    w = zeros(1, n_vals);
    k = zeros(1, n_vals);
    
    for i = 1:n_vals
        [v(i), w(i), k(i)] = calculateVelocities(qdot(:,i), qddot(:,i));
    end
end

function [v, w, k] = calculateVelocities(qdot, qddot)
%calculateVelocities
%
% Inputs:
%   qdot: \dot{q}, the time derivative of position (vel vector)
%   qddot: \ddot{q}, the second time derivative of position (accel vector)
%
% Outputs
%   v: translational velocity
%   w: rotational velocity
%   kappa: curvature
    %% Calculate using vector notation
    % Initialize variables
    J = [0 -1; 1 0]; %pi/2 rotation matrix

    % Calculate velocities
    v = sqrt(qdot'*qdot);
    w = -(qdot'*J*qddot)/(qdot'*qdot);
    
    % Calculate kurvature
    k = -(qdot'*J*qddot)*(qdot'*qdot)^(-3/2);
end