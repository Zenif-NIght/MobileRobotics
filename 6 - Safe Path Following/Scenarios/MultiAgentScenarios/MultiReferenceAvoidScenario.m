classdef MultiReferenceAvoidScenario < MultiAgentScenario
    %MultiReferenceAvoidScenario Implements several agents which track a
    %reference as individual agents
    
    properties
        qd_mat % Stores the desired trajectory over time, used for plotting results,
               % The matrix has 2 x n_agents number of rows (2 for each
               % agent)
        waypoints % an array of path points in the desired order (e.g., [q0,q1,...,qn] 
                  % where qi = [x; y]
        traj_follow = {} % A cell structure containing the desired trajectory for each follower
        traj_eps = {} % A cell structure containing the desired trajectory for the epsilon point of each follower
        agent_colors % n_agents x 3 matrix where each row represents the color of a different agent
        state_value = {} % Value of the state of each agent over time
        
        % Plotting flags for results
        plot_agent_traj_results = false; % true => each agent's actual trajectory will be overlayed onto the world plot
    end
    
    methods
        function obj = MultiReferenceAvoidScenario(veh, world, waypoints, x0, Q)
            %MultiReferenceAvoidScenario Construct an instance of this class
            %   veh : instance to a vehicle class to be used to create the
            %   agents
            %   world: instance of the polygon world class
            %   waypoints: an array of path points in the desired order (e.g., [q0,q1,...,qn] 
            %              where qi = [x; y]
            %   x0: Cell structure of initial states, one for each agent to
            %   be run (size n_agents)
            %   Q: Offsets from virtual leader of all the agents
            %      2 x n_agents where each column is the desired offset
            
            
            % Define desired trajectory pamameters
            dt = 0.01;
            vd = 1;
            k_max = 0.5; % Maximum curvature
            sig_max = 0.5; % Maximum change in curvature
            
            % Create the virtual leader trajectory
            tic
            vl_traj = TrajUtil.createClothoidTrajectory(waypoints, vd, dt, k_max, sig_max);
            time_vl_traj = toc
            
            % Create a unique color for each agent
            n_agents = length(x0);
            agent_colors = distinguishable_colors(n_agents);
            
            % Create each agent and its corresponding plotter
            agents = cell(n_agents, 1);
            plotters = {};
            for i = 1:length(agents)
                % Create the agent
                veh_i = veh(x0{i}); veh_i.use_dim_eps = false;
                traj_follow{i} = TrajUtil.createOffsetTrajectory(vl_traj, Q(:,i)); % Desired trajectory for follower
                traj_eps{i} = TrajUtil.createOffsetTrajectory(traj_follow{i}, [veh_i.eps_path; 0]); % Desired trajectory for the epsilon point of the follower
                agents{i} = ReferenceLineAvoidAgent(veh_i, world, traj_follow{i}, traj_eps{i});
                %agents{i} = ReferencePureAvoidAgent(veh_i, world, traj_follow{i}, traj_eps{i});
                
                % Create a vehicle plotter
                plotters{end+1} = SingleAgentPlotter(@(t)veh_i.getConfiguration(t), agent_colors(i,:));
                
                % Createa a plotter for the desired position
                %plotters{end+1} = PositionPlotter(@(t)agents{i}.ReferenceTraj(t));
                plotters{end+1} = PositionPlotter(@(t)traj_follow{i}.reference_traj(t), agent_colors(i,:));
                %plotters{end+1} = PositionPlotter(@(t)traj_eps{i}.reference_traj(t), agent_colors(i,:));
                %plotters{end+1} = TwoDRangePlotter(veh_i);
                
                % Initialize the occupancy grid plotter for agent 1
                if i == 1
                    figure('units','normalized','outerposition',[0 0 1 1]);
                    plotters{end+1} = OccupancyPlotter(agents{i}.map);
                    plotters{end}.plot_grid = true;
                    plotters{end}.initializePlot(0);
                end
            end
            
            % Initialize the object
            obj = obj@MultiAgentScenario(agents, world, plotters, true);
            
            % Store object variables
            obj.waypoints = waypoints;
            obj.traj_follow = traj_follow;
            obj.traj_eps = traj_eps;
            obj.agent_colors = agent_colors;
        end
        
        function initializeWorldPlot(obj, ax)
            initializeWorldPlot@MultiAgentScenario(obj, ax);
            
            % Plot the waypoints for the path
            hold on;
            plot(obj.waypoints(:,1), obj.waypoints(:,2), 'ro', 'linewidth', 5);
        end
        
        function initializePlots(obj)
            % Plot normal plots
            initializePlots@MultiAgentScenario(obj);
            
            % Get indices for plotting the epsilon trajectory
            len = length(obj.tmat);
            ind = zeros(1,len);
            for k = 1:len
                ind(k) = obj.traj_eps{1}.getIndex(obj.tmat(k));
            end
            
            % Plot the reference trajectory for each agent
            hold on;
            obj.qd_mat = zeros(2*obj.n_agents, len);
            ind_q_i = 1:2; % Iteratively stores the position indices for each agent
            for i = 1:obj.n_agents
                % Plot the epsilon trajectory
                %plot(obj.traj_eps{i}.x(ind), obj.traj_eps{i}.y(ind), ':', 'linewidth', 1, 'color', obj.agent_colors(i,:));
                
                % Plot the actual trajectory
                if i == 1
                    plot(obj.traj_follow{i}.x(ind), obj.traj_follow{i}.y(ind), 'linewidth', 2, 'color', obj.agent_colors(i,:));
                else
                    plot(obj.traj_follow{i}.x(ind), obj.traj_follow{i}.y(ind), ':', 'linewidth', 1, 'color', obj.agent_colors(i,:));
                end
                
                % Store the desired states
                obj.qd_mat(ind_q_i, :) = [obj.traj_follow{i}.x(ind); ...
                                          obj.traj_follow{i}.y(ind)];
                                      
                % Update the position indices
                ind_q_i = ind_q_i + 2; % Add two to adjust for the two positions
            end
            
            % Initialize data for storing state
            n_steps = length(obj.tmat);
            for i = 1:obj.n_agents
                obj.state_value{i} = zeros(1,n_steps);
            end
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
                    plot(obj.xmat(x_ind, :), obj.xmat(y_ind, :), 'linewidth', 2, 'color', obj.agent_colors(i,:));
                end
            end
            
            % Loop through and calculate formation error
            ind_q_d = 1:2; % Stores the indices of agent i within q_d            
            formation_error = zeros(size(obj.tmat));
            for i = 1:obj.n_agents
                % Create a figure for the actual and desired positions
                figure;
                
                % Get state indices
                x_ind = obj.state_ind{i}(1);
                y_ind = obj.state_ind{i}(2);
                
                % Plot the x position vs desired position
                subplot(3, 1, 1);
                plot(obj.tmat, obj.qd_mat(ind_q_d(1), :), ':k', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(x_ind, :), 'b', 'linewidth', 2);
                ylabel('x');
                set(gca, 'fontsize', 18);
                
                % Plot the y position vs desired position
                subplot(3, 1, 2);
                plot(obj.tmat, obj.qd_mat(ind_q_d(2), :), ':k', 'linewidth', 3); hold on;
                plot(obj.tmat, obj.xmat(y_ind, :), 'b', 'linewidth', 2);
                ylabel('y');
                set(gca, 'fontsize', 18);
                
                % Plot the state of the vehicle
                subplot(3,1,3);
                plot(obj.tmat,obj.state_value{i}, 'b', 'linewidth', 2);
                ylabel('State');
                xlabel('time (s)');
                set(gca, 'fontsize', 18);
                
                % Compute the formation error for each point in time
                err_pos = obj.qd_mat(ind_q_d, :) - obj.xmat([x_ind; y_ind], :);
                err_vec = zeros(size(formation_error));
                for k = 1:length(err_vec)
                    err_vec(k) = norm(err_pos(:,k));
                end
                formation_error = formation_error + err_vec;
                
                % Upate the position indices for the next agent
                ind_q_d = ind_q_d + 2;
            end
            
            % Plot the formation error
            figure;
            formation_error = formation_error ./ obj.n_agents;
            display(['Average error: ' num2str(mean(formation_error))]);
            plot(obj.tmat, formation_error, 'b', 'linewidth', 3);
            
            % Plot the desired curvature for each agent
            figure;
            curvature_des = zeros(obj.n_agents, length(obj.traj_follow{1}.x));
            curvature_act = zeros(obj.n_agents, size(obj.xmat, 2));
            t_curv = linspace(obj.tmat(1), obj.tmat(end), length(obj.traj_follow{1}.x));
            for i = 1:obj.n_agents
                for k = 1:length(obj.traj_follow{i}.x)
                    % Get the commanded velocities
                    qdot = [obj.traj_follow{i}.xdot(k); obj.traj_follow{i}.ydot(k)];
                    qddot = [obj.traj_follow{i}.xddot(k); obj.traj_follow{i}.yddot(k)];
                    [~, ~, kappa] = calculateVelocities(qdot, qddot);
                    curvature_des(i,k) = kappa;
                    
                    traj_vl.q = [obj.traj_follow{1}.x(k); obj.traj_follow{1}.y(k)];
                    traj_vl.qdot = [obj.traj_follow{1}.xdot(k); obj.traj_follow{1}.ydot(k)];
                    traj_vl.qddot = [obj.traj_follow{1}.xddot(k); obj.traj_follow{1}.yddot(k)];
                    traj_vl.qdddot = [obj.traj_follow{1}.xdddot(k); obj.traj_follow{1}.ydddot(k)];
                    
                    %traj = getFollowerTrajFromLeaderTraj(traj_vl, [-1.5; 1.5]);
                end
                
                for k = 1:size(obj.xmat, 2)
                    x = obj.xmat(obj.state_ind{i}, k);
                    [v, w] = obj.agents{i}.vehicle.kinematics.getVelocities(0, x, 0);
                    
                    % Get the curvature
                    curvature_act(i,k) = w/v; 
                end
                
                % plot the curvature
                hold on;
                plot(t_curv, curvature_des(i,:), ':', 'color', obj.agent_colors(i,:), 'linewidth', 2);
                plot(obj.tmat, curvature_act(i,:), 'color', obj.agent_colors(i,:), 'linewidth', 2);
            end
            
            % Plot the curvature bounds
            max_k_foll = 2.0;
            plot([t_curv(1) t_curv(end)], [max_k_foll, max_k_foll], 'r:', 'linewidth', 1);
            plot([t_curv(1) t_curv(end)], -[max_k_foll, max_k_foll], 'r:', 'linewidth', 1);
            set(gca, 'ylim', [-max_k_foll - .25, max_k_foll + .25]);
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
                obj.state_value{i}(k) = obj.agents{i}.state;
            end
        end
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

function [v, w, k, sig] = calculateVelAndCurve(traj)
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
    [psi, v, w, a, alpha] = getTrajectoryInformation(traj);
    
    k = w/v;
    sig = alpha/v - w/v^2;
end

function traj = getFollowerTrajFromLeaderTraj(traj_l, tau)
%getFollowerTrajFromLeaderTraj Returns the follower trajectory given the leader
%trajectory
%
% Inputs:
%   traj_l: Struct with leader trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
%   tau: 2x1 nominal offset from the leader
%
% Outputs:
%   traj: Struct with follower trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
    % Extract the leader trajectory information
    [psi_l, v_l, w_l, a_l, alpha_l] = getTrajectoryInformation(traj_l);
    R = [cos(psi_l), -sin(psi_l); sin(psi_l) cos(psi_l)];
    J = [0 -1; 1 0];
    
    % Calcualte the follower information
    traj.q = R*tau + traj_l.q;
    traj.qdot = w_l*R*J*tau + traj_l.qdot;
    traj.qddot = R*(w_l^2*J^2 + alpha_l*J)*tau + traj_l.qddot;
end

function [psi, v, w, a, alpha] = getTrajectoryInformation(traj)
%getTrajectoryInformation calcualte trajectory information directly from
%trajectory
%
% Inputs:
%   traj: Struct with trajectory information
%       .q = position
%       .qdot = velocity vector
%       .qddot = acceleration vector
%       .qdddot = jerk vector
%
% Outputs:
%   psi: orientation
%   v: translational velocity
%   w: rotational velocity
%   a: translational acceleration
%   alpha: rotational acceleration

    % Extract trajectory information
    xdot = traj.qdot(1); % Velocity vector
    ydot = traj.qdot(2);
    xddot = traj.qddot(1); % Accleration vector
    yddot = traj.qddot(2);
    xdddot = traj.qdddot(1); % Jerk vector
    ydddot = traj.qdddot(2);
    
    % Calculate the trajectgory variables
    psi = atan2(ydot, xdot);
    v = sqrt(xdot^2+ydot^2);
    w = 1/v^2*(xdot*yddot - ydot*xddot);
    a = (xdot*xddot + ydot*yddot)/v;
    alpha = (xdot*ydddot-ydot*xdddot)/v^2 - 2*a*w/v;    
end


