function launch_simple_fields()
    % Clear previous work
    close all;
    clear;
    clc;
    
    % Configure file path
    addpath Dependencies
    addpath Dependencies/intersections
    addpath Scenarios
    addpath Sensors
    addpath VectorFields
    addpath Vehicle
    addpath Vehicle/Kinematics
    addpath World
    
    %%%%%%%%%%%%%%%%% Additional scenarios in which you may be interested %%%%%%%%%%%%%%%%%%%%%
    % Reference tracking scenario
    %scenario = ReferenceTrackingScenario(BetterUnicycleVehicle);
    %scenario = ReferenceTrackingScenario(SimpleUnicycleVehicle);
    %scenario = ReferenceTrackingScenario(SmoothDifferentialDriveVehicle);
    
    % Velocity tracking scenarios
    %scenario = VelocityTrackingScenario(BetterUnicycleVehicle);    
    %scenario = VelocityTrackingScenario(SimpleUnicycleVehicle);
    
    %%%%%%%%%%%%%%%%% Simple Vector Field Problems %%%%%%%%%%%%%%%%%%%%%
    % Initialize the plotting variables (a meshgrid is used with x_vec and
    % y_vec for originating the arrows)
    x_vec = -5:.5:5; % x values for plotting arrows in the vector field
    y_vec = -5:.5:5; % y values for plotting arrows in the vector field
    
    % Create a vector field
    %field = GoToGoalField(x_vec, y_vec, [3; 4], 1);
    %field = AvoidObstacle(x_vec, y_vec, [1; 1], 1);
    %field = OrbitField(x_vec, y_vec, [1; 1], 2, -0.25, .1);
    %field = LineVectorField(x_vec, y_vec, [0; 0], 0, 1, 1);
    field = LineVectorField(x_vec, y_vec, [-2; 2], pi/4, 1, 1);
    
    % Vector field scenario
    scenario = VectorFieldScenario(field, BetterUnicycleVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = VectorFieldScenario(field, SimpleUnicycleVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = VectorFieldScenario(field, SmoothDifferentialDriveVehicle, EmptyWorld, VECTOR_FOLLOWING_TYPE.POINT);
    
    % Run the scenario
    scenario.runScenario();
end


