function launch_switched()
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
    
    %%%%%%%%%%%%%%%%% Switched Field Problems %%%%%%%%%%%%%%%%%%%%%
    % Switching vector fields
    %scenario = SwithingLineScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    scenario = SwithingLineScenarioObstacleAvoid(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    %scenario = SwithingLineScenarioObstacleAvoidBetterSwitch(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );       
        
    % Run the scenario
    scenario.runScenario();
end


