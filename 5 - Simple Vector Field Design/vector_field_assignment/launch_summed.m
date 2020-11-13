function launch_summed()
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
    
    %%%%%%%%%%%%%%%%% Summed Vector Field Problems %%%%%%%%%%%%%%%%%%%%%
    % Summed vector fields
%     scenario = CombinedGoToGoalVectorScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
%     scenario = CombinedGoToGoalOrbitAvoidScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    scenario = CombinedGoToGoalOrbitAvoidWithBarrierScenario(BetterUnicycleVehicle, VECTOR_FOLLOWING_TYPE.POINT );
    
    % Run the scenario
    scenario.runScenario();
end


