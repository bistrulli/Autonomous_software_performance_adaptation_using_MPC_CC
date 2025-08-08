function config = closedQNUnbalancedConfig()
    config=struct();
    config.stateSize = 4;
    config.P = [
        [0, 1.0, 0.0, 0.0];
        [0.0, 0, 0.6, 0.4];
        [0.7, 0.3, 0, 0.0];
        [0.8, 0.2, 0, 0.0]
    ]';
    config.mu = [1.0, 8.0, 4.0, 4.0]';
    
    config.horizon = 45;
    config.deltaT = 0.1;
    config.spoolTime = 30;
    
    config.steadyState_controllerFronzenVMs = [];
end
