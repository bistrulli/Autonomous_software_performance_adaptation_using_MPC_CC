function config = closedQNConfig()
config=struct();    
config.stateSize = 4;
    config.P = [
        [0, 1.0, 0.0, 0.0];
        [0.0, 0, 0.5, 0.5];
        [1, 0.0, 0, 0.0];
        [1, 0.0, 0, 0.0]
    ]';
    config.mu = [1.0, 8.0, 4.0, 4.0]';
    
    config.horizon = 45;
    config.spoolTime = 30;
    config.deltaT = 0.1;
    config.steadyState_controllerFronzenVMs = [];
end
