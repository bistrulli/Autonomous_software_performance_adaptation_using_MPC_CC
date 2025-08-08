function config = returnSystemConfig(systemUnderConsideration)
    config=struct();
    
    switch systemUnderConsideration
        case 'formulationComparison'
            config = formulationComparisonQN();
        case 'costComparison'
            config = formulationComparisonQN();
            config.horizon = 45;
            config.spoolTime = 25;
            config.deltaT=1
        case 'uOptComparison'
            config = formulationComparisonQN();
            config.horizon = 30;
            config.spoolTime = 25;
            config.deltaT=1
            config.B=[1;0 ; 0; 0]
            config.limVM=16;
        case 'previewPlayground'
            config = previewPlaygroundConfig();
        case 'openQNforFormulationComparison'
            config=openQNforFormulationComparisonConfig();
        otherwise
            error('Unknown AppMode: %s', AppConstants.AppMode);
    end    
    config.systemUnderConsideration=systemUnderConsideration;
    
end



function config = formulationComparisonQN()
    config=struct();
    config.stateSize = 4;
    config.P = [    [0, 1.0, 0.0, 0.0],
                    [0.0, 0, 0.6, 0.4],
                    [0.7, 0.3, 0, 0.0],
                    [0.8, 0.2, 0, 0.0]
                ]';
    config.mu = [1., 8.0, 4.0, 4.0]';

    config.horizon = 35;
    config.deltaT = 0.1;
    config.B=[0; 1 ; 0; 0];
    config.limVM=160;
    config.steadyState_controllerFronzenVMs = [];
end



function config = previewPlaygroundConfig()
    config=struct();
    config.stateSize = 4;
    config.P = [
         0 ,        0    ,1 ,   1;
    .45000  ,       0   , 0.0000  ,  0.0000;
         0  ,  0.6000  ,       0   ,      0;
         0  ,  0.4000 ,        0    ,     0;
    ];
    config.mu = [    1.4,    18.0,    4.0,    4.0   ]';

    config.horizon = 30;
    config.deltaT = 0.1;
    config.spoolTime = 20;
    config.B=[0; 1 ; 0; 0];
    config.limVM=160;
end

