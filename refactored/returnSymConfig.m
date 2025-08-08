function config = returnSymConfig(simulationKind)
config = struct('title','');    
    switch simulationKind
        case 'formulationComparison'            
            config.useVM=false;
            config.useDisturbance=false;
            config.x0R=100*ones(4,1);
            config.lenSim=200;
            config.simulationKind='noVM';
            config.disturbanceProfile=@(x) [];
        case 'costsComparison'            
            config.injectDisturbance=false;
            config.distPreviewInMPCs=false;
            config.noiseOnSensors=true;
            config.x0R=100*ones(4,1);            
            config.lenSim=150;
            config.simulationKind='step';            
            config.disturbanceProfile = @(x)  [23*sin(x/30)+ 27*sin(x/45-13)+  21*sin(x/15+5)+(45+45*sign(x-100))];
        case 'previewPlayground'            
            config.injectDisturbance=false;
            config.distPreviewInMPCs=false;
            config.noiseOnSensors=true;
            config.x0R=400*ones(4,1);            
            config.lenSim=400;
            config.simulationKind='step';            
            config.disturbanceProfile = @(x) [23*sin(x/30)+ 27*sin(x/45-13)+  21*sin(x/15+5)+(45+45*sign(x-100))];
        case 'steadyStateController'            
            config.injectDisturbance=false;
            config.distPreviewInMPCs=false;
            config.noiseOnSensors=true;
            config.x0R=100*ones(4,1);            
            config.lenSim=500;
            config.simulationKind='ss';            
            config.disturbanceProfile = @(x) [23*sin(x/30)+ 27*sin(x/45-13)+  21*sin(x/15+5)+(20*sign(cos(x/40)))];
        otherwise
            error('Unknown AppMode: %s', AppConstants.AppMode);
    end
end
