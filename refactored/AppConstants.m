classdef AppConstants
    %APPCONSTANTS Centralized global constants for the application
    
    properties (Constant)
        % Application info
        AppName = 'MyAwesomeApp'
        Version = '1.0.0'
        systemUnderConsideration="openQNforFormulationComparison" % "closedQN", "closedQNUnbalanced"
        
        InjectNoise = false;
        DisturbancePreview = false;        
        useDelay=true;
    
    end
end
