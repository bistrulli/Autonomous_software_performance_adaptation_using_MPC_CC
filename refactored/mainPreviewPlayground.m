clear all

system=returnSystemConfig('previewPlayground');

mode = 'previewPlayground'; 
% Create the optimizer
optim = buildOptimizerFormulationCostComparisons(mode,system)
symConfig = returnSymConfig('previewPlayground');
% Run the simulation based on the mode
symConfig.injectDisturbance=true;
symConfig.distPreviewInMPCs=false;
symConfig.noiseOnSensors=true;
symConfig.title='no preview, tracking x2+x3, and x2+x4 to 275';
runSimulation(symConfig,optim,system);


symConfig.distPreviewInMPCs=true;

symConfig.title='with preview, tracking x2+x3, and x2+x4 to 275';
runSimulation(symConfig,optim,system);

