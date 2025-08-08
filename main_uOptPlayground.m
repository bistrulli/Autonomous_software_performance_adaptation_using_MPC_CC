clear all

system=returnSystemConfig('uOptComparison');

% Create the optimizer
[optim,optimFroze] = buildOptimizeruOpt(system)
symConfig = returnSymConfig('steadyStateController');
% Run the simulation based on the mode
symConfig.injectDisturbance=true;
symConfig.distPreviewInMPCs=true;
symConfig.noiseOnSensors=false;
symConfig.title='\muOpt'
runSimulation(symConfig,struct('a',optim,'b',optimFroze),system);


optim=buildOptimizerFormulationCostComparisons('comparison_uOpt',system);
symConfig.simulationKind='step';


symConfig.title='MPC';
runSimulation(symConfig,optim,system);


