
clear all
% Select mode ('MILP' or 'MIQP' or 'MINLP')
mode = 'MINLP'; 

% Create the optimizer
system=returnSystemConfig('costComparison');
optim = buildOptimizerFormulationCostComparisons(mode,system)
symConfig = returnSymConfig('costsComparison');
% Run the simulation based on the mode
symConfig.injectDisturbance=false;
runSimulation(symConfig,optim,system);

