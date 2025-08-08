clear all

% main.m - Runs the optimizer for both formulations ('handmade' and 'mip') and performs simulation

% Select mode ('handmade' or 'mip')
mode = 'handmade'; % Change to 'mip' or 'handmade' for standard MIP formulation

% Create the optimizer
system=returnSystemConfig('openQNforFormulationComparison');
optim = buildOptimizerFormulationComparisons(mode,system,'intlinprog');
symConfig = returnSymConfig('formulationComparison');
% Run the simulation based on the mode
runSimulation(symConfig,optim,system);

% Title for the plot based on mode
switch mode
    case 'handmade'
        title('Complementarity formulation');
    case 'mip'
        title('MIP standard');
end

hold on;
