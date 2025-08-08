function config = initDynamicalSystem()
    switch AppConstants.AppMode
        case 'openQN'
            config = openQNConfig();
        case 'closedQN'
            config = closedQNConfig();
        case 'closedQNUnbalance'
            config = testConfig();
        otherwise
            error('Unknown AppMode: %s', AppConstants.AppMode);
    end
end