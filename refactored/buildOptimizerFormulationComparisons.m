function optim = buildOptimizerFormulationComparisons(mode,config,solver)
    % mode: 'handmade' or 'mip'
    if nargin<3
        solver='gurobi';
    end
    P=config.P;
    mu=config.mu;
    deltaT=config.deltaT;
    horizon=config.horizon;

    c = sdpvar(4, horizon);
    x0 = sdpvar(4, horizon);
    xDot = sdpvar(4, horizon);
    s = sdpvar(4, horizon);
    lambda = sdpvar(4, horizon); % only used in 'handmade'
    gap = x0 - c;

    % Base constraints
    constr = [c >= 1; c(2:4,:) <= 16; x0 >= 0; x0 <= 400; c(1,:) == 1000];

    obj = 1; % INNER OBJ to minimize
    KKtFaiDaTe=[];%[lambda>=0,s<=0]% sono implicate dal complements
    if strcmp(mode, 'handmade')
        
        Q = eye(4) * 2;
        for tIdx = 1:horizon
            %obj = obj + sum(abs(s(:,tIdx) - gap(:,tIdx)));
            e = -2 * gap(:,tIdx);
            stationarity = Q * s(:,tIdx) + e + eye(4)' * lambda(:,tIdx);
            KKtFaiDaTe = [KKtFaiDaTe;
                stationarity == 0;
                complements(lambda(:,tIdx) >= 0, -s(:,tIdx) >= 0)];
        end
    end


    for tIdx = 1:horizon-1
        switch mode
            case 'handmade'
                xDot(:,tIdx) = (P - eye(4)) * (mu .* (s(:,tIdx) + c(:,tIdx)));
            case 'mip'
                xDot(:,tIdx) = (P - eye(4)) * (mu .* min(c(:,tIdx), x0(:,tIdx)));
        end
        constr = [constr;
            sum(c(2:4,tIdx)) <= 20;
            x0(:,tIdx+1) == x0(:,tIdx) + deltaT * xDot(:,tIdx)];
    end


    % Objective: track reference
    trackError = x0(2:4,2:end) - 75;
    
    
    % Create optimizer
    %optimize([constr; KKtFaiDaTe;x0(:,1)==400],sum(abs(trackError(:))))
    optim = optimizer([constr; KKtFaiDaTe], sum(abs(trackError(:))), sdpsettings('solver',solver), x0(:,1), c);
end
