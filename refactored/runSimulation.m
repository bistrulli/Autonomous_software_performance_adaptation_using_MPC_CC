function runSimulation(symConfig,optim,config)
    % Supported modes: 'simplot', 'step', 'ss'


    
    deltaT = config.deltaT;
    x0R = symConfig.x0R;
    lenSim = symConfig.lenSim;
    spoolTime = config.spoolTime;
    stateSize = config.stateSize;
    tInt = [0; deltaT];
    liveInstances = ones(3, lenSim + spoolTime);
    history = []; timeTable = []; usedCore = [];
    tic;
    for kSim = 1:lenSimEnter Journal Title, ISSN or Publicsher Name
        disturbance = symConfig.disturbanceProfile(kSim:kSim + config.horizon - 1);
        if all(isempty(disturbance))
            disturbance=0;
        end
        switch symConfig.simulationKind
            case 'noVM'
                [cOpt, stat1, stat2, stat3]  = optim(x0R);
                disturbance=0;                
                liveInstances(:, kSim + spoolTime) = 0; 
                coreToAllocate = vec(cOpt(:,1));                
                
            case 'step'                
                noiseOnSensors=symConfig.noiseOnSensors;
                distPreviewInMPCs=symConfig.distPreviewInMPCs;
                if symConfig.injectDisturbance==false
                    disturbance=zeros(size(disturbance));
                    noiseOnSensors=0;
                    distPreviewInMPCs=1;                   
                end
                distP = vec(disturbance + randn(size(disturbance)) * noiseOnSensors) * distPreviewInMPCs;
                feedbackX0 = max(0, round(x0R + randn(size(x0R)) * noiseOnSensors));

                [cOpt, ~, ~, ~,optim] = optim([distP; feedbackX0; vec(liveInstances(:, kSim:kSim + spoolTime - 1))]);
                disturbance = disturbance(1);

                coreToAllocate = cOpt(1:stateSize);    
                liveInstances(:, kSim + spoolTime) = round(liveInstances(:, kSim + spoolTime - 1) + cOpt(stateSize+1:end));
                

            case 'ss'
                if symConfig.injectDisturbance==false
                    disturbance=zeros(size(disturbance));                                    
                end
                steadyState_controller=optim.a;
                steadyState_controllerFronzenVMs=optim.b;
                distCont = sum(x0R) + disturbance(1) * diff(tInt); %integral effect                
                [cOpt, stat1, ~, ~] = steadyState_controller(distCont); assert(stat1 == 0);
                intancesThisTime = cOpt(stateSize+1:stateSize+3);
                liveInstances(:, kSim + spoolTime) = round(intancesThisTime);        
                [cOpt, stat1, ~, ~] = steadyState_controllerFronzenVMs([distCont; liveInstances(:, kSim)]);
                assert(stat1 == 0);
                coreToAllocate = cOpt(1:stateSize);                
                disturbance = disturbance(1);

            otherwise
                error('Unknown simulation mode: %s', mode);
        end
        [tff, x0ff,tInt] = simStep(@stepQNSystem, x0R, tInt, [coreToAllocate; disturbance],config.P,config.mu,config.B);
        x0R=x0ff(end,:)';
        
        disp([kSim,x0R'])
        usedCore = [usedCore; coreToAllocate'];
        history = [history; x0ff];
        timeTable = [timeTable; tff];
    end
    toc
    plotResults(deltaT, history, timeTable, liveInstances, usedCore, lenSim, spoolTime, symConfig);
    
end

function [tff, x0ff,tInt] = simStep(sysFun, x0R, tInt, controlInput,P,mu,B)
    odeFun = @(t, x0) sysFun(x0, controlInput,P,mu,B);
    [tff, x0ff] = ode45(odeFun, tInt, x0R);
    
    tInt = tInt + diff(tInt);
end

function plotResults(deltaT, history, timeTable, liveInstances, usedCore, lenSim, spoolTime, symConfig)
    figure; 
    hold on;
    s1 = subplot(3, 1, 2);
    seq = 1:(lenSim + spoolTime);
    plot((-1 + seq) * deltaT, liveInstances');
    legend('2-core VM', '4-core VM', '8-core VM');
    ylim(s1, [0, 20]);

    s2 = subplot(3, 1, 1);
    plot(timeTable', history);
    legend('x1', 'x2', 'x3', 'x4');
    ylim(s2, [0, 800]);

    s3 = subplot(3, 1, 3);
    ylim(s3, [0, 110]);
    seq = 1:lenSim;
    plot((-1 + seq) * deltaT, usedCore(:, 2:end));
    legend('core on x2', 'core on x3', 'core on x4');

    linkaxes([s1, s2, s3], 'x');
    title(s2, sprintf('Simulation Mode: %s', upper(symConfig.title)));
end
