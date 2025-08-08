function optim = buildOptimizerFormulationCostComparisons(kindOfCost,config)
    P=config.P;
    mu=config.mu;
    deltaT=config.deltaT;
    horizon=config.horizon;
    spoolTime = config.spoolTime;
    stateSize=size(P,1);
    c=sdpvar(4,horizon);
    x0=sdpvar(4,horizon);
    xDot=sdpvar(4,horizon);
    s=sdpvar(4,horizon);
    lambda=sdpvar(4,horizon);
    distPreview=sdpvar(1,horizon);
    gap=x0-c;
    vecCost=[0.02,0.04,0.08];
    % Dinamica delle repliche. Assumo 3 tagli da 2, 4 e 8 core
    x_inst=sdpvar(3,horizon+spoolTime); %status of instances
    u_inst=intvar(3,horizon); %control action on adding/removing istances
    %x_inst in 1:spooltime represents the in past 
    constrO=[ ];
    for tIdx=1:spoolTime   
        constrO=[constrO;
            %x_inst(:,tIdx+1)==x_inst(:,tIdx) not actually here
        ];
    end
    %from present upward we can control the intances
    idxShifted=1;
    for tIdx=spoolTime:horizon+spoolTime-1    
        constrO=[ constrO;
            x_inst(:,tIdx+1)==x_inst(:,tIdx)+u_inst(:,idxShifted);        
        ];
        idxShifted=idxShifted+1;
    end
    
    
    
    constrO=[ constrO; c>=1;
        x_inst<=config.limVM;    x_inst>=0; u_inst>=-1; u_inst<=1
    x0>=0; x0<=4000;  c(1,:)==1000];
    Q=eye(stateSize)*2;
    KKtFaiDaTe=[];%[lambda>=0,s<=0]% sono implicate dal complements
    for tIdx=1:horizon
        e=-2*gap(:,tIdx);
        stationariety=Q*s(:,tIdx)+e+eye(stateSize)'*lambda(:,tIdx);
        KKtFaiDaTe=[KKtFaiDaTe;
            stationariety==0;
            complements(lambda(:,tIdx)>=0, -s(:,tIdx)>=0)
        ];
    end
    for tIdx=1:horizon-1
        xDot(:,tIdx)=(P-eye(4))*(mu.*(s(:,tIdx)+c(:,tIdx)));
        xDot(:,tIdx)=xDot(:,tIdx)+config.B*distPreview(1,tIdx);
        constrO=[ constrO;
                sum(c(2:4,tIdx))<=[2,4,8]*x_inst(:,tIdx);
                x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
    end
    %x_instm, x0 are positive
    
    
    switch kindOfCost
        case 'MILP'       
            
            actuationCost=0.1*sum(vecCost*x_inst(:,1:end));%+sum(vecCost*u_inst(:,1:end)*spoolTime);%+sum(abs(x_inst(:,1:end-1)-x_inst(:,2:end)),'all')/100;
            cost=actuationCost+trackError;
        case 'MIQP'
            trackError=sum((x0(1,2:end)-150).^2)+sum(([0,1,1,1]*x0).^2)/100;
            actuationCost=sum(vecCost*x_inst(:,1:end))+sum(vecCost*u_inst(:,1:end)*spoolTime);
            cost=actuationCost+trackError;
        case 'MINLP'
            
            xi=-5:0.1:5;
            exploitConvexity=true;
            if exploitConvexity
                f=interp1(xi,log(cosh(xi)),vecCost*x_inst(:,1:end),'lp');
            else
                f=interp1(xi,log(cosh(xi)),vecCost*x_inst(:,1:end),'milp');% to *not* exploit convexity
            end
            trackError=sum(abs(x0(1,2:end)-250))+sum(abs([0,1,1,1]*(x0(:,1:end-1)-x0(:,2:end))));
            actuationCost=sum(f)*0.1
            cost=actuationCost+trackError;
        case 'previewPlayground'
            trackError=sum(abs([1,1]*x0([2:3],2:end)-275))+sum(abs([1,1]*x0([2,4],2:end)-275));
            actuationCost=sum(vecCost*x_inst(:,1:end))%+sum(vecCost*u_inst(:,1:end)*spoolTime);

            cost=trackError+actuationCost;
        case 'comparison_uOpt'
            cost=sum(abs([1,0,0,0]*x0-300),'all')+sum(vecCost*x_inst)/1000;

        otherwise
            error('Unknown AppMode: %s', mode);
    end
    aaa=sdpsettings('solver','gurobi','warmstart',1);
    aaa.gurobi.MIPGap=1E-2;
    %sdpsettings('solver',AppConstants.solver,'warmstart',1)
    optim=optimizer([constrO; KKtFaiDaTe;],cost,aaa,[vec(distPreview);x0(:,1); vec(x_inst(:,1:spoolTime))],[vec(c(:,1)); vec(u_inst(:,1))]);
    
    end