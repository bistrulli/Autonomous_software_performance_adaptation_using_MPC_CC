addpath("../utilityScripts/")
addpath("../systemsInit")
closedQNwithDelaySystemInit
c=sdpvar(4,horizon);
x0=sdpvar(4,horizon);
xDot=sdpvar(4,horizon);
s=sdpvar(4,horizon);
lambda=sdpvar(4,horizon);
obj=1;
gap=x0-c;
vecCost=[0.02,0.04,0.08];
% Dinamica delle repliche. Assumo 3 tagli da 2, 4 e 8 core
x_inst=sdpvar(3,horizon+spoolTime); %status of instances
x_cost=sdpvar(3,horizon+spoolTime); %status of instances
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
    x_inst<=8;    x_inst>=0; u_inst>=-1; u_inst<=1
x0>=0; x0<=40000;  c(1,:)==1000];
Q=eye(stateSize)*2;
KKtFaiDaTe=[];%[lambda>=0,s<=0]% sono implicate dal complements
for tIdx=1:horizon
    obj=obj+sum(abs(s(:,tIdx)-gap(:,tIdx)));
    e=-2*gap(:,tIdx);
    stationariety=Q*s(:,tIdx)+e+eye(stateSize)'*lambda(:,tIdx);
    KKtFaiDaTe=[KKtFaiDaTe;
        stationariety==0;
        complements(lambda(:,tIdx)>=0, -s(:,tIdx)>=0)
    ];
end
for tIdx=1:horizon-1
    xDot(:,tIdx)=(P-eye(4))*(mu.*(s(:,tIdx)+c(:,tIdx)));
    constrO=[ constrO;
            sum(c(2:4,tIdx))<=[2,4,8]*x_inst(:,tIdx);
            x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
end
%x_instm, x0 are positive
trackError=sum(abs(x0(3,2:end)-275))+sum(abs([0,1,0,1]*x0-80))/100;
xi=-5:0.1:5;
exploitConvexity=true;
if exploitConvexity
    f=interp1(xi,log(cosh(xi)),vecCost*x_inst(:,1:end),'lp');
else
    f=interp1(xi,log(cosh(xi)),vecCost*x_inst(:,1:end),'milp');% to *not* exploit convexity
end

actuationCost=sum(f)*20+sum(vecCost*u_inst(:,end)*spoolTime)
cost=actuationCost+trackError;
optim=optimizer([constrO; KKtFaiDaTe;],cost,sdpsettings('solver','gurobi','debug',5),[x0(:,1); vec(x_inst(:,1:spoolTime))],[vec(c(:,1)); vec(u_inst(:,1))])
%optimize([constrO; KKtFaiDaTe;x0(:,1)==200*rand(4,1); x_inst(:,1:spoolTime)==1],cost,sdpsettings('solver','mosek','debug',5))

stepPlotInstances

title('log cosh tracking 3 to 275')

%xopt=stepOpt([x0R; newC(:)]);
hold on
