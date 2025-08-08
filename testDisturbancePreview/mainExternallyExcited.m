addpath("../utilityScripts/")
addpath("../systemsInit/")
openQNwithDelaySystemInit 
%%% Important flags
% injectNoise=true;
% disturbancePreview=true;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

c=sdpvar(4,horizon);
x0=sdpvar(4,horizon);
xDot=sdpvar(4,horizon);
distPreview=sdpvar(1,horizon);
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
    x_inst<=18;    x_inst>=0; u_inst>=-1; u_inst<=1
x0>=0; x0<=2000;  c(1,:)==1000];
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
    xDot(2,tIdx)=xDot(2,tIdx)+distPreview(1,tIdx);
    constrO=[ constrO;
            sum(c(2:4,tIdx))<=[2,4,8]*x_inst(:,tIdx);
            x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
end
%x_instm, x0 are positive
trackError=sum(([1,1]*x0([2:3],2:end)-275).^2)+sum(([1,1]*x0([2,4],2:end)-275).^2)+sum((x0([1,2],2:end)).^2,"all")/100
           
actuationCost=sum(vecCost*x_inst(:,1:end).^2)*20+sum(vecCost*u_inst(:,1:end)*spoolTime)
cost=actuationCost+trackError;
optim=optimizer([constrO; KKtFaiDaTe;],cost,[],[distPreview(:); x0(:,1); vec(x_inst(:,1:spoolTime))],[vec(c(:,1)); vec(u_inst(:,1))])

optimize([constrO; KKtFaiDaTe;x0(:,1)==200; x_inst(:,1:spoolTime)==1],cost,sdpsettings('solver','gurobi','debug',5))

stepPlotInstances

title('miqp tracking 3 to 275 with disturbance')


hold on


