getConstantsIntoWorkspace
c=sdpvar(4,horizon);
x0=sdpvar(4,horizon);
xDot=sdpvar(4,horizon);
s=sdpvar(4,horizon);
lambda=sdpvar(4,horizon);
obj=1;
gap=x0-c;


constrO=[ c>=1;c(2:4,:)<=16; x0>=0; x0<=400;  c(1,:)==1000];
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
            sum(c(2:4,tIdx))<=20;
            x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
end

trackError=x0(2:4,2:end)-75;

optim=optimizer([constrO; KKtFaiDaTe;],sum(abs(trackError(:))),sdpsettings('solver','mosek'),[x0(:,1)],c)


simPlot

title('comp constr fai da te')

%xopt=stepOpt([x0R; newC(:)]);
hold on
