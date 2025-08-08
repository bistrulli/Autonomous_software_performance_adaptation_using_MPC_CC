clear all
getConstantsIntoWorkspace

c=sdpvar(4,horizon);
x0=sdpvar(4,horizon);
xDot=sdpvar(4,horizon);
s=sdpvar(4,horizon);
obj=1;
gap=x0-c;

constr=[c>=1;c(2:4,:)<=16; x0>=0; c(1,:)==1000 ; x0<=400];
for tIdx=1:horizon-1
    xDot(:,tIdx)=(P-eye(4))*(mu.*min(c(:,tIdx),x0(:,tIdx)));
    constr=[ constr; 
    sum(c(2:4,tIdx))<=20;    
    x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
end



trackError=x0(2:4,2:end)-75;
optim=optimizer([constr],sum(abs(trackError(:))),sdpsettings('solver','mosek'),[x0(:,1)],c)
simPlot
title('MIP standard')

%xopt=stepOpt([x0R; newC(:)]);
hold on
