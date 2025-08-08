getConstantsIntoWorkspace
c=sdpvar(4,horizon);
x0=sdpvar(4,horizon);
xDot=sdpvar(4,horizon);
s=sdpvar(4,horizon);
obj=1;
gap=x0-c;

constrI=[s<=0; ];
constrO=[x0>=0; x0<=400; c>=1;c(2:4,:)<=16; c(1,:)==1000 ];
for tIdx=1:horizon
    obj=obj+sum(abs(s(:,tIdx)-gap(:,tIdx)));
    constrI=[ xDot(:,tIdx)==(P-eye(4))*(mu.*(s(:,tIdx)+c(:,tIdx))); % P is zero on diagonal
        constrI];
end
for tIdx=1:horizon-1
    constrO=[ constrO; x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx); ];
end

KKTCond=kkt(constrI,obj,[x0(:);c(:)]);
rng(1)
%%


trackError=x0(2:4,2:end)-75;

optim=optimizer([constrO; KKTCond;],sum(abs(trackError(:))),sdpsettings('solver','mosek'),[x0(:,1)],c)

simPlot
title('comp constr')

%xopt=stepOpt([x0R; newC(:)]);
hold on
