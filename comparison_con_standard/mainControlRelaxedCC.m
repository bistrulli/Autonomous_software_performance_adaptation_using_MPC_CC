getConstantsIntoWorkspace

import casadi.*

opti = casadi.Opti()
useCasadi=1
%%
c=opti.variable(4,horizon);
x0=opti.variable(4,horizon);
x0para=opti.parameter(4);
xDot=opti.variable(4,horizon);
s=opti.variable(4,horizon);
lambda=opti.variable(4,horizon);

gap=x0-c;


opti.subject_to(c(:)>=1);
opti.subject_to(vec(c(2:4,:))<=16);
opti.subject_to(x0(:)>=0)
opti.subject_to(vec(c(1,:))==1000);
opti.subject_to(x0(:,1)==x0para);
Q=eye(stateSize)*2;
opti.subject_to(lambda(:)>=0)
opti.subject_to(s(:)<=0)% sono implicate dal complements
for tIdx=1:horizon
    e=-2*gap(:,tIdx);
    stationariety=Q*s(:,tIdx)+e+eye(stateSize)'*lambda(:,tIdx);
    opti.subject_to(stationariety(:)==0);
    opti.subject_to(lambda(:,tIdx).*(-s(:,tIdx))<=.0001);
end
for tIdx=1:horizon-1
    xDot(:,tIdx)=(P-eye(4))*(mu.*(s(:,tIdx)+c(:,tIdx)));   
    opti.subject_to(x0(:,tIdx+1)==x0(:,tIdx)+deltaT*xDot(:,tIdx))
end

trackError=x0(2:4,2:end)-75;

opti.minimize(sum(abs(trackError(:))))
opti.set_value(x0para, 10*ones(4,1))
p_opts = struct('expand',true);
s_opts = struct('max_iter',100);
p_opts.error_on_fail=false;
opti.solver('ipopt',struct('error_on_fail',false),s_opts);
%sol = opti.solve()

simPlot

title('comp constr fai da te')

%xopt=stepOpt([x0R; newC(:)]);
hold on
