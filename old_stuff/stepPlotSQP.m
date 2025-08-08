close all


tic
liveInstances=zeros(3,200+spoolTime)+1;
history=[]
timeTable=[]
usedCore=[];
for kSim=1:200   
    [cOpt,c,b,d]=optim([x0R; vec(liveInstances(:,kSim:kSim+spoolTime-1))]); 
    coreToAllocate=cOpt(1:stateSize);
    intancesThisTime=cOpt(stateSize+1:end);
    assert(size(intancesThisTime,1)==3)
    
    liveInstances(:,kSim+spoolTime)=round(liveInstances(:,kSim+spoolTime-1)+intancesThisTime);
    odeFun = @(t,x0 ) stepSystem(x0,[coreToAllocate]);
    [tff,x0ff]=ode45(odeFun,tInt,x0R,[]);
    history=[history;x0ff];
    timeTable=[timeTable;tff];
    usedCore=[usedCore;coreToAllocate'];
    x0R=x0ff(end,:)'
    intancesThisTime'
    coreToAllocate'
    % First subplot (top-left)
    
    tInt=tInt+diff(tInt);
    
end
toc
subplot(3, 1, 2); % 2 rows, 2 columns, first subplot
seq=1:(200+spoolTime);
plot((-1+seq)*deltaT,liveInstances')
legend('2-core VM','4-core VM','8-core VM')

subplot(3, 1, 1); % 2 rows, 2 columns, first subplot
plot(timeTable',history);
legend('x1','x2','x3','x4')
title ('tracking x3 to 75')

subplot(3, 1, 3); % 2 rows, 2 columns, first subplot
seq=1:(200);
plot((-1+seq)*deltaT,usedCore(:,2:end));
legend('core on x2','core on x3','core on x4')
title ('tracking x3 to 75')