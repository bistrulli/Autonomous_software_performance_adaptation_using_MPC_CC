close all


tic
liveInstances=zeros(3,lenSim+spoolTime)+2;
history=[]
timeTable=[]
usedCore=[];

for kSim=1:lenSim 
   
    disturbance=disturbanceProfile(kSim)*diff(tInt);
    if openQNFlag~=1
        distCont=sum(x0R)+disturbance*diff(tInt);
    end
    [cOpt,stat1,stat2,stat3]=steadyState_controller(distCont);    
    assert(stat1==0)
    coreToAllocate=cOpt(1:stateSize);
    intancesThisTime=cOpt(stateSize+1:stateSize+3);
    assert(size(intancesThisTime,1)==3)
    liveInstances(:,kSim+spoolTime)=round(intancesThisTime);
    if ~isempty(steadyState_controllerFronzenVMs)
        [cOpt,stat1,stat2,stat3]=steadyState_controllerFronzenVMs([distCont;liveInstances(:,kSim)]);    
        assert(stat1==0)
        coreToAllocate=cOpt(1:stateSize);
    end
    odeFun = @(t,x0 ) stepQNSystem(x0,[coreToAllocate;disturbance]);
    [tff,x0ff]=ode113(odeFun,tInt,x0R,[]);
    history=[history;x0ff];
    timeTable=[timeTable;tff];
    usedCore=[usedCore;coreToAllocate'];
    
    x0R=x0ff(end,:)'
    [kSim, intancesThisTime',    coreToAllocate']
    % First subplot (top-left)
    
    tInt=tInt+diff(tInt);
    
end
toc
%%
s1=subplot(3, 1, 2); % 2 rows, 2 columns, first subplot
seq=1:(lenSim+spoolTime);
plot((-1+seq)*deltaT,liveInstances')
legend('2-core VM','4-core VM','8-core VM')
ylim(s1,[0,15])

s2=subplot(3, 1, 1); % 2 rows, 2 columns, first subplot
plot(timeTable',history);
legend('x1','x2','x3','x4')
ylim(s2,[0,800])

s3=subplot(3, 1, 3); % 2 rows, 2 columns, first subplot
ylim(s3,[0,110])
seq=1:(lenSim);
plot((-1+seq)*deltaT,usedCore(:,2:end));
legend('core on x2','core on x3','core on x4')
linkaxes([s1 s2 s3],'x')