close all


tic
liveInstances=zeros(3,lenSim+spoolTime)+1;
history=[]
timeTable=[]
usedCore=[];

for kSim=1:lenSim 
    
    disturbance=disturbanceProfile(kSim:kSim+horizon-1);
    if isempty(disturbance)
        [cOpt,c,b,d]=optim([x0R; vec(liveInstances(:,kSim:kSim+spoolTime-1))]); 
        disturbance=0;
    else
        distP=vec(disturbance+randn(size(disturbance))*injectNoise)*disturbancePreview;
        feedbackX0=max(0,round(x0R+randn(size(x0R))*injectNoise));
        [cOpt,c,b,d]=optim([distP;feedbackX0; vec(liveInstances(:,kSim:kSim+spoolTime-1))]); 
        disturbance=disturbance(1);
    end
    coreToAllocate=cOpt(1:stateSize);
    intancesThisTime=cOpt(stateSize+1:end);
    assert(size(intancesThisTime,1)==3)
    
    liveInstances(:,kSim+spoolTime)=round(liveInstances(:,kSim+spoolTime-1)+intancesThisTime);
    odeFun = @(t,x0 ) stepQNSystem(x0,[coreToAllocate;disturbance]);
    [tff,x0ff]=ode15s(odeFun,tInt,x0R,[]);
    history=[history;x0ff];
    timeTable=[timeTable;tff];
    usedCore=[usedCore;coreToAllocate'];
    x0R=x0ff(end,:)'
    [kSim, intancesThisTime',    coreToAllocate(2:end)']
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