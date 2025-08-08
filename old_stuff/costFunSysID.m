function [error] = costFunSysID(uSeq,x0)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
persistent stepOpt
if isempty(stepOpt)
    [stepOpt]=stepSystemOptimizer();
end
horizon=20;
deltaT=0.1;
tInt=[0,deltaT];
error=0;
uSeq=reshape(uSeq,[horizon,3]);
x0=reshape(x0,[4,1]);
for i=1:horizon
    newC=uSeq(i,:)';
    %odeFun=@(t,x0) stepOpt([x0;1000; newC]);
    %odeFun=@(t,x0) stepSystem(x0,[1000; newC]);
    %[tff,x0ff]=ode15s(odeFun,tInt,x0,[]);
    x1hat=x0+deltaT*stepSystem(x0,[1000; newC]);
    x0=reshape(x1hat,[4,1]);
    tmp=(x1hat(2:end)-75).^2;
    error=error+sum(tmp);
    tInt=tInt+diff(tInt);
end

end