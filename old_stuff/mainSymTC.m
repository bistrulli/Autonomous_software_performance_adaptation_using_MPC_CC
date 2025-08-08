c=1;
close all

mu=1;
xa=0:0.01:2;


y=mu*(min(0,xa-c)+c);

plot(xa,y)

x=sdpvar(1);
s=sdpvar(1);
obj=abs(s-(x-c));
y=mu*(s+c);
a=optimizer([s<=0; ],obj,[],x,y)


    



ya=a(xa);
hold on

plot(xa,ya)

%%
figA=figure
hold on
figB=figure
hold on
[stepOpt,kktOpt]=stepSystemOptimizer()

x0=ones(4,1)*100;
x0A=x0;
tInt=[0,3];
for i=1:10
    sum(x0)
    newC=1+randi(15,[3,1]);
    odeFun = @(t,x0 ) stepSystem(x0,[1000; newC]);
    odeBVLS = @(t,x0 ) stepBVLS(x0,[1000; newC]);
    odeFunOpt=@(t,x0) stepOpt([x0;1000; newC]);
    [tff,x0ff]=ode15s(odeFunOpt,tInt,x0,[]);
    [tffA,x0ffA]=ode15s(odeBVLS,tInt,x0A,[]);
    figure(figA)
    plot(tff,x0ff);
    figure(figB)
    plot(tffA,x0ffA);
    x0=x0ff(end,:)';
    x0A=x0ffA(end,:)';
    tInt=tInt+3;
    
end

%%


stepOpt=stepSystemOptimizer()

