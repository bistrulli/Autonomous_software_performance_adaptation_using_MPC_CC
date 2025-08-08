clear all
stateSize=4
P = [
         0 ,        0    ,1 ,   1;
    .45000  ,       0   , 0.0000  ,  0.0000;
         0  ,  0.6000  ,       0   ,      0;
         0  ,  0.4000 ,        0    ,     0;
    ];


mu = [    1.4,    18.0,    4.0,    4.0   ]';

stepQNSystem([],[],P,mu);
lenSim=500
horizon=30;
deltaT=.1

x0R=400*ones(4,1)
tInt=[0,deltaT];
useCasadi=0

disturbanceProfile= @(x)  23*sin(x/30)+ 27*sin(x/45-13)+  21*sin(x/15+5)+(45+45*sign(x-100))
openQNFlag=1;
injectNoise=true;
disturbancePreview=true;
spoolTime=20