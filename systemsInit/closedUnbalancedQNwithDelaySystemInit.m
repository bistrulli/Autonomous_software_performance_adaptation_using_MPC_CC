clear all
stateSize=4
P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]';


mu = [    1.0,    8.0,    4.0,    4.0   ]';

stepQNSystem([],[],P,mu);
horizon=45;
deltaT=0.1

x0R=25*ones(4,1)
tInt=[0,.1];
useCasadi=0
openQNFlag=0;
disturbanceProfile= @(x)  23*sin(x/30)+ 27*sin(x/45-13)+  21*sin(x/15+5)+(45+45*sign(x-100))
spoolTime=30
lenSim=400
injectNoise=false;
disturbancePreview=false;