clear all
stateSize=4
P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.5, 0.5],
    [1, 0.0, 0, 0.0],
    [1, 0.0, 0, 0.0]
    ]';


mu = [    1.0,    8.0,    4.0,    4.0   ]';

stepQNSystem([],[],P,mu);
horizon=45;
deltaT=0.1
steadyState_controllerFronzenVMs=[];
x0R=100*ones(4,1)
tInt=[0,.1];
useCasadi=0
openQNFlag=0;
disturbanceProfile= @(x) [];
spoolTime=30
lenSim=400
injectNoise=false;
disturbancePreview=false;