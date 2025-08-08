clear all
stateSize=4
P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]';


mu = [    1.0,    8.0,    4.0,    4.0   ]';


horizon=35;
deltaT=0.1

x0R=100*ones(4,1)
tInt=[0,.1];
useCasadi=0