function [xD] = stepBVLS(x0,c)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
stateSize=4;

P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]';


mu = [    1.0,    8.0,    4.0,    4.0   ]';

Q=eye(stateSize)*2
c=reshape(c,stateSize,1)
x0=reshape(x0,stateSize,1)
op=optimoptions('quadprog','Algorithm','active-set');
s = quadprog(Q, -2*(x0-c), eye(stateSize), zeros(stateSize,1),[],[],[],[],zeros(stateSize,1),op);


s=reshape(s,stateSize,1);
tmp=mu.*(s+c);

xD=(P-eye(stateSize))*tmp;

end