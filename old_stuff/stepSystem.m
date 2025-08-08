function [xN] = stepSystem(x0,c)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%disp(x0)


P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]';
mu = [    1.0,    8.0,    4.0,    4.0   ]';
c=reshape(c,[4,1]);
x0=reshape(x0,[4,1]);
xN=(P-eye(4))*(mu.*(min(x0,c))); % P is zero on diagonal




end