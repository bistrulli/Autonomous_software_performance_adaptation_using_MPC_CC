function [xN] = stepQNSystem(x0,cores,P,mu,B)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%disp(x0)
if nargin <=4
    B=[0; 1 ; 0; 0];
end
if numel(cores)>size(P,1)% disturbance for state two
    cores=reshape(cores,[5,1]);
    w=cores(end,1);
    cores=cores(1:4,:);
else
    w=0;
end
cores=reshape(cores,[4,1]);
x0=reshape(x0,[4,1]);
xN=(P-eye(4))*(mu.*(min(x0,cores)))+ B*w; % P is zero on diagonal




end