function [stepOpt,kktInner] = stepSystemOptimizer()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

kktInner=0;
P = [
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]';


mu = [    1.0,    8.0,    4.0,    4.0   ]';



c=sdpvar(4,1);
x0=sdpvar(4,1);
s=sdpvar(4,1);
obj=sum((s-(x0-c)).^2);
constr=[s<=0];
xN=-mu.*(s+c);
xN=xN+P*(mu.*(s+c)); % P is zero on diagonal
in=[x0;c];
%stepOpt=optimizer(constr,obj,[],in,[xN;s;s-(x0-c)]);
stepOpt=optimizer(constr,obj,sdpsettings('solver','gurobi'),in,[xN]);




end