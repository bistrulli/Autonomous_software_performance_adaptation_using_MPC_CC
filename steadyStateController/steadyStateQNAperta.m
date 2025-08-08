%Questo test è interessante perchè la simulazione converge su un punto di
%equilibro diverso da quello trovato da MPC

addpath("../utilityScripts/")
addpath("../")
openQNwithDelaySystemInit
cores=sdpvar(4,1);
x0=sdpvar(4,1);
xDot=sdpvar(4,1);
distPreview=sdpvar(1,1);
vecCost=[0.02,0.04,0.08];

x_inst=intvar(3,1); %status of instances
constrO=[ x_inst>=1; cores>=1;
          x_inst<=18;  cores(1)==1000; 
          (P-eye(4))*(mu.*min(x0,cores))+[0;distPreview;0;0]==xDot;
          xDot==0;  x0>=0;
          sum(cores(2:4,1))<=[2,4,8]*x_inst];

%%
%x_instm, x0 are positive
trackError=sum(([0,1,1,0]*x0-275).^2)+sum(([0,1,0,1]*x0-275).^2)+sum(([1,0,0,0]*x0).^2)+sum(vecCost*x_inst)/1000;
cost=trackError;

optimize([constrO; distPreview==50],cost,sdpsettings('solver','mosek','debug',1))
%%
optim=optimizer(constrO,cost,[],[distPreview;],[vec(cores); x_inst;x0]);
stepPlotInstancesSS

title('Steady state solver')


hold on


