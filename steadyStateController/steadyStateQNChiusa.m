

addpath("../utilityScripts/")
addpath("../")
closedQNwithNoDelaySystemInit

cores=sdpvar(4,1);
x0=sdpvar(4,1);
xDot=sdpvar(4,1);
q_size=sdpvar(1,1);
x_inst=intvar(3,1); %status of instances

vecCost=[0.02,0.04,0.08];


constrO=[ x_inst>=1; x_inst<=10;
            cores>=1;
            cores(1)==10000; 

          (P-eye(4))*(mu.*min(x0,cores))==xDot;
          xDot==0;

          x0>=0; sum(x0)==q_size
          [0,1,1,1]*cores<=[2,4,8]*x_inst
          ];


%x_instm, x0 are positive
cost=sum(abs([1,0,0,0]*x0-q_size))+sum(vecCost*x_inst)/1000;

optimize([constrO; q_size==100],cost,sdpsettings('solver','gurobi','debug',1))
%%
steadyState_controller=optimizer(constrO,cost,sdpsettings('solver','gurobi'),[q_size],[vec(cores);x_inst]);
stepPlotInstancesSS

title('Steady state solver')


hold on


