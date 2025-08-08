clear all
getConstantsIntoWorkspace
useCasadi=3
import casadi.*


c=SX.sym('c',4,horizon);
x0=SX.sym('x0',4,horizon);
x0para=SX.sym('x0para',4);
xDot=SX.sym('xDot',4,horizon);
s=SX.sym('s',4,horizon);
lambda=SX.sym('lambda',4,horizon);

% parameters
p = [x0para];
x = [c(:);x0(:);xDot(:);s(:);lambda(:)];
trackError=x0(2:4,2:end)-75;
f = sum(abs(trackError(:)))%+0.01*sumsqr(c(:));
lbx = zeros(size(x))-inf;
ubx = zeros(size(x))+inf;
mask=setdiff(1:numel(c),1:4:numel(c));
lbx(mask)=1;
ubx(mask)=16;
lbx(1:4:numel(c))=1000;
ubx(1:4:numel(c))=1000;

t=numel(c)+1;
lbx(t:t+numel(x0(:))-1)=0;
ubx(t:t+numel(x0(:))-1)=4000;
t=t+numel(x0(:))+numel(xDot(:));
lbx(t:t+numel(s)-1)=-4000;
ubx(t:t+numel(s)-1)=0;
t=t+numel(s)
lbx(t:t+numel(lambda)-1)=0;
ubx(t:t+numel(lambda)-1)=inf;

g=[
    
    x0(:,1)-x0para;
]
lbg = [
    zeros(numel(x0(:,1)),1);];

ubg = [
    zeros(numel(x0(:,1)),1);
    ];
G=[]
H=[]


Q=eye(stateSize)*2;
gap=x0-c;

for tIdx=1:horizon
    e=-2*gap(:,tIdx);
    stationariety=Q*s(:,tIdx)+e+eye(stateSize)'*lambda(:,tIdx);
    g=[g;stationariety(:);];
    lbg=[lbg; zeros(stateSize,1);];
    ubg=[ubg; zeros(stateSize,1);];
    G=[G;lambda(:,tIdx)]
    H=[H;(-s(:,tIdx))]
    %g=[g;stationariety(:);lambda(:,tIdx).*(-s(:,tIdx))];
    %lbg=[lbg; zeros(stateSize,1);zeros(stateSize,1)];
    %ubg=[ubg; zeros(stateSize,1);1e-2+zeros(stateSize,1) ];
end
for tIdx=1:horizon-1
    xDot(:,tIdx)=(P-eye(4))*(mu.*(s(:,tIdx)+c(:,tIdx)));   
    g=[g;x0(:,tIdx+1)-(x0(:,tIdx)+deltaT*xDot(:,tIdx))]
    ubg=[ubg; zeros(stateSize,1); ];
    lbg=[lbg; zeros(stateSize,1); ];

end


%nlp = struct('x',x, 'p',p,'f',f, 'g',g);


%'lbg',lbg,'ubg',ubg
%solver = nlpsol('solver', 'ipopt', nlp,struct())

%solver('lbg',lbg,'ubg',ubg,'p',100*ones(4,1))

%G=Function('G',{x},{G})
%H=Function('H',{x},{H})

mpec = struct('x', x,'f',f,'g',g,'G',G,'H',H,'p',p);
solver_initalization = struct('x0', 1+zeros(size(x)),'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg,'p0',100*ones(4,1));
settings = HomotopySolverOptions();
solver_settings.settings_lpec.lpec_solver ="Gurobi";
[result_homotopy,stats_homotopy] = mpec_homotopy_solver(mpec,solver_initalization,settings);
f_opt_homotopy = full(result_homotopy.f);
w_opt_homotopy = full(result_homotopy.x);
%%  Settings
solver_settings = MPECOptimizerOptions();
solver_settings.settings_lpec.lpec_solver ="Gurobi";
%solver_settings.initalization_strategy = "TakeInitialGuessDirectly";
 solver_settings.initalization_strategy = "RelaxAndProject";
solver_settings.consider_all_complementarities_in_lpec = true;
solver_settings.tol_B_stationarity = 1e-8;
solver_settings.relax_and_project_iters = 2;
solver_settings.relax_and_project_kappa = 0.5;
solver_settings.relax_and_project_sigma0 = 0.01;
solver_settings.plot_lpec_iterate = 0;
%solver_settings.settings_lpec.lpec_solver = 'Ell_inf';

%solver_initalization = struct('x0', x0, 'lbx',lbx, 'ubx',ubx,'lbg',lbg, 'ubg',ubg);
tic
[result_active_set,stats_active_set] = mpec_optimizer(mpec, solver_initalization, solver_settings);
toc
w_opt_active_set = full(result_active_set.x);
f_opt_active_set = full(result_active_set.f);
fprintf('\n-------------------------------------------------------------------------------\n');
fprintf('Method \t\t Objective \t comp_res \t n_biactive \t CPU time (s)\t Sucess\t Stat. type\n')
fprintf('-------------------------------------------------------------------------------\n');
fprintf('homotopy \t %2.2e \t %2.2e \t\t %d \t\t\t %2.2f \t\t\t\t %d\t %s\n',f_opt_homotopy,stats_homotopy.comp_res,stats_homotopy.n_biactive,stats_homotopy.cpu_time_total,stats_homotopy.success,stats_homotopy.multiplier_based_stationarity)
fprintf('Active Set \t %2.2e \t %2.2e \t\t %d \t\t\t %2.2f \t\t\t\t %d\t %s\n',f_opt_active_set,stats_active_set.comp_res,stats_active_set.n_biactive,stats_active_set.cpu_time_total,stats_active_set.success,stats_active_set.multiplier_based_stationarity)
fprintf('-------------------------------------------------------------------------------\n');
fprintf(' || w_homotopy - w_active_set || = %2.2e \n',norm(w_opt_homotopy-w_opt_active_set));
fprintf('solution is (%2.4f,%2.4f) \n',w_opt_active_set(1),w_opt_active_set(2));

