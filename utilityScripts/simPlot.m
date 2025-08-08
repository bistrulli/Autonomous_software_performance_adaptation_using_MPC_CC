close all
figure
hold on
tic
for kSim=1:200   
    if useCasadi==0
        cOpt=optim(x0R);
    elseif useCasadi>=3
        x0init=result_active_set.x; % use old solution
        solver_initalization = struct('x0', x0init,'lbx',lbx,'ubx',ubx,'lbg',lbg,'ubg',ubg,'p0',x0R);
        [result_active_set,stats_active_set] = mpec_optimizer(mpec, solver_initalization, solver_settings);
        cOpt=result_active_set.x(1:4);
    else
        
        opti.set_value(x0para, x0R);
        try
            sol = opti.solve();
            cOpt=opti.value(c);
        catch
            cOpt=opti.debug.value(c);
            
        end       

    end
    odeFun = @(t,x0 ) stepQNSystem(x0,[cOpt(:,1)]);
    [tff,x0ff]=ode45(odeFun,tInt,x0R,[]);
    x0R=x0ff(end,:)'
    plot(tff,x0ff);
    tInt=tInt+diff(tInt);
end

toc