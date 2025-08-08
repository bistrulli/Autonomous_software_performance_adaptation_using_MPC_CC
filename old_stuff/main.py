#%%
import numpy as np
import lcqpow
import matplotlib.pyplot as plt
import scipy
from scipy.linalg import block_diag
from scipy.integrate import solve_ivp
from qpsolvers import solve_qp

stateSize=4

P=np.array([
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
    ]).T;
mu = np.array([    1.0,    8.0,    4.0,    4.0   ]).reshape([4,1]);
print(P)



def stepSystemOptimizer(x0,c):
    Q=np.eye(stateSize)*2
    c=np.reshape(c,[stateSize,1])
    x0=np.reshape(x0,[stateSize,1])
    s = solve_qp(Q, -2*np.asarray(x0-c).T, np.eye(stateSize), np.zeros([stateSize,]), None, None, solver="osqp")


    s=np.reshape(s,[stateSize,1])
    tmp=mu*(s+c)

    xD=(P-np.eye(stateSize))@tmp
    return xD.ravel()


def KKTsystem(x0,c):
    Q=np.eye(stateSize)*2
    c=np.reshape(c,[stateSize,1])
    x0=np.reshape(x0,[stateSize,1])
    QQ=block_diag(Q,Q*0)
    gg=np.hstack([-2*np.asarray(x0-c),x0*0]);

    # Setup data of first QP.
    L = block_diag(Q,Q*0)
    R = block_diag(Q*0,Q)

    x0opt = np.zeros((stateSize*2,))

    lbB=np.hstack([-1e6*np.ones((stateSize,)),np.zeros((stateSize,))]);
    ubB=np.hstack([np.zeros((stateSize,)),1e6*np.ones((stateSize,))]);
    nV = 2
    nC = stateSize*2
    nComp = stateSize
    lcqp = lcqpow.LCQProblem(nV=stateSize*2, nC=nC, nComp=nComp)
    A=np.hstack((Q,-Q))
    eqb=2*(x0-c)

    options = lcqpow.Options()
    options.setPrintLevel(lcqpow.PrintLevel.INNER_LOOP_ITERATES)
    options.setQPSolver(lcqpow.QPSolver.QPOASES_DENSE)
    lcqp.setOptions(options)

    retVal = lcqp.loadLCQP(Q=QQ, g=gg, L=L, R=R, lb=lbB,
                           ub=ubB, lbA=eqb,ubA=eqb, A=A,x0=x0opt)


    if retVal != lcqpow.ReturnValue.SUCCESSFUL_RETURN:
        print("Failed to load LCQP.")

    retVal = lcqp.runSolver()

    if retVal != lcqpow.ReturnValue.SUCCESSFUL_RETURN:
        print("Failed to solve LCQP.")

    stats = lcqpow.OutputStatistics()
    xOpt = lcqp.getPrimalSolution()
    yOpt = lcqp.getDualSolution()
    lcqp.getOutputStatistics(stats)
    print("xOpt = ", xOpt)
    print("yOpt = ", yOpt)
    print("i = ", stats.getIterTotal())
    print("k = ", stats.getIterOuter())
    print("rho = ", stats.getRhoOpt())
    print("WSR = ", stats.getSubproblemIter())
    pass




def stepSystem(x0,c):
    c=np.reshape(c,[stateSize,1])
    x0=np.reshape(x0,[stateSize,1])
    tmp=mu*np.minimum(x0,c)
    xD=(P-np.eye(stateSize))@tmp;
    return xD.ravel()




newC=np.random.randint(1,16,[stateSize,1])*0+9
x0R=np.random.uniform(0,100,[stateSize,1])*0+100
xDopt=stepSystemOptimizer(x0R,newC)
xDstd=stepSystem(x0R,newC)

print('---')
print(xDopt,xDstd)
print(xDopt-xDstd)
print('---')
#%%
x0R=np.random.uniform(0,100,[stateSize,1])*0+100
x0RO=x0R*1
tInt=np.array([0,3])
f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
for i in range(0,30):
    # x0R=np.random.uniform(0,100,[stateSize,1])*0+100
    # x0RO=x0R*1
    newC=np.random.randint(1,16,[4,1])
    newC[0,0]=1000;
    odeFunOpt= lambda t,y: stepSystemOptimizer(y,newC)
    odeFun= lambda t,y: stepSystem(y,newC)

    rkSTD=solve_ivp(odeFun,tInt,x0R.ravel(),method='RK45',vectorized=True)
    rkOpt=solve_ivp(odeFunOpt,tInt,x0RO.ravel(),method='RK45',vectorized=True)
    ax1.plot(rkSTD.t,rkSTD.y.T,'o-')
    print(rkOpt.y.shape)

    print(rkSTD.y.T[-1])
    print(rkOpt.y.T[-1])
    x0R=rkSTD.y.T[-1,:].T
    x0RO=rkOpt.y.T[-1,:].T
    print("-1")
    print(x0R)
    print(x0RO)
    # print(sum(x0R-x0RO))
    print("#")

    #print(rkSTD.y[0])
    ax2.plot(rkOpt.t,rkOpt.y.T,'o-')
    tInt+=3

#plt.show()

# %%

KKTsystem(x0R,newC)
# %%
