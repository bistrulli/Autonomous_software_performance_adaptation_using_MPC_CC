import sys
import os

print(str(sys.path))

dir_path = os.path.dirname(os.path.realpath(__file__))
print("current working dir: %s" % dir_path)

sys.path.insert(0, "/home/wizard1993/dist/bin/")

from pyomo.environ import *
from pyomo.gdp import *
import numpy as np

# Problem parameters
stateSize = 4
horizon = 150
deltaT = 0.1
P = np.array([
    [0, 1.0, 0.0, 0.0],
    [0.0, 0, 0.6, 0.4],
    [0.7, 0.3, 0, 0.0],
    [0.8, 0.2, 0, 0.0]
]).T
mu = np.array([1.0, 8.0, 4.0, 4.0])
I= np.eye(stateSize);
# Create Pyomo model
model = ConcreteModel()

# Define sets
model.t = RangeSet(1, horizon)
model.t_minus_1 = RangeSet(1, horizon-1)
model.i = RangeSet(1, stateSize)

# Define variables
model.c = Var(model.i, model.t, domain=NonNegativeReals)
model.x0 = Var(model.i, model.t, domain=NonNegativeReals, bounds=(0, 4000))
model.xDot = Var(model.i, model.t_minus_1)
model.slack = Var(model.i, model.t, domain=NonNegativeReals)
model.s = Var(model.i, model.t,bounds=(-4000, 0))
model.lambda_ = Var(model.i, model.t, domain=NonNegativeReals, bounds=(0, 40000))


# Constraints
model.constraints = ConstraintList()

# c(1,:) == 1000
for t in model.t:
    model.constraints.add(model.c[1, t] == 1000)


# c(2:4,:) <= 16
for t in model.t:
    for i in range(2, stateSize+1):
        model.constraints.add(model.c[i, t] <= 16)
        model.constraints.add(model.c[i, t] >= 1)

for t in model.t:
    for i in range(1, stateSize+1):
        model.constraints.add(model.x0[i, t] -75<= model.slack[i, t])
        model.constraints.add(model.x0[i, t] -75>= -model.slack[i, t])


# Dynamic constraints
for t in model.t_minus_1:
    for i in model.i:
        xDot_expr = sum((P-I)[i-1, j-1] * mu[j-1] * (model.s[j, t] + model.c[j, t]) for j in model.i)
        model.constraints.add(model.xDot[i, t] == xDot_expr)
        model.constraints.add(model.x0[i, t+1] == model.x0[i, t] + deltaT * model.xDot[i, t])

# Complementarity constraints
for t in model.t:
    for i in model.i:
        model.constraints.add(2 * model.s[i, t] + (2 * (model.x0[i, t] - model.c[i, t])) + model.lambda_[i, t] == 0)
        d=Disjunction(expr= [(model.lambda_[i, t] >= 0), (model.s[i, t] <= 0)])  # Complementarity condition
        model.add_component("cc{}o{}".format(i,t),d)

# Objective function
model.obj = Objective(
    expr=sum(model.slack[i, t] for i in model.i for t in model.t),
    sense=minimize
)


print(model)
solver = SolverFactory('cbc')
# Solver
TransformationFactory('gdp.cuttingplane').apply_to(model)
#solver = SolverFactory('ipopt',executable="/home/wizard1993/dist/bin/ipopt")
#solver = SolverFactory('gdpopt.loa')

for i in range(0,stateSize):
    model.x0[1+i, 1].fix(100)
solver.solve(model, tee=True)



for i in range(0,stateSize):
    model.x0[1+i, 1].fix(200)

#solver.solve(model, tee=True,mip_solver="cbc")
a=solver.solve(model, tee=False)
print(a)