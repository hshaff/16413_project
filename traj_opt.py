from pydrake.solvers import MathematicalProgram, Solve
from pydrake.common.containers import EqualToDict
import numpy as np

prog = MathematicalProgram()
T = 3
x0 = prog.NewContinuousVariables(7, 'x0')
x1 = prog.NewContinuousVariables(7, 'x1')
x2 = prog.NewContinuousVariables(7, 'x2')
x3 = prog.NewContinuousVariables(7, 'x3')

init_config = (0.5748382822262621, 0.9783629810748132, 0.008609035007501997, -1.306724006830466, -1.2430831908952924, 2.7181811822310156, 2.369339829267124) #(0.01200158428400755, -0.5697816014289856, 5.6801487517077476e-05, -2.8105969429016113, -0.00025768374325707555, 3.0363450050354004, 0.7410701513290405)
final_config = (-1.814210113567768, 1.7436845852216114, 1.3413095650110507, -1.0837430500575262, -0.8884820180071649, 2.364136535213516, -0.374817962722338)#(-0.40501277339102076, 1.5350324087458784, 0.3849950132630586, -0.6654749503105135, -0.014910767633732114, 2.9147031381461224, 0.2058514103986795)

for i in range(7):
    # prog.AddQuadraticCost(
    #     np.sqrt((x1[i] - x0[i])**2)
    # )
    # prog.AddQuadraticCost(
    #     np.sqrt((x2[i] - x1[i])**2)
    # )
    # prog.AddQuadraticCost(
    #     np.sqrt((x3[i] - x2[i])**2)
    # )
    prog.AddLinearConstraint(x0[i] == init_config[i])
    prog.AddLinearConstraint(x3[i] == final_config[i])


# prog.AddQuadraticErrorCost(np.identity(len(x1)), x0, x1)
# prog.AddQuadraticErrorCost(np.identity(len(x2)), x1, x2)
# prog.AddQuadraticErrorCost(np.identity(len(x3)), x2, x3)

prog.AddCost(np.sqrt((x0-x1).dot(x0-x1)))
prog.AddCost(np.sqrt((x3-x1).dot(x3-x1)))

# prog.AddCost(np.sqrt((x3-x2).dot(x3-x2)))
#prog.AddCost((x2-x3).dot(x2-x3))


def bezier(t, p0, p1, p2):
    return (1-t)**2*p0 + 2*(1-t)*t*p1 + t**2*p2

result = Solve(prog, np.ones(28))
infeas = result.GetInfeasibleConstraints(prog)
for c in infeas:
    print(c)
print(result.GetSolution(x0))
p0 = result.GetSolution(x0)
print(result.GetSolution(x1))
p1 = result.GetSolution(x1)
print(result.GetSolution(x2))
p2 = result.GetSolution(x3)
print(result.GetSolution(x3))

def bezier_path(T):
    traj = []
    for i in range(T):
        t = i/T
        traj.append(bezier(t, p0, p1,p2))
    return traj
    

print(bezier_path(100))

# dynamics_constraint = bezier


