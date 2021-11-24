from time import perf_counter
import numpy as np
from bioptim import ControlType, SolutionIntegrator, OptimalControlProgram, InterpolationType

from JumperOcp import JumperOcp, Jumper, OdeSolver

if __name__ == "__main__":
    root_path_model = "/".join(__file__.split("/")[:-1])
    jumper_model = Jumper(root_path_model + "/models/")
    # ode_solver = OdeSolver.COLLOCATION(method='legendre', polynomial_degree=9)
    ode_solver = OdeSolver.RK8(n_integration_steps=3)
    jumper = JumperOcp(jumper=jumper_model, control_type=ControlType.CONSTANT, ode_solver=ode_solver)
    #
    # tic = perf_counter()
    #
    from bioptim import Solution, InitialGuess
    X = jumper.x_init[0]

    n = jumper_model.n_shoot

    a = np.array([0, 0, 0, -190, 260, -50])
    a = a[:, np.newaxis]
    u0 = np.repeat(a, repeats=1 * n / 5, axis=1)
    b = np.array([0, 0, 0, 0, 0, 0])
    b = b[:, np.newaxis]
    u1 = np.repeat(b, repeats=4 * n / 5 + 1, axis=1)
    U = np.concatenate([u0, u1], axis=1)
    U = InitialGuess(U, interpolation=InterpolationType.EACH_FRAME)

    sol = Solution(jumper.ocp, [jumper.x_init[0], U])
    sol.integrate().animate(show_meshes=False)
    # sol.integrate(integrator=SolutionIntegrator.SCIPY_LSODA).animate(show_meshes=False)


