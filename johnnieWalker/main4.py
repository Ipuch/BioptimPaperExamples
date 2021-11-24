from time import perf_counter

from bioptim import ControlType, SolutionIntegrator, OptimalControlProgram, Solution, InitialGuess

from JumperOcp import JumperOcp, Jumper, OdeSolver


if __name__ == "__main__":
    root_path_model = "/".join(__file__.split("/")[:-1])
    jumper_model = Jumper(root_path_model + "/models/", time=0.35)
    # ode_solver = OdeSolver.COLLOCATION(method="legendre", polynomial_degree=4)
    ode_solver = OdeSolver.RK8()

    jumper = JumperOcp(jumper=jumper_model, control_type=ControlType.CONSTANT, ode_solver=ode_solver)
    sol = Solution(jumper.ocp, [jumper.x_init, jumper.u_init])
    # sol.integrate(integrator=SolutionIntegrator.SCIPY_DOP853).animate(show_meshes=True)
    sol.integrate(integrator=SolutionIntegrator.SCIPY_DOP853)

    jumper_model = Jumper(root_path_model + "/models/", time=0.35)
    jumper = JumperOcp(jumper=jumper_model,
                       control_type=ControlType.CONSTANT,
                       ode_solver=ode_solver,
                       update_obj=True,
                       X0=sol.states['all'])
    tic = perf_counter()
    sol = jumper.solve(limit_memory_max_iter=10, exact_max_iter=2000, force_no_graph=False)
    print(f"Time to solve : {perf_counter() - tic}sec")
    #

    sol.print()
    # sol.graphs()
    sol.animate(show_meshes=False)
    jumper.ocp.save(sol, "test.bo")
