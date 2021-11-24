from time import perf_counter

from bioptim import ControlType, SolutionIntegrator, OptimalControlProgram

from JumperOcp import JumperOcp, Jumper, OdeSolver

if __name__ == "__main__":
    root_path_model = "/".join(__file__.split("/")[:-1])
    jumper_model = Jumper(root_path_model + "/models/")
    ode_solver = OdeSolver.COLLOCATION(method="legendre", polynomial_degree=4)
    # ode_solver = OdeSolver.RK8(n_integration_steps=3)
    # jumper = JumperOcp(jumper=jumper_model, control_type=ControlType.CONSTANT, ode_solver=ode_solver)

    #
    # print(jumper.x_init[0])
    # from bioptim import Solution, InitialGuess
    # sol = Solution(jumper.ocp, [jumper.x_init[0], jumper.u_init[0]])
    # sol.integrate(integrator=SolutionIntegrator.SCIPY_DOP853).animate(show_meshes=True)
    # m = sol.ocp.nlp[0].model

    # for i in range(self.model.nbSoftContacts()):
    #     c = self.model.softContact(i)
    #     c = m.softContact(0)
    #     c.computeForce(x, dx, a)
    # biorbd.SoftContactSphere(c)

    jumper_model = Jumper(root_path_model + "/models/")
    jumper = JumperOcp(jumper=jumper_model, control_type=ControlType.CONSTANT, ode_solver=ode_solver, update_obj=True)
    tic = perf_counter()
    sol = jumper.solve(limit_memory_max_iter=0, exact_max_iter=1, force_no_graph=True)
    print(f"Time to solve : {perf_counter() - tic}sec")
    #

    sol.print()
    sol.graphs()
    sol.animate(show_meshes=False)
    jumper.ocp.save(sol, "test.bo")
