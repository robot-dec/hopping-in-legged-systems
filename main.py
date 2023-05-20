from scipy.integrate import solve_ivp
from math import sin, cos, pi, sqrt
import numpy as np

from consts import Constants as c
from plot import plot_hopping
from animate import animate_solution
from dynamics import Dyanmics


def run():
    t_span = [
        0,      # initial time
        10      # final time
    ]
    y0 = [
        0.0,
        0.0,
        0.0,
        0.5,
        c.k0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0
    ]

    t_eval = np.linspace(t_span[0], t_span[1], 1000)

    dynamics = Dyanmics(thrust_setpoint=0.03, attitude_setpoint=0.0)

    res = solve_ivp(
        fun=dynamics.step,
        t_span=t_span,
        y0=y0,
        method="RK45",
        t_eval=t_eval
    )

    print("stance interval = {}".format(pi * sqrt(c.M2 / c.KL)))
    print("for H = {}, hopping cycle = {}".format(
        c.H, pi * sqrt(c.M2 / c.KL) + sqrt(8 * c.H / c.g)))

    y  = dynamics.calculate_output_variables(res.t, res.y)

    plot_hopping(res.t, res.y, y, "Passive dynamics")

    animate_solution(res.t, res.y, y)


if __name__ == "__main__":
    run()
