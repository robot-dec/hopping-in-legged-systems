from math import sin, cos
import numpy as np

from consts import Constants as c


class State:
    STANCE = "touching the ground"
    FLIGHT = "in the air"


class Dyanmics:
    def __init__(self, thrust_setpoint, attitude_setpoint):
        # Control setpoints
        self.zeta_d = thrust_setpoint
        self.theta1_d = attitude_setpoint
        self.zeta_init = 0.0

        # Initial conditions
        self.xtd = 0.0
        self.t_switch = 0.0
        self.state = State.FLIGHT  # Assume start in flight. Control will quickly update otherwise
        self.max_zeta = self.zeta_init  # Internal state for leg extension
        self.debounce_count = 0

    def debounce(self, input):
        if (input):
            self.debounce_count += 1
            if (self.debounce_count > 100):
                return True
        else:
            self.debounce_count = 0
        return False

    def update_state(self, t, y):
        if self.debounce((self.state == State.FLIGHT) and (y[3] <= 0)):
            self.state = State.STANCE
            self.t_switch = t
            self.xtd = y[2]
        if (self.state == State.STANCE) and (y[3] > 0):
            self.state = State.FLIGHT
            self.t_switch = t

    def update_control(self, t, y):
        # PD control for leg angle
        tau = c.Kp(y[3]) * (y[0] - self.theta1_d) + c.Kv(y[3]) * y[5]

        # Bang-bang control of leg length for hopping height
        if (self.state == State.STANCE):
            zeta = min(
                self.zeta_init + c.k * pow(t - self.t_switch, 2), self.zeta_d)
            self.max_zeta = zeta
        elif (self.state == State.FLIGHT):
            zeta = max(
                self.max_zeta - c.k * pow(t - self.t_switch, 2), self.zeta_init)
        zeta = self.zeta_init

        return tau, zeta

    def step(self, t, x):
        # State variables are: x = [
        #  Leg angle (theta1)
        #  Hip angle (theta2)
        #  Foot translation (x0)
        #  Foot height (y0)
        #  Leg length (w)
        #  Leg rotational velocity
        #  Hip rotational velocity
        #  Foot velocity in x
        #  Foot velocity in y
        #  Rate of change of leg length
        # ]

        self.update_state(t, x)

        tau, zeta = self.update_control(t, x)

        # Calculate distance between hip joint and leg CoG
        W = x[4] - c.r1
        # Calculate external forces
        if x[3] < 0.0:
            Fx = -1 * c.KG * (x[2] - self.xtd) - c.BG * x[7]
        else:
            Fx = 0.0
        if x[3] < 0.0:
            Fy = -1 * c.KG * x[3] - c.BG * x[8]
        else:
            Fy = 0.0
        if (c.k0 - x[4] + zeta) > 0.0:
            Fk = c.KL * (c.k0 - x[4] + zeta)
        else:
            Fk = c.Kl2 * (c.k0 - x[4] + zeta) - c.Bl2 * x[9]

        lhs = np.array([
                [
                    cos(x[0]) * (c.M2 * W * x[4] + c.I1),
                    c.M2 * c.r2 * cos(x[1]),
                    c.M2 * W,
                    0,
                    c.M2 * W * sin(x[0])
                ],
                [
                    -1 * sin(x[0]) * (c.M2 * W * x[4] + c.I1),
                    -1 * c.M2 * c.r2 * sin(x[1]),
                    0,
                    c.M2 * W,
                    c.M2 * W * cos(x[0])
                ],
                [
                    cos(x[0]) * (c.M1 * c.r1 * W - c.I1),
                    0,
                    c.M1 * W,
                    0,
                    0
                ],
                [
                    -1 * sin(x[0]) * (c.M1 * c.r1 * W - c.I1),
                    0,
                    0,
                    c.M1 * W,
                    0
                ],
                [
                    -1 * cos(x[1] - x[0]) * c.I1 * c.r2,
                    c.I1 * x[4],
                    0,
                    0,
                    0
                ]
            ]
        )

        rhs = np.array([
            W * c.M2 * (pow(x[5], 2) * W * sin(x[0]) - 2 * x[5] * x[9] * cos(x[0]))
            + c.r2 * pow(x[6], 2) * sin(x[1]) + c.r1 * pow(x[5], 2) * sin(x[0])
            - c.r1 * Fx * pow(cos(x[0]), 2) + cos(x[1]) * (c.r1 * Fy * sin(x[1])
                                                           - tau)
            + Fk * W * sin(x[1]),
            W * c.M2 * (pow(x[5], 2) * W * cos(x[1]) + 2 * x[5] * x[9] * sin(x[0])
                        + c.r2 * pow(x[6], 2) * cos(x[1])
                        + c.r1 * pow(x[6], 2) * cos(x[0]) - c.g)
            + c.r1 * Fx * cos(x[0]) * sin(x[0]) - sin(x[0]) *
            (c.r1 * Fy * sin(x[0]) - tau) + Fk * W * cos(x[0]),
            W * (c.M1 * c.r1 * pow(x[5], 2) * sin(x[0]) - Fk * sin(x[0]) + Fx)
            - cos(x[0]) * (Fy * c.r1 * sin(x[0]) 
                           - Fx * c.r1 * cos(x[0]) - tau),
            W * (c.M1 * c.r1 * pow(x[5], 2) * cos(x[0]) - Fk * cos(x[0]) + Fy
                 - c.M1 * c.g)
            - sin(x[0]) * (Fy * c.r1 * sin(x[0]) - Fx * c.r1 * cos(x[0])
                           - tau),
            W * (Fk * c.r2 * sin(x[1] - x[0]) + tau) - c.r1 * cos(x[1] - x[0])
            * (c.r1 * Fy * sin(x[0]) - c.r1 * Fx * cos(x[0]) - tau)
            ])

        return np.concatenate((np.array((x[5], x[6], x[7], x[8], x[9])),
                               np.linalg.solve(lhs, rhs)))

    @staticmethod
    def calculate_output_variables(t, x):
        # Output variables are, y = [
        #  Hip translation (x1)
        #  Hip height (y1)
        #  Body translation (x2)
        #  Body height (y2)
        # ]
        sin_theta_1 = np.sin(x[0])
        cos_theta_1 = np.cos(x[0])

        x1 = x[2] + sin_theta_1 * c.r1
        y1 = x[3] + cos_theta_1 * c.r1
        x2 = x[2] + sin_theta_1 * x[4] + np.sin(x[1]) * c.r2
        y2 = x[3] + cos_theta_1 * x[4] + np.cos(x[1]) * c.r2

        return np.array([x1, y1, x2, y2])

