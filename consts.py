class Constants:
    M1 = 1.0
    M2 = 10.0
    I1 = 1.0
    I2 = 10.0
    r1 = 0.5
    r2 = 0.4
    k0 = 1.0
    KL = pow(10.0, 3)
    Kl2 = pow(10.0, 5)
    Bl2 = 175.0
    KG = pow(10.0, 5)
    BG = 300.0
    H = 0.35
    g = 9.81

    k = 5.0

    @staticmethod
    def Kp(y0):
        if y0 <= 0:
            return 1800
        else:
            return 1200

    @staticmethod
    def Kv(y0):
        if y0 <= 0:
            return 200
        else:
            return 60
