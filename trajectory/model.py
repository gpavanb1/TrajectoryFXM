from splitfxm.model import Model
from .equation import TrajectoryEquation


class TrajectoryModel(Model):
    def __init__(self, rho=1.0, Cd=0.3, A=1.0e-2, T=1e5, m=100.0, L=1000, g=9.81):
        self._equation = TrajectoryEquation(rho, Cd, A, T, m, L, g)
