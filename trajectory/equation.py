from math import atan, cos, sin, sqrt
import numpy as np
from splitfxm.derivatives import dx, d2x


class TrajectoryEquation:
    def __init__(self, rho, Cd, A, T, m, L, g):
        """
        Initialize the trajectory equation parameters.

        Parameters
        ----------
        rho : float
            Density of the fluid.
        Cd : float
            Drag coefficient.
        A : float
            Cross-sectional area.
        T : float
            Thrust force.
        m : float
            Mass of the object.
        L : float
            Length of the domain.
        g : float
            Gravitational acceleration.

        """
        self._rho = rho
        self._Cd = Cd
        self._A = A
        self._T = T
        self._L = L
        self._m = m
        self._g = g

    def residuals(self, cell_sub, scheme, scheme_opts={}):
        """
        Compute the residuals of the trajectory equation.

        Parameters
        ----------
        cell_sub : CellSub
            The cell subdomain.
        scheme : Scheme
            The scheme to use for the computation.
        scheme_opts : dict, optional
            Options for the scheme, by default {}.

        Returns
        -------
        dict
            The residuals of the trajectory equation.
        """
        # Compute the residuals using the scheme
        center_idx = len(cell_sub) // 2
        y_values = np.array([c.values()[0] for c in cell_sub])
        ypp = d2x(y_values, cell_sub, scheme)
        yp = dx(y_values, cell_sub, scheme)
        v = cell_sub[center_idx].values()[1]
        v_values = np.array([c.values()[1] for c in cell_sub])
        vp = dx(v_values, cell_sub, scheme)

        # Write the governing equation
        # Compute drag
        D = 0.5*self._rho*self._Cd*self._A*v**2
        a = atan(yp)
        vx = v*cos(a)

        # Extract the variables
        T = self._T
        m = self._m
        g = self._g

        # Compute the residuals
        eps = 1e-8
        res = np.array([0.0] * 2)
        res[0] = ypp + g * (1 + yp**2)/(v**2 + eps)
        res[1] = vp - (sqrt(1 + yp**2)/(v+eps))*(T/m - D/m) + g*yp/(v+eps)
        return res
