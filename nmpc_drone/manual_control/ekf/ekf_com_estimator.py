import numpy as np

class EkfComEstimator():
    def __init__(self, DynParam):
        J = DynParam['J']
        self.J_inv = np.diag([1/J(0,0), 1/J(1,1), 1/J(2,2), 1/J(3,3)])
        Rt = 0.01**2 * np.eye((10,))