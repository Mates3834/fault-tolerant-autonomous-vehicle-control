import numpy as np
from scipy.linalg import expm

class LinearVehicle:
    def __init__(self, dt=0.02):
        self.dt=dt
        # Generic stable lateral/yaw-like dynamics.
        self.A=np.array([[-0.65, 1.0],[-1.35,-0.8]],dtype=float)
        self.B=np.array([[0.45],[1.25]],dtype=float)
        self.C=np.eye(2)
        M=np.block([[self.A,self.B],[np.zeros((1,3))]])
        Md=expm(M*dt)
        self.Ad=Md[:2,:2]; self.Bd=Md[:2,2:3]
        self.x=np.zeros(2)

    def step(self,u,effectiveness=1.0,disturbance=None):
        disturbance=np.zeros(2) if disturbance is None else np.asarray(disturbance)
        self.x=self.Ad@self.x+self.Bd[:,0]*(effectiveness*u)+disturbance
        return self.x.copy()
