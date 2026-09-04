import numpy as np
from scipy.linalg import solve_discrete_are

class LQR:
    def __init__(self,Ad,Bd):
        Q=np.diag([10.0,3.0]); R=np.array([[0.8]])
        P=solve_discrete_are(Ad,Bd,Q,R)
        self.K=np.linalg.solve(R+Bd.T@P@Bd,Bd.T@P@Ad)

    def command(self,x,ref):
        e=np.asarray(x)-np.asarray(ref)
        return float(np.clip(-(self.K@e)[0],-1.0,1.0))


class FaultTolerantLQR:
    def __init__(self,nominal,minimum_effectiveness=0.35):
        self.nominal=nominal
        self.estimated_effectiveness=1.0
        self.minimum_effectiveness=minimum_effectiveness

    def set_effectiveness(self,value):
        self.estimated_effectiveness=float(np.clip(value,self.minimum_effectiveness,1.0))

    def command(self,x,ref):
        u=self.nominal.command(x,ref)
        return float(np.clip(u/self.estimated_effectiveness,-1.0,1.0))
