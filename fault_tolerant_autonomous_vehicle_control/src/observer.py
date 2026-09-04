import numpy as np
from scipy.signal import place_poles

class LuenbergerObserver:
    def __init__(self,Ad,Bd,C):
        self.Ad=Ad; self.Bd=Bd; self.C=C
        self.L=place_poles(Ad.T,C.T,[0.35,0.45]).gain_matrix.T
        self.xhat=np.zeros(2)

    def update(self,u,y):
        innovation=np.asarray(y)-self.C@self.xhat
        self.xhat=self.Ad@self.xhat+self.Bd[:,0]*u+self.L@innovation
        return self.xhat.copy(),innovation.copy()
