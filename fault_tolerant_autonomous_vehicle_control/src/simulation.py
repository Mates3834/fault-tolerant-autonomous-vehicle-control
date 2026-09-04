import numpy as np
from .model import LinearVehicle
from .controllers import LQR,FaultTolerantLQR
from .observer import LuenbergerObserver
from .faults import actuator_effectiveness,sensor_bias
from .detector import ResidualDetector

def run(fault_tolerant=False,duration=16.0,dt=0.02,seed=4):
    rng=np.random.default_rng(seed)
    plant=LinearVehicle(dt)
    nominal=LQR(plant.Ad,plant.Bd)
    ctrl=FaultTolerantLQR(nominal)
    obs=LuenbergerObserver(plant.Ad,plant.Bd,plant.C)
    detector=ResidualDetector()

    ts=[]; xs=[]; hats=[]; us=[]; rs=[]; flags=[]; refs=[]
    for k in range(int(duration/dt)):
        t=k*dt
        ref=np.array([0.45 if t>1 else 0.0,0.0])
        u=ctrl.command(obs.xhat,ref) if fault_tolerant else nominal.command(obs.xhat,ref)
        eff=actuator_effectiveness(t)
        x=plant.step(u,effectiveness=eff)
        y=x+sensor_bias(t)+rng.normal(0,0.01,2)
        xhat,res=obs.update(u,y)
        detected,score=detector.update(res)

        # Generic accommodation after residual detection. In this synthetic
        # experiment the diagnosed actuator-loss level is supplied by the
        # scenario model; this is not an online fault-parameter estimator.
        if fault_tolerant and detected and t>=6.0:
            ctrl.set_effectiveness(0.55)

        ts.append(t); xs.append(x); hats.append(xhat); us.append(u)
        rs.append(score); flags.append(detected); refs.append(ref)

    a=lambda q: np.asarray(q)
    X=a(xs); R=a(refs)
    rmse=float(np.sqrt(np.mean((X[:,0]-R[:,0])**2)))
    return {"t":a(ts),"x":X,"xhat":a(hats),"u":a(us),
            "residual":a(rs),"detected":a(flags),"ref":R,"rmse":rmse}
