import matplotlib.pyplot as plt
from src.simulation import run

base=run(False)
ftc=run(True)
print("Baseline RMSE:",base["rmse"])
print("Fault-tolerant RMSE:",ftc["rmse"])

plt.figure()
plt.plot(base["t"],base["ref"][:,0],"--",label="Reference")
plt.plot(base["t"],base["x"][:,0],label="Nominal under faults")
plt.plot(ftc["t"],ftc["x"][:,0],label="Fault-tolerant")
plt.xlabel("Time [s]"); plt.ylabel("Tracked state")
plt.grid(True); plt.legend(); plt.title("Fault-Tolerant Control Comparison")
plt.show()

plt.figure()
plt.plot(ftc["t"],ftc["residual"],label="Observer residual norm")
plt.xlabel("Time [s]"); plt.ylabel("Residual")
plt.grid(True); plt.legend(); plt.title("Residual-Based Fault Detection")
plt.show()
