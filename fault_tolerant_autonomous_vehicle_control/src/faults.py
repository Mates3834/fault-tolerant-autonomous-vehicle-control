import numpy as np

def actuator_effectiveness(t,onset=6.0,level=0.55):
    return level if t>=onset else 1.0

def sensor_bias(t,onset=10.0,bias=0.18):
    return np.array([bias,0.0]) if t>=onset else np.zeros(2)
