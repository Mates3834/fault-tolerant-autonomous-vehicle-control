import numpy as np

class ResidualDetector:
    def __init__(self,threshold=0.12,persistence=4):
        self.threshold=threshold
        self.persistence=persistence
        self.count=0
        self.detected=False

    def update(self,residual):
        score=float(np.linalg.norm(residual))
        self.count=self.count+1 if score>self.threshold else max(0,self.count-1)
        if self.count>=self.persistence:
            self.detected=True
        return self.detected,score
