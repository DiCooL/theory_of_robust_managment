class FunctionCorrection:
    def __init__(self, T: float, dt: float):
        self.T = T
        self.dt = dt
        self.y = 0.0

    def reset(self) -> None:
        self.y = 0.0

    def update(self, u: float) -> float:
        self.y = self.y + self.dt * self.T * (u - self.y)
        return self.y
