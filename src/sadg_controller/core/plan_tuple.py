class PlanTuple:
    def __init__(self, x: float, y: float, time: float) -> None:
        self.x = x
        self.y = y
        self.time = time

    def __repr__(self) -> str:
        return f"PlanTuple(x={self.x}, y={self.y}, t={self.time})"
