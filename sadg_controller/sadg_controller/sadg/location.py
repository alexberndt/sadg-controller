import math


class Location:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __eq__(self, __o: object) -> bool:
        return self.x == __o.x and self.y == __o.y

    def __repr__(self) -> str:
        return f"Location(x={self.x},y={self.y})"


def calculate_distance(l1: Location, l2: Location) -> float:
    """
    Calculate the euclidean distance between
    two locations.
    """
    return math.sqrt((l1.x - l2.x) ** 2 + (l1.y - l2.y) ** 2)
