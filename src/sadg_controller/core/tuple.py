

from sadg_controller.core.location import Location


class Tuple:
    def __init__(self, start: Location, goal: Location, t_s: float, t_g: float) -> None:
        self.start = start
        self.goal = goal 
        self.t_s = t_s
        self.t_g = t_g