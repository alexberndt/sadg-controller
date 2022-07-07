from logging import Logger
from re import S
from typing import Dict

from sadg_controller.core.agent import Agent
from sadg_controller.core.location import Location
from sadg_controller.core.tuple import Tuple

logger = Logger(__name__)


class Plan:
    def __init__(self, solution: Dict) -> None:
        self.plans = _parse_solution(solution)
        self.logger = logger


def _parse_solution(solution: Dict) -> None:

    # statistics = solution["statistics"]
    schedule = solution["schedule"]

    plans = {}

    for id, agent_schedule in schedule.items():

        plan = []

        for start, goal in zip(agent_schedule, agent_schedule[1:]):

            loc_s = Location(start["x"], start["y"])
            loc_g = Location(goal["x"], goal["y"])
            t_s = start["t"]
            t_g = goal["t"]

            plan.append(Tuple(loc_s,loc_g,t_s,t_g))

        plans[id] = plan

    return plans


