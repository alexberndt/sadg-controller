from logging import getLogger
from typing import Dict

from sadg_controller.mapf.plan_tuple import PlanTuple

logger = getLogger(__name__)


class Plan:
    def __init__(self, solution: Dict, dimensions: Dict) -> None:
        self.plans = _parse_solution(solution, dimensions)
        self.logger = logger


def _parse_solution(solution: Dict, dimensions: Dict) -> None:
    """Parse a ECBS-derived MAPF solution.

    Converts the raw solution output of the ECBS planner into

    Args:
        solution:
        dimensions:
    """

    # read dimensions
    dims = dimensions["dimensions"]
    x_offset = dims["x_offset"]
    y_offset = dims["y_offset"]
    resolution = dims["resolution"]

    # statistics = solution["statistics"]
    schedule = solution["schedule"]

    plans = {}

    for id, agent_schedule in schedule.items():

        plan = []

        for schedule_item in agent_schedule:

            x = schedule_item["y"] * resolution + x_offset
            y = -schedule_item["x"] * resolution + y_offset
            t = schedule_item["t"]

            plan.append(PlanTuple(x, y, t))

        plans[id] = plan

    return plans
