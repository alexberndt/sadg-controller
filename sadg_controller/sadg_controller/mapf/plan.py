# sadg-controller - MAPF execution with Switchable Action Dependency Graphs
# Copyright (c) 2023 Alex Berndt
# Copyright (c) 2023 Niels van Duijkeren, Robert Bosch GmbH
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Affero General Public License as published
# by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Affero General Public License for more details.
#
# You should have received a copy of the GNU Affero General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
