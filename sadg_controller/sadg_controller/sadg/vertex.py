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

from typing import Any, List, Union

from sadg_controller.mapf.plan_tuple import PlanTuple
from sadg_controller.sadg.location import Location, calculate_distance
from sadg_controller.sadg.status import Status

NOMINAL_SPEED_M_PER_S = 2  # meters per second


class Vertex:
    def __init__(
        self,
        agent_id: str,
        plan_tuples: Union[List[PlanTuple], PlanTuple],
        vertex_idx: int,
        status: Status = Status.STAGED,
    ) -> None:

        self.agent_id = agent_id.replace("agent", "")
        self.vertex_idx = vertex_idx
        self.shorthand = f"v_{self.agent_id}_{self.vertex_idx}"

        self.plan_tuples = plan_tuples if type(plan_tuples) == list else [plan_tuples]
        self.status = status
        self.color = status.color()

        self.start_loc = loc(self.plan_tuples[0])
        self.start_time = time(self.plan_tuples[0])

        # List of dependencies pointing TO this vertex
        self.dependencies = []
        self.next_vertex = None
        self.prev_vertex = None
        self._update()

    def get_shorthand(self) -> str:
        """Returns the shorthand `v_{agent_id}_{vertex_idx}`."""
        return self.shorthand

    def get_vertex_idx(self) -> int:
        """Returns the vertex index of this vertex."""
        return self.vertex_idx

    def get_agent_id(self) -> int:
        """Returns the agent id this vertex is associated with."""
        return self.agent_id

    def append_plan_tuple(self, location: PlanTuple):
        self.plan_tuples.append(location)
        self._update()

    def get_start_loc(self) -> Location:
        return self.start_loc

    def get_goal_loc(self) -> Location:
        return self.goal_loc

    def get_start_time(self) -> float:
        return self.start_time

    def get_goal_time(self) -> float:
        return self.goal_time

    def get_status(self) -> Status:
        return self.status

    def set_prev(self, vertex) -> None:
        self.prev_vertex = vertex

    def set_next(self, vertex) -> None:
        self.next_vertex = vertex
        vertex.set_prev(self)

    def set_status(self, status: Status) -> None:
        self.status = status
        self.color = status.color()

    def get_next(self):
        return self.next_vertex

    def has_next(self) -> bool:
        return self.next_vertex is not None

    def get_prev(self):
        return self.prev_vertex

    def has_prev(self) -> bool:
        return self.prev_vertex is not None

    def add_dependency(self, dependency) -> None:
        self.dependencies.append(dependency)

    def get_dependencies(self) -> List[Any]:
        return self.dependencies

    def can_execute(self) -> bool:
        """
        Checks if _all_ tail vertices of _active_ dependencies
        pointing to this vertex are completed. If so, this ver-
        tex can be executed, otherwise not.
        """
        for dependency in self.dependencies:
            if dependency.is_active():
                if dependency.get_tail().get_status() is not Status.COMPLETED:
                    return False
        return True

    def get_blocking_vertices(self) -> List:
        """
        Get list of vertices blocking this vertex.
        """
        blocking_vertices = []
        for dependency in self.dependencies:
            if dependency.is_active():
                if dependency.get_tail().get_status() is not Status.COMPLETED:
                    blocking_vertices.append(dependency.get_tail())
        return blocking_vertices

    def _update(self) -> None:
        plan_tuples_distance = calculate_plan_tuple_distance(self.plan_tuples)

        self.expected_completion_time = plan_tuples_distance / NOMINAL_SPEED_M_PER_S

        self.goal_loc = loc(self.plan_tuples[-1])
        self.goal_time = time(self.plan_tuples[-1])

    def get_expected_completion_time(self) -> float:
        return self.expected_completion_time

    def get_progress(self) -> float:
        """
        Return the fraction of progress this vertex is from completed.
        # TODO - need to obtain live info from agent here
        """
        return 0.5

    def __repr__(self):
        # return f"Vertex(agent_id={self.agent_id}, plan_tuples={self.plan_tuples}, status={self.status})"
        return f"Vertex({self.agent_id}, [{self.start_loc},..., {self.goal_loc}], {self.status})"

    def __str__(self):
        return f"Vertex({self.agent_id}, [{self.start_loc},..., {self.goal_loc}], {self.status})"


def loc(p: PlanTuple) -> Location:
    return Location(p.x, p.y)


def time(p: PlanTuple) -> float:
    return p.time


def calculate_plan_tuple_distance(plan_tuples: List[PlanTuple]) -> float:
    """
    Calculates the total distance of successive
    locations in the provided list of plan tuples.
    """
    l_prev = None
    distance = 0
    for plan_tuple in plan_tuples:
        if l_prev is None:
            l_prev = loc(plan_tuple)
            continue
        l_curr = loc(plan_tuple)
        distance += calculate_distance(l_prev, l_curr)
        l_prev = l_curr
    return distance
