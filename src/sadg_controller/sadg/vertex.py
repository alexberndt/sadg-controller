from typing import List, Union

from sadg_controller.mapf.plan_tuple import PlanTuple
from sadg_controller.sadg.location import Location
from sadg_controller.sadg.status import Status


class Vertex:
    def __init__(
        self,
        agent_id: str,
        plan_tuples: Union[List[PlanTuple], PlanTuple],
        vertex_idx: int,
        status: Status = Status.STAGED,
    ) -> None:

        self.agent_id = agent_id
        self.vertex_idx = vertex_idx
        self.shorthand = f"v_{self.agent_id}_{self.vertex_idx}"

        self.plan_tuples = plan_tuples if type(plan_tuples) == list else [plan_tuples]
        self.status = status

        self.start_loc = loc(self.plan_tuples[0])
        self.start_time = time(self.plan_tuples[0])
        self._update()

    def get_shorthand(self) -> str:
        return self.shorthand

    def get_vertex_idx(self) -> int:
        return self.vertex_idx

    def get_agent_id(self) -> int:
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

    def _update(self) -> None:
        self.goal_loc = loc(self.plan_tuples[-1])
        self.goal_time = time(self.plan_tuples[-1])

    def __repr__(self):
        # return f"Vertex(agent_id={self.agent_id}, plan_tuples={self.plan_tuples}, status={self.status})"
        return f"Vertex({self.agent_id}, [{self.start_loc},..., {self.goal_loc}], {self.status})"

    def __str__(self):
        return f"Vertex({self.agent_id}, [{self.start_loc},..., {self.goal_loc}], {self.status})"


def loc(p: PlanTuple) -> Location:
    return Location(p.x, p.y)


def time(p: PlanTuple) -> float:
    return p.time
