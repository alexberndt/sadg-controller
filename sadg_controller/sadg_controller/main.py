# sadg-controller
# Copyright (c) 2023 Alexander Berndt, Robert Bosch GmbH
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

from logging import DEBUG, Logger

from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.compiler import compile_sadg
from sadg_controller.sadg.sadg import SADG
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.visualizer import Visualizer


def main() -> None:

    agent_count = 4
    ecbs_w = 1.8

    logger = Logger(__name__, level=DEBUG)

    roadmap_path = (
        "home/alex/sadg_ws/src/sadg-controller/sadg_controller/data/roadmaps/test"
    )
    roadmap = Roadmap(roadmap_path)

    starts = roadmap.random_locations(agent_count)
    goals = roadmap.random_locations(agent_count)

    problem = MAPFProblem(roadmap, starts, goals, logger)
    plan = problem.solve(suboptimality_factor=ecbs_w)

    sadg: SADG = compile_sadg(plan, logger)
    sadg_visualizer = Visualizer(sadg)

    agent_ids = [f"agent{id}" for id in range(agent_count)]
    agent_curr_vertex = [
        sadg.get_first_agent_vertex(agent_id) for agent_id in agent_ids
    ]

    for sim_iter in range(50):

        for idx, curr_vertex in enumerate(agent_curr_vertex):
            agent_id = f"agent{idx}"

            # Agent is at current vertex, can execute (not blocked), and status == STAGED
            if curr_vertex.get_status() == Status.STAGED:

                if curr_vertex.can_execute():
                    print(f"{agent_id}: set to IN-PROGRESS ...")
                    curr_vertex.set_status(Status.IN_PROGRESS)
                else:
                    print(f"{agent_id}: blocked by dependencies ...")

            # Agent is at executing current vertex
            elif curr_vertex.get_status() == Status.IN_PROGRESS:

                # Only set agents 0 and 2 to completed unless 10 time-steps in
                if idx != 0 or sim_iter > 18:
                    print(f"{agent_id}: set to COMPLETED ...")
                    curr_vertex.set_status(Status.COMPLETED)

            elif curr_vertex.get_status() == Status.COMPLETED:

                if curr_vertex.has_next():
                    print(f"{agent_id}: next vertex STAGED ...")

                    # Set current vertex of this agent to the next in the sequence
                    agent_curr_vertex[idx] = curr_vertex.get_next()

                else:
                    print(f"{agent_id}: finished plan!")
            else:
                raise RuntimeError("Should not achieve this state ...")

        sadg_visualizer.refresh()
        print("----------------------------------------")
        sadg.optimize(horizon=2)


if __name__ == "__main__":
    main()
