from logging import DEBUG, Logger

from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap
from sadg_controller.sadg.compiler import compile_sadg


def main() -> None:

    agent_count = 8
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

    sadg = compile_sadg(plan, logger)

    sadg.optimize()


if __name__ == "__main__":
    main()
