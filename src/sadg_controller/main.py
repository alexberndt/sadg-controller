from sadg_controller.core.controller import SADGController

# from sadg_controller.core.locations import Locations
from sadg_controller.core.mapf import MAPFProblem
from sadg_controller.core.roadmap import Roadmap


def main(roadmap_file: str, agv_count: int):

    roadmap = Roadmap(roadmap_file)

    starts = roadmap.random_locations(agv_count)
    goals = roadmap.random_locations(agv_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=1.1)

    controller = SADGController(plan)

    del controller


if __name__ == "__main__":

    roadmap_file = "data/roadmaps/test/roadmap.csv"
    agv_count = 8

    main(roadmap_file, agv_count)
