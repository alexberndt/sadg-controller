from sadg_controller.core.mapf import MAPFProblem
from sadg_controller.core.roadmap import Roadmap
from sadg_controller.sadg.functions import sadg_compiler


def main(roadmap_file: str, dimensions_file: str, agv_count: int):

    roadmap = Roadmap(roadmap_file, dimensions_file)

    starts = roadmap.random_locations(agv_count)
    goals = roadmap.random_locations(agv_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=1.1)

    sadg = sadg_compiler(plan)

    del sadg


if __name__ == "__main__":

    roadmap_name = "test"
    roadmap_file = f"data/roadmaps/{roadmap_name}/roadmap.csv"
    dimensions_file = f"data/roadmaps/{roadmap_name}/dimensions.yaml"

    agv_count = 8

    main(roadmap_file, dimensions_file, agv_count)
