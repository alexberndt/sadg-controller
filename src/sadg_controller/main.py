from sadg_controller.core.sadg_compiler import sadg_compiler
from sadg_controller.core.se_adg_compiler import se_adg_compiler
from sadg_controller.mapf.problem import MAPFProblem
from sadg_controller.mapf.roadmap import Roadmap


def main(roadmap_file: str, dimensions_file: str, agv_count: int):

    roadmap = Roadmap(roadmap_file, dimensions_file)

    starts = roadmap.random_locations(agv_count)
    goals = roadmap.random_locations(agv_count)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve(suboptimality_factor=1.8)

    sadg = sadg_compiler(plan)
    se_adg = se_adg_compiler(plan)

    del sadg
    del se_adg

    for t in range(1, 1):

        print(f"t = {t}")


if __name__ == "__main__":

    roadmap_name = "test"
    roadmap_file = f"data/roadmaps/{roadmap_name}/roadmap.csv"
    dimensions_file = f"data/roadmaps/{roadmap_name}/dimensions.yaml"

    agv_count = 19

    main(roadmap_file, dimensions_file, agv_count)
