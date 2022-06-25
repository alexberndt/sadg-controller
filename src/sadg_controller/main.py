from sadg_controller.core.agv import AGV
from sadg_controller.core.mapf import MAPFProblem
from sadg_controller.core.roadmap import Roadmap

def main():

    roadmap = Roadmap("data/roadmaps/warehouse.csv")
    agv1 = AGV("1")
    agv2 = AGV("2")

    problem = MAPFProblem(roadmap, [agv1, agv2])

    problem.solve()


if __name__ == "__main__":
    main()