from sadg_controller.core.controller import SADGController
from sadg_controller.core.locations import Locations
from sadg_controller.core.mapf import MAPFProblem
from sadg_controller.core.roadmap import Roadmap


def main():

    roadmap = Roadmap("data/roadmaps/warehouse.csv")

    starts = roadmap.random_locations(20)
    goals = roadmap.random_locations(20)

    problem = MAPFProblem(roadmap, starts, goals)
    plan = problem.solve()

    controller = SADGController(plan)

    




if __name__ == "__main__":
    main()
