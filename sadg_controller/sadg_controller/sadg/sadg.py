from logging import Logger
from typing import Dict, List

import mip
from mip import BINARY, CONTINUOUS, MINIMIZE, minimize, xsum
from mip.entities import Var

from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_group import DependencyGroup
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex

EPSILON = 0.01
M = 1000000


class SADG:
    def __init__(
        self,
        vertices_by_agent: Dict[str, List[Vertex]],
        regular_deps: Dict[str, List[Dependency]],
        switchable_dep_groups: Dict[str, List[DependencyGroup]],
        logger: Logger,
    ) -> None:
        self.vertices_by_agent = vertices_by_agent
        self.regular_deps = regular_deps
        self.switchable_dep_groups = switchable_dep_groups
        self.logger = logger

    def get_first_agent_vertex(self, agent_id: str) -> Vertex:
        return self.vertices_by_agent[agent_id][0]

    def get_agent_vertices(self, agent_id: str) -> List[Vertex]:
        return self.vertices_by_agent[agent_id]

    def get_switchable_dep_groups(self) -> Dict[str, List[DependencyGroup]]:
        return self.switchable_dep_groups

    def optimize(self, horizon: float = 5) -> None:  # noqa: C901
        """
        Run Optimization for the SADG method.

        Args:
            horizon: Time horizon within which to consider dependency
                group parameters
        """

        # Build the MILP
        self.logger.info("Optimizing ...")

        # Create MILP dictionary
        m_opt = mip.Model(name="SADG_RHC", sense=MINIMIZE)
        m_opt.clear()

        # Initialize continuous optimization variables
        for _, vertices in self.vertices_by_agent.items():
            for vertex in vertices:
                m_opt.add_var(name=vertex.get_shorthand(), var_type=CONTINUOUS)

        # Add Type-1 Dependencies
        for _, deps in self.regular_deps.items():
            for dep in deps:
                tail = dep.get_tail()
                head = dep.get_head()

                tail_var = m_opt.var_by_name(tail.get_shorthand())
                head_var = m_opt.var_by_name(head.get_shorthand())

                m_opt.add_constr(
                    lin_expr=head_var
                    >= tail_var + tail.get_expected_completion_time() + EPSILON,
                    name=f"{tail.get_shorthand()}_to_{head.get_shorthand()}",
                )

        # Add Type-2 Dependencies
        for _, dep_groups in self.switchable_dep_groups.items():
            for dep_group in dep_groups:

                # Check if dependency group can be switched
                if dep_group.is_switchable() and dep_group.within_horizon(horizon):

                    # Add big-M constraint for each dependency switch in
                    # the dependency group
                    b: Var = m_opt.add_var(name=dep_group.get_uid(), var_type=BINARY)

                    for dependency_switch in dep_group.get_dependencies():

                        # Add active dependency
                        active_dep = dependency_switch.get_active()
                        tail = active_dep.get_tail()
                        head = active_dep.get_head()

                        tail_var = m_opt.var_by_name(tail.get_shorthand())
                        head_var = m_opt.var_by_name(head.get_shorthand())

                        # Add big-M constraint:        M
                        m_opt.add_constr(
                            lin_expr=head_var >= tail_var + EPSILON - b * M,
                            name=f"{tail.get_shorthand()}_to_{head.get_shorthand()}",
                        )

                        # Add inactive dependency
                        inactive_dep = dependency_switch.get_inactive()
                        tail = inactive_dep.get_tail()
                        head = inactive_dep.get_head()

                        tail_var = m_opt.var_by_name(tail.get_shorthand())
                        head_var = m_opt.var_by_name(head.get_shorthand())

                        # Add big-M constraint:     (1 - b)*M
                        m_opt.add_constr(
                            lin_expr=head_var >= tail_var + EPSILON - (1 - b) * M,
                            name=f"{tail.get_shorthand()}_to_{head.get_shorthand()}",
                        )

                else:

                    # Add non-switchable dependency for each active dependency
                    # switch in the dependency group
                    for dependency_switch in dep_group.get_dependencies():
                        active_dep = dependency_switch.get_active()

                        tail = active_dep.get_tail()
                        head = active_dep.get_head()

                        m_opt.add_constr(
                            lin_expr=head_var >= tail_var,
                            name=f"{tail.get_shorthand()}_to_{head.get_shorthand()}",
                        )

        # Add boundary conditions
        for _, vertices in self.vertices_by_agent.items():
            for vertex in vertices:
                # Looping through vertices to find first
                # STAGED or IN_PROGRESS vertex and set boundary
                # condition in MILP problem
                if vertex.get_status() == Status.STAGED:
                    m_opt.add_constr(
                        lin_expr=m_opt.var_by_name(vertex.get_shorthand())
                        >= vertex.get_expected_completion_time(),
                        name=f"boundary_{vertex.get_shorthand()}",
                    )
                    break
                elif vertex.get_status() == Status.IN_PROGRESS:
                    progress = vertex.get_progress()
                    m_opt.add_constr(
                        lin_expr=m_opt.var_by_name(vertex.get_shorthand())
                        >= (1 - progress) * vertex.get_expected_completion_time(),
                        name=f"boundary_{vertex.get_shorthand()}",
                    )
                    break
                else:

                    # Check if vertex has next. If not, we need
                    # to set a dummy boundary condition, since the
                    # entire list of vertices has been completed.
                    if not vertex.has_next():
                        m_opt.add_constr(
                            lin_expr=m_opt.var_by_name(vertex.get_shorthand()) >= 0,
                            name=f"boundary_{vertex.get_shorthand()}",
                        )

        # Get last vertex for each agent to create objective function
        last_vertices = []
        for _, vertices in self.vertices_by_agent.items():
            last_vertices.append(vertices[-1])

        last_vertex_vars = [
            m_opt.var_by_name(var.get_shorthand()) for var in last_vertices
        ]

        # Define objective function
        m_opt.verbose = 0
        m_opt.objective = minimize(xsum(last_vertex_vars))

        # Run the optimization
        _ = m_opt.optimize(max_seconds=60)
        self.logger.info(f"{m_opt.objective_value}")
        print(f"{m_opt.objective_value}")

        for var in m_opt.vars:

            # Find all binary variables corresponding
            # to a dependency group
            if var.name.startswith("dg_"):

                # Parse DG name : "dg_{agent_id}_{dg_index}"
                dg_without_prefix = var.name.replace("dg_", "")
                agent_id, dep_group_idx = dg_without_prefix.split("_")
                dep_group_idx = int(dep_group_idx)

                if var.x == 1.0:
                    self.logger.info(
                        f"Dependency group {dg_without_prefix}: Switching ..."
                    )
                    # print(f"Dependency group {dg_without_prefix}: Switching ...")
                    # Apply switching to DG
                    self.switchable_dep_groups[agent_id][dep_group_idx].switch()

                else:
                    self.logger.info(
                        f"Dependency group {dg_without_prefix}: NOT Switching ..."
                    )
                    # print(f"Dependency group {dg_without_prefix}: NOT Switching ...")
