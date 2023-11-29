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

from logging import Logger
from typing import Dict, List

import mip
from mip import BINARY, CONTINUOUS, INF, MINIMIZE, minimize, xsum
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

                # Add t_s and t_g dependencies
                m_opt.add_var(
                    name=f"{vertex.get_shorthand()}_s", var_type=CONTINUOUS, lb=-INF
                )
                m_opt.add_var(
                    name=f"{vertex.get_shorthand()}_g", var_type=CONTINUOUS, lb=-INF
                )

                # Add dependency: t_g_i >= t_s_i + expected execution time
                vertex_s_name = vertex.get_shorthand() + "_s"
                vertex_s = m_opt.var_by_name(vertex_s_name)
                vertex_g_name = vertex.get_shorthand() + "_g"
                vertex_g = m_opt.var_by_name(vertex_g_name)

                # Add dependency: t_g_i >= t_s_i + expected execution time
                m_opt.add_constr(
                    lin_expr=vertex_g
                    >= vertex_s + vertex.get_expected_completion_time() + EPSILON,
                    name=f"{vertex_s_name}_to_{vertex_g_name}",
                )

        # Add Type-1 Dependencies
        for _, deps in self.regular_deps.items():
            for dep in deps:
                tail = dep.get_tail()
                head = dep.get_head()

                head_s_name = head.get_shorthand() + "_s"
                head_s = m_opt.var_by_name(head_s_name)
                tail_g_name = tail.get_shorthand() + "_g"
                tail_g = m_opt.var_by_name(tail_g_name)

                # Add dependency: t_s_i >= t_g_{i-1}
                m_opt.add_constr(
                    lin_expr=head_s >= tail_g + EPSILON,
                    name=f"{tail_g_name}_to_{head_s_name}",
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
                        tail_act = active_dep.get_tail()
                        head_act = active_dep.get_head()

                        tail_act_g_name = tail_act.get_shorthand() + "_g"
                        tail_act_g_var = m_opt.var_by_name(tail_act_g_name)
                        head_act_s_name = head_act.get_shorthand() + "_s"
                        head_act_s_var = m_opt.var_by_name(head_act_s_name)

                        # Add big-M constraint:        M
                        m_opt.add_constr(
                            lin_expr=head_act_s_var >= tail_act_g_var + EPSILON - b * M,
                            name=f"sw_fwd_{tail_act_g_name}_to_{head_act_s_name}",
                        )

                        # Add inactive dependency
                        inactive_dep = dependency_switch.get_inactive()
                        tail_in = inactive_dep.get_tail()
                        head_in = inactive_dep.get_head()

                        tail_in_g_name = tail_in.get_shorthand() + "_g"
                        tail_in_g_var = m_opt.var_by_name(tail_in_g_name)
                        head_in_s_name = head_in.get_shorthand() + "_s"
                        head_in_s_var = m_opt.var_by_name(head_in_s_name)

                        # Add big-M constraint:     (1 - b)*M
                        m_opt.add_constr(
                            lin_expr=head_in_s_var
                            >= tail_in_g_var + EPSILON - (1 - b) * M,
                            name=f"sw_rev_{tail_in_g_var}_to_{head_in_s_var}",
                        )

                else:

                    # Add non-switchable dependency for each active dependency
                    # switch in the dependency group
                    for dependency_switch in dep_group.get_dependencies():
                        active_dep = dependency_switch.get_active()

                        tail_act = active_dep.get_tail()
                        head_act = active_dep.get_head()

                        tail_act_g_name = tail_act.get_shorthand() + "_g"
                        tail_act_g_var = m_opt.var_by_name(tail_act_g_name)
                        head_act_s_name = head_act.get_shorthand() + "_s"
                        head_act_s_var = m_opt.var_by_name(head_act_s_name)

                        m_opt.add_constr(
                            lin_expr=head_act_s_var >= tail_act_g_var + EPSILON,
                            name=f"sw_nan_{tail_act_g_name}_to_{head_act_s_name}",
                        )

        # Add boundary conditions
        for _, vertices in self.vertices_by_agent.items():
            for vertex in vertices:
                # Looping through vertices to find first
                # STAGED or IN_PROGRESS vertex and set boundary
                # condition in MILP problem

                if vertex.get_status() == Status.STAGED:

                    var_name = vertex.get_shorthand() + "_g"
                    m_opt.add_constr(
                        lin_expr=m_opt.var_by_name(var_name)
                        >= vertex.get_expected_completion_time(),
                        name=f"boundary_{var_name}",
                    )
                    break

                elif vertex.get_status() == Status.IN_PROGRESS:
                    progress = vertex.get_progress()

                    var_name = vertex.get_shorthand() + "_g"
                    m_opt.add_constr(
                        lin_expr=m_opt.var_by_name(var_name)
                        >= (1 - progress) * vertex.get_expected_completion_time(),
                        name=f"boundary_{var_name}",
                    )
                    break
                else:

                    # Check if vertex has next. If not, we need
                    # to set a dummy boundary condition, since the
                    # entire list of vertices has been completed.
                    if not vertex.has_next():
                        var_name = vertex.get_shorthand() + "_g"
                        m_opt.add_constr(
                            lin_expr=m_opt.var_by_name(var_name) >= 0,
                            name=f"boundary_{var_name}",
                        )

        # Get last vertex for each agent to create objective function
        last_vertices = []
        for _, vertices in self.vertices_by_agent.items():
            last_vertices.append(vertices[-1])

        last_vertex_vars = [
            m_opt.var_by_name(var.get_shorthand() + "_g") for var in last_vertices
        ]

        # Define objective function
        m_opt.verbose = 0
        m_opt.objective = minimize(xsum(last_vertex_vars))

        # Print optimization problem
        print(m_opt.objective)
        for constr in m_opt.constrs:
            print(constr)

        # Run the optimization
        _ = m_opt.optimize(max_seconds=60)
        self.logger.info(f"{m_opt.objective_value}")
        print(f"{m_opt.objective_value}")

        print("Values of optimization variables")
        for var in m_opt.vars:
            print(f"{var.name}: {var.x}")

        print("--------------------------------")

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
                    dep_group = self.switchable_dep_groups[agent_id][dep_group_idx]
                    if dep_group.is_switchable():
                        dep_group.switch()

                else:
                    self.logger.info(
                        f"Dependency group {dg_without_prefix}: NOT Switching ..."
                    )
                    # print(f"Dependency group {dg_without_prefix}: NOT Switching ...")
