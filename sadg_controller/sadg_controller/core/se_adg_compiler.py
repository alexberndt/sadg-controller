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

from logging import getLogger

from sadg_controller.core.geometry import intersects
from sadg_controller.mapf.plan import Plan
from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex, loc
from sadg_controller.se_adg.se_adg import SE_ADG

logger = getLogger(__name__)


def se_adg_compiler(P: Plan) -> SE_ADG:  # noqa: C901
    """Algorithm 1: SE-ADG Compiler.

    Compiles a spatially exclusive action dependency graph (SE-ADG) from
      a valid MAPF plan.

    Args:
        plan: MAPF plan determined by a MAPF planner.

    Raises:
        PlanContainsDeadlocks.

    Returns:
        SE-ADG from the provided plan.
    """

    # Initialize
    E_se_adg = {}
    V_se_adg = {}

    # Add sequence of vents for each AGV (Alg. 1, lines 1 - 13)
    for agent_id, P_i in P.plans.items():

        # Initialize
        P_i = iter(P_i)
        V_se_adg[agent_id] = []
        E_se_adg[agent_id] = []

        p = next(P_i)
        v = Vertex(agent_id, p, Status.STAGED)
        v_prev = None

        for p_k in P_i:

            v.append_plan_tuple(p_k)

            # Check for spatial exclusivity
            if not intersects(loc(p), loc(p_k)):

                V_se_adg[agent_id].append(v)

                if v_prev is not None:
                    E_se_adg[agent_id].append(Dependency(v_prev, v))

                v_prev = v
                p = p_k
                v = Vertex(agent_id, p, Status.STAGED)

    # Add switchable dependency pairs (Alg. 1, lines 14 - 19)
    for agent_i, vertices_i in V_se_adg.items():
        for agent_j, vertices_j in V_se_adg.items():

            if agent_i == agent_j:
                continue

            for v_i_k in vertices_i:
                for v_j_l in vertices_j:

                    if (
                        v_i_k.get_start_loc() == v_j_l.get_goal_loc()
                        and v_i_k.get_goal_time() <= v_j_l.get_goal_time()
                    ):
                        E_se_adg[agent_i].append(Dependency(v_i_k, v_j_l))

    return SE_ADG(V_se_adg, E_se_adg)
