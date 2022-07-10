from logging import getLogger

from sadg_controller.core.geometry import intersects
from sadg_controller.mapf.plan import Plan
from sadg_controller.sadg.dependency import Dependency
from sadg_controller.sadg.dependency_switch import DependencySwitch
from sadg_controller.sadg.sadg import SADG
from sadg_controller.sadg.status import Status
from sadg_controller.sadg.vertex import Vertex, loc

logger = getLogger(__name__)


def sadg_compiler(P: Plan) -> SADG:  # noqa: C901
    """Algorithm 2: SADG Compiler.

    Compiles a switchable action dependency graph (SADG) from
      a valid MAPF plan.

    Args:
        plan: MAPF plan determined by a MAPF planner.

    Raises:
        PlanContainsDeadlocks.

    Returns:
        SADG from the provided plan.
    """

    # N = len(P.plans.keys())

    # Initialize
    E_regular = {}
    E_switchable = {}
    V_sadg = {}

    # Add sequence of vents for each AGV (Alg. 2, lines 1 - 13)
    for agent_id, P_i in P.plans.items():

        # initialize
        # N_i = len(P_i)
        P_i = iter(P_i)
        V_sadg[agent_id] = []
        E_regular[agent_id] = []
        E_switchable[agent_id] = []

        p = next(P_i)
        v = Vertex(agent_id, p, Status.STAGED)
        v_prev = None

        for p_k in P_i:

            v.append_plan_tuple(p_k)

            # Check for spatial exclusivity
            if not intersects(loc(p), loc(p_k)):

                V_sadg[agent_id].append(v)

                if v_prev is not None:
                    E_regular[agent_id].append(Dependency(v_prev, v))

                # reset
                v_prev = v
                p = p_k
                v = Vertex(agent_id, p, Status.STAGED)

    # Add switchable dependency pairs (Alg. 2, lines 14 - 24)
    for agent_i, vertices_i in V_sadg.items():
        for agent_j, vertices_j in V_sadg.items():

            if agent_i == agent_j:
                continue

            for k, v_i_k in enumerate(vertices_i):
                for l, v_j_l in enumerate(vertices_j):  # noqa: E741

                    if (
                        v_i_k.get_start_loc() == v_j_l.get_goal_loc()
                        and v_i_k.get_goal_time() <= v_j_l.get_goal_time()
                    ):

                        fwd = Dependency(v_i_k, v_j_l)

                        try:
                            v_i_k_minus_1 = vertices_i[k - 1]
                            v_j_l_plus_1 = vertices_j[l + 1]
                            rev = Dependency(v_j_l_plus_1, v_i_k_minus_1)
                            switch = DependencySwitch(fwd, rev, False)
                            E_switchable[agent_i].append(switch)

                        except IndexError:
                            logger.debug(
                                f"k-1={k - 1}, l+1={l + 1} are not valid indexes."
                                "Cannot construct dependency switch."
                            )
                            E_regular[agent_i].append(fwd)

    return SADG(V_sadg, E_regular, E_switchable)
