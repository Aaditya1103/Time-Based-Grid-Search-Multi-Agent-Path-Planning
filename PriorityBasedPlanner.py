import time
from typing import List, Tuple

import numpy as np

from GridWithDynamicObstacles import Grid, Interval, ObstacleArrangement, Position
from BaseClasses import MultiAgentPlanner, StartAndGoal
from BaseClasses import SingleAgentPlanner
from Node import NodePath
from SafeInterval import SafeIntervalPathPlanner
from Plotting import PlotNodePaths


class PriorityPlanner(MultiAgentPlanner):
    """
    PriorityPlanner schedules single-agent planning tasks one-by-one.
    Each agent's starting cell is initially reserved, we order agents by
    distance-to-goal (longest first), plan for each in that order, and
    commit the resulting path as a dynamic reservation so later agents avoid it.
    """

    @staticmethod
    def plan_all(
        world: Grid,
        start_goal_pairs: List[StartAndGoal],
        single_agent_planner: SingleAgentPlanner,
        verbose: bool = False,
    ) -> Tuple[List[StartAndGoal], List[NodePath]]:
        """
        Return a tuple (ordered_start_goal_list, corresponding_paths).
        If any agent fails to find a path, returns ([], []).
        """
        print("Single-agent algorithm:", single_agent_planner)

        # block out all initial starts for a short interval so planners treat them as occupied
        initial_block = Interval(0, 10)
        for sg in start_goal_pairs:
            world.reserve_position(sg.start, sg.index, initial_block)

        # highest priority = furthest start->goal distance
        ordered = sorted(start_goal_pairs, key=lambda sg: -sg.distance_start_to_goal())

        planned_paths: List[NodePath] = []
        for sg in ordered:
            if verbose:
                print(f"[Planner] Computing path for agent #{sg.index}, start={sg.start}, goal={sg.goal}")

            # allow the active agent to be at its own start while planning
            world.clear_initial_reservation(sg.start, sg.index)

            route = single_agent_planner.plan(world, sg.start, sg.goal, verbose)

            if route is None:
                print(f"[Planner] No route found for agent #{sg.index}. Aborting.")
                return ([], [])

            # mark the route so future planners see it as a dynamic obstacle
            world.reserve_path(route, sg.index)
            planned_paths.append(route)

        return (ordered, planned_paths)


# ---------------------------
# Execution / demo section
# ---------------------------

VERBOSE = False
SHOW_VIS = True


def demo():
    # grid dimensions
    side = 21
    grid = Grid(
        np.array([side, side]),
        num_obstacles=250,
        obstacle_avoid_points=None,  # set below after we build start/goal list
        obstacle_arrangement=ObstacleArrangement.ARRANGEMENT1,
    )

    # create symmetric start/goal set (keeps same layout as original script)
    sg_list = [StartAndGoal(i, Position(1, i), Position(19, 19 - i)) for i in range(1, 16)]
    avoid_pts = [pt for sg in sg_list for pt in (sg.start, sg.goal)]
    # set obstacle avoidance points (Grid constructor accepted them earlier; set now if needed by implementation)
    grid.obstacle_avoid_points = avoid_pts

    t0 = time.time()
    ordered, paths = PriorityPlanner.plan_all(grid, sg_list, SafeIntervalPathPlanner, VERBOSE)
    elapsed = time.time() - t0
    print(f"Total planning time: {elapsed:.5f} s")

    if not ordered:
        print("Planner terminated with failure (no complete solution).")
        return

    if VERBOSE:
        print("Planned paths:")
        for p in paths:
            print(p)

    if SHOW_VIS:
        PlotNodePaths(grid, ordered, paths)


if __name__ == "__main__":
    demo()
