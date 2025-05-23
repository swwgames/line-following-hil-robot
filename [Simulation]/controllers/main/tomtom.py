from collections import deque
from typing import Dict, List, Optional, Tuple, Union

grid_map = {
    "A1": {"N": "P1", "E": "A2", "S": "C1", "W": None},
    "A2": {"N": "P2", "E": "A3", "S": None, "W": "A1"},
    "A3": {"N": "P3", "E": "A4", "S": None, "W": "A2"},
    "A4": {"N": "P4", "E": "A5", "S": None, "W": "A3"},
    "A5": {"N": None, "E": "A6", "S": "B1", "W": "A4"},
    "A6": {"N": None, "E": None, "S": "B2", "W": "A5"},

    "B1": {"N": "A5", "E": "B2", "S": "C2", "W": None},
    "B2": {"N": "A6", "E": None, "S": "C3", "W": "B1"},

    "C1": {"N": "A1", "E": "C2", "S": "D1", "W": None},
    "C2": {"N": "B1", "E": "C3", "S": "D2", "W": "C1"},
    "C3": {"N": "B2", "E": None, "S": "E6", "W": "C2"},

    "D1": {"N": "C1", "E": "D2", "S": "E1", "W": None},
    "D2": {"N": "C2", "E": None, "S": "E2", "W": "D1"},

    "E1": {"N": "D1", "E": "E2", "S": None, "W": None},
    "E2": {"N": "D2", "E": "E3", "S": None, "W": "E1"},
    "E3": {"N": None, "E": "E4", "S": "P5", "W": "E2"},
    "E4": {"N": None, "E": "E5", "S": "P6", "W": "E3"},
    "E5": {"N": None, "E": "E6", "S": "P7", "W": "E4"},
    "E6": {"N": "C3", "E": None, "S": "P8", "W": "E5"},

    "P1": {"N": None, "E": None, "S": "A1", "W": None},
    "P2": {"N": None, "E": None, "S": "A2", "W": None},
    "P3": {"N": None, "E": None, "S": "A3", "W": None},
    "P4": {"N": None, "E": None, "S": "A4", "W": None},
    "P5": {"N": "E3", "E": None, "S": None, "W": None},
    "P6": {"N": "E4", "E": None, "S": None, "W": None},
    "P7": {"N": "E5", "E": None, "S": None, "W": None},
    "P8": {"N": "E6", "E": None, "S": None, "W": None},
}

class TomTom:
    """
    Navigator that plans and executes routes on a grid of named nodes,
    using a LineTracer to follow lines and make 90° turns.
    """
    def __init__(
        self,
        tracer
    ):
        self.tracer = tracer
        self.grid_map = grid_map

    def plan_route(self, goal: str) -> List[str]:
        """
        Breadth-first search for the shortest path from current_node to goal.
        Returns list of node names, including start and goal; empty if unreachable.
        """
        frontier = deque([[self.current_node]])
        visited = {self.current_node}
        while frontier:
            path = frontier.popleft()
            node = path[-1]
            if node == goal:
                return path
            for direction, neighbor in self.grid_map[node].items():
                if neighbor and neighbor not in visited:
                    visited.add(neighbor)
                    frontier.append(path + [neighbor])
        return []

    def _relative_turn(self, desired: str) -> Union[None, str, Tuple[str,str]]:
        """
        Compute the minimal sequence of 90° turns (CW/CCW) to go from
        self.heading to desired (one of 'N','E','S','W').
        Returns:
          None          = no turn
          'CW' or 'CCW' = single 90°
          ('CW','CW')  = 180°
        """
        order = ['N','E','S','W']
        ci = order.index(self.heading)
        di = order.index(desired)
        diff = (di - ci) % 4
        if diff == 0:
            return None
        if diff == 1:
            return 'CW'
        if diff == 3:
            return 'CCW'
        # diff == 2
        return ('CW','CW')

    def navigate_to(
        self,
        origin: str,
        goal: str,
        start_heading: str = 'N'
    ):
        # re-init state
        self.current_node = origin
        self.heading = start_heading

        path = self.plan_route(goal)
        if not path:
            print(f"No path from {origin} to {goal}")
            return

        print(f"Route: {' → '.join(path)} (start heading={self.heading})")

        for next_node in path[1:]:
            # 1) figure out absolute direction (map‐north) to next_node
            for abs_dir, nb in self.grid_map[self.current_node].items():
                if nb == next_node:
                    desired = abs_dir
                    break
            else:
                raise RuntimeError(f"Bad step: {self.current_node}→{next_node}")

            # 2) compute relative turn(s) from self.heading → desired
            turns = self._relative_turn(desired)
            print(f"At {self.current_node}, heading={self.heading} → next={next_node}, desired={desired}, turns={turns}")

            # 3) pivot **in place** if needed
            if turns is None:
                # no pivot; we just go straight off this junction
                pass
            else:
                # could be single or double 90°’s
                if isinstance(turns, tuple):
                    for t in turns:
                        print(f" Pivot: {t}")
                        self.tracer.pivot_into_direction(direction=t)
                else:
                    print(f" Pivot: {turns}")
                    self.tracer.pivot_into_direction(direction=turns)

                # now we are already aligned with the new branch
                self.heading = desired

            # 4) drive straight to the _next_ junction
            if next_node.startswith('P'):
                self.tracer.drive_forward_until_bump()
            else:
                self.tracer.follow_until_junction()


            # 5) update position
            self.current_node = next_node
            print(f" Arrived at {self.current_node}, heading={self.heading}")

    def _rotate_heading(self, heading: str, turn: str) -> str:
        """Rotate a compass heading by 'CW' or 'CCW'."""
        order = ['N','E','S','W']
        i = order.index(heading)
        if turn == 'CW':
            return order[(i+1)%4]
        if turn == 'CCW':
            return order[(i-1)%4]
        return heading

    def _sense_junction(self) -> Dict[str,bool]:
        """
        Return what the robot sees at the current junction:
          'front','left','right' → True if that branch exists.
        """
        flags = {}
        # front = any line under front array?
        flags['front'] = any(self.tracer.robot.read_line_sensors('front'))
        # left  = any line under left array?
        flags['left']  = any(self.tracer.robot.read_line_sensors('left'))
        # right = any line under right array?
        flags['right'] = any(self.tracer.robot.read_line_sensors('right'))
        return flags

    def _match_observation(self,
                           node: str,
                           heading: str,
                           obs: Dict[str,bool]) -> bool:
        """
        Given a map node & heading candidate, return True if its
        map‐neighbors match the observed front/left/right availability.
        """
        for rel_dir, saw in obs.items():
            # absolute direction of that relative branch
            if rel_dir == 'front':
                abs_dir = heading
            elif rel_dir == 'left':
                abs_dir = self._rotate_heading(heading, 'CCW')
            else:  # 'right'
                abs_dir = self._rotate_heading(heading, 'CW')
            exists = self.grid_map[node][abs_dir] is not None
            if exists != saw:
                return False
        return True

    def locate_self(
        self,
        max_steps: int = 50,
        known_heading: Optional[str] = None
    ) -> Optional[Tuple[str,str]]:
        """
        Drive through junctions until the robot deduces its exact
        (node, heading).  If you know your initial heading, pass it
        in via known_heading to filter candidates immediately.
        Returns that pair, or None if inconclusive.
        """
        # 1) initialize candidate set
        if known_heading in ('N','E','S','W'):
            # only these headings
            candidates = [(node, known_heading)
                          for node in self.grid_map]
        else:
            # all four possible headings at every node
            candidates = [
                (node, h)
                for node in self.grid_map
                for h in ('N','E','S','W')
            ]
        steps = 0

        while len(candidates) > 1 and steps < max_steps:
            steps += 1

            # drive until the next junction
            self.tracer.follow_until_junction()

            # sense which branches exist here
            obs = self._sense_junction()
            print(f"[Step {steps}] saw {obs}, candidates={candidates}")

            # 2) eliminate any candidate whose map‐pattern ≠ obs
            candidates = [
                (node,head)
                for node,head in candidates
                if self._match_observation(node,head,obs)
            ]
            if len(candidates) <= 1:
                break

            # 3) pick the best branch to split the remaining candidates
            best_rel, best_score = None, float('inf')
            for rel, saw in obs.items():
                if not saw:
                    continue
                # count how many of the remaining candidates actually have this branch
                count = sum(
                    1 for (n,h) in candidates
                    if self.grid_map[n][
                        (h if rel=='front' else
                         self._rotate_heading(h,'CCW') if rel=='left' else
                         self._rotate_heading(h,'CW'))
                    ] is not None
                )
                if 0 < count < best_score:
                    best_score, best_rel = count, rel

            if best_rel is None:
                print("No discriminating branch—stopping localization.")
                break

            # 4) physically pivot into that branch (or go straight)
            if best_rel == 'left':
                self.tracer.pivot_into_direction('CCW')
                rel_turn = 'CCW'
            elif best_rel == 'right':
                self.tracer.pivot_into_direction('CW')
                rel_turn = 'CW'
            else:
                rel_turn = None

            # 5) advance each surviving candidate along that branch
            new_cands = []
            for node,head in candidates:
                new_h = self._rotate_heading(head, rel_turn) if rel_turn else head
                # pick the neighbor in that direction
                if best_rel == 'front':
                    nbr = self.grid_map[node][head]
                else:
                    nbr = self.grid_map[node][new_h]
                if nbr:
                    new_cands.append((nbr, new_h))
            candidates = new_cands

        # done
        if len(candidates) == 1:
            node, head = candidates[0]
            print(f"✅ Located at {node}, heading={head}")
            return node, head
        else:
            print("❌ Unable to localize uniquely")
            return None
        
    def _last_junction_before(self, pnode: str) -> str:
        """
        In our grid every P-lane node has exactly one neighbor
        that *is* a junction.  Return that neighbor.
        """
        nbrs = [n for n in self.grid_map[pnode].values() if n]
        if len(nbrs) != 1:
            raise ValueError(f"{pnode} has {len(nbrs)} non-junction neighbors!")
        return nbrs[0]

    def perform_box_run(self, pickup: str, dropoff: str, origin: str, heading: str):
        """
        1) go to the last junction before `pickup`
        2) drive forward until bump (pick up)
        3) back up to that same junction
        4) go to the last junction before `dropoff`
        5) drive forward until bump (drop off)
        6) back up to that junction
        """
        # —— pickup phase ——
        print(f"→ Navigating to pickup lane {pickup}")
        
        self.navigate_to(origin, pickup, heading)
        self.tracer.drive_backward_until_junction()

        # —— dropoff phase ——
        print(f"→ Navigating to dropoff lane {dropoff}")
        cNode = self._last_junction_before(pickup)
        self.navigate_to(cNode, dropoff, 'N')
        self.tracer.drive_backward_until_junction()
        self.current_node = self._last_junction_before(dropoff)

        print("✅ Box run complete")