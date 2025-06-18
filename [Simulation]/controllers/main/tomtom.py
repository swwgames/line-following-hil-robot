import heapq
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
    def __init__(self, tracer):
        """Initialize the TomTom navigation system with a tracer and a grid map.

        Args:
            tracer (obj): An instance of the LineTracer class for driving.
            grid_map (dict): The map of the grid with node connections.
        """

        self.tracer = tracer
        self.grid_map = grid_map
        self.blocked_paths = set()

    def plan_route(self, goal: str) -> List[str]:
        """Plan a route from the current node to the goal node using a breadth-first search. Also minimizes 90° turns.

        Args:
            goal (str): The name of the goal node.

        Returns:
            List[str]: A list of node names from the current node to the goal (inclusive), or an empty list if the goal is unreachable.
        """

        # all possible headings
        headings = ['N', 'E', 'S', 'W']
        start = (self.current_node, self.heading)

        # initialize distances
        INF = float('inf')
        dist = {(n, h): INF for n in self.grid_map for h in headings}
        dist[start] = 0

        # to reconstruct the path
        prev = {}

        dq = deque([start])
        while dq:
            u_node, u_head = dq.popleft()
            u_cost = dist[(u_node, u_head)]
            # explore all available moves
            for abs_dir, v in self.grid_map[u_node].items():
                if v is None or (u_node, v) in self.blocked_paths:
                    continue
                # compute how many 90° turns this move costs
                turns = self._relative_turn(u_head, abs_dir)
                if turns is None:
                    w = 0
                    new_head = u_head
                elif isinstance(turns, tuple):
                    w = len(turns)
                    new_head = abs_dir
                else:
                    w = 1
                    new_head = abs_dir

                new_state = (v, new_head)
                new_cost = u_cost + w
                if new_cost < dist[new_state]:
                    dist[new_state] = new_cost
                    prev[new_state] = (u_node, u_head)
                    # 0-cost edges → front, 1+-cost → back
                    if w == 0:
                        dq.appendleft(new_state)
                    else:
                        dq.append(new_state)

        # pick the heading at 'goal' with the fewest turns
        best_head, best_cost = min(
            ((h, dist[(goal, h)]) for h in headings),
            key=lambda x: x[1]
        )
        if best_cost == INF:
            return []

        # reconstruct the (node,heading) sequence
        seq = []
        state = (goal, best_head)
        while state != start:
            seq.append(state)
            state = prev[state]
        seq.append(start)
        seq.reverse()

        # return only the node names
        return [node for node, _ in seq]

    def plan_route_astar(self, goal: str) -> List[str]:
        """Plan a route from the current node to the goal node using A* search algorithm.

        Args:
            goal (str): The name of the goal node.

        Returns:
            List[str]: A list of node names from the current node to the goal (inclusive), or an empty list if the goal is unreachable.
        """

        # 1) helper: map node names to (x,y) coords on a grid
        def coords(n: str):
            # letters A–E → rows 0–4, digits 1–6 → cols 0–5, P1–P8 above E-row
            if n.startswith('P'):
                # map P1…P4 to row -1, P5…P8 to row 6 (optional)
                i = int(n[1:]) - 1
                row = -1 if i < 4 else 6
                col = i % 4 + (0 if row == -1 else 2)
            else:
                row = ord(n[0]) - ord('A')
                col = int(n[1]) - 1
            return row, col

        # 2) heuristic: minimal turns to face best cardinal toward goal
        def heuristic(state):
            node, heading = state
            r0, c0 = coords(node)
            rg, cg = coords(goal)
            dr, dc = rg - r0, cg - c0
            if abs(dr) > abs(dc):
                desired = 'S' if dr > 0 else 'N'
            else:
                desired = 'E' if dc > 0 else 'W'
            turn = self._relative_turn(heading, desired)
            if turn is None:
                return 0
            return len(turn) if isinstance(turn, tuple) else 1

        # 3) A* setup
        headings = ['N', 'E', 'S', 'W']
        start = (self.current_node, self.heading)
        INF = float('inf')
        g_score = {(n, h): INF for n in self.grid_map for h in headings}
        g_score[start] = 0
        came_from = {}

        # priority queue stores (f_score, g_score, (node,heading))
        open_heap = [(heuristic(start), 0, start)]
        visited = set()

        while open_heap:
            f, g, (u_node, u_head) = heapq.heappop(open_heap)
            if (u_node, u_head) in visited:
                continue
            visited.add((u_node, u_head))

            if u_node == goal:
                # reconstruct path of nodes
                path = []
                cur = (u_node, u_head)
                while cur in came_from:
                    path.append(cur[0])
                    cur = came_from[cur]
                path.append(self.current_node)
                return list(reversed(path))

            # expand neighbors
            for abs_dir, v in self.grid_map[u_node].items():
                if v is None or (u_node, v) in self.blocked_paths:
                    continue
                turns = self._relative_turn(u_head, abs_dir)
                if turns is None:
                    w = 0
                    new_head = u_head
                elif isinstance(turns, tuple):
                    w = len(turns)
                    new_head = abs_dir
                else:
                    w = 1
                    new_head = abs_dir

                tentative_g = g_score[(u_node, u_head)] + w
                if tentative_g < g_score[(v, new_head)]:
                    g_score[(v, new_head)] = tentative_g
                    came_from[(v, new_head)] = (u_node, u_head)
                    f_score = tentative_g + heuristic((v, new_head))
                    heapq.heappush(open_heap, (f_score, tentative_g, (v, new_head)))

        # no path
        return []

    def _relative_turn(self, *args) -> Union[None, str, Tuple[str, str]]:
        """Compute the relative turn direction from the current heading to the desired heading.

        Args:
            *args: either one or two arguments:
                - if one argument, it is the desired heading
                - if two arguments, the first is the current heading and the second is the desired heading

        Raises:
            TypeError: if the number of arguments is not 1 or 2.

        Returns:
            Union[None, str, Tuple[str, str]]:
                - None if no turn is needed (same heading)
                - 'CW' for a single clockwise turn
                - 'CCW' for a single counter-clockwise turn
                - ('CW', 'CW') for a 180° turn (two clockwise turns)
        """

        order = ['N', 'E', 'S', 'W']
        # unpack args
        if len(args) == 1:
            current = self.heading
            desired = args[0]
        elif len(args) == 2:
            current, desired = args
        else:
            raise TypeError(f"_relative_turn() expects 1 or 2 args, got {len(args)}")

        ci = order.index(current)
        di = order.index(desired)
        diff = (di - ci) % 4
        if diff == 0:
            return None
        if diff == 1:
            return 'CW'
        if diff == 3:
            return 'CCW'
        # diff == 2
        return ('CW', 'CW')

    def navigate_to(self, origin: str, goal: str, start_heading: str = 'N', _is_retry: bool = False):
        """Navigate from the origin node to the goal node, starting with a given heading.

        Args:
            origin (str): The name of the starting node.
            goal (str): The name of the goal node.
            start_heading (str): The initial heading of the robot, default is 'N'.
            _is_retry (bool): Internal flag to prevent infinite retry loops.
        """

        self.current_node = origin
        self.heading = start_heading

        path = self.plan_route_astar(goal)

        if not path:
            # If planning fails, check if we have blocked paths in our memory.
            if self.blocked_paths and not _is_retry:
                print(f"No path found from {origin} to {goal}. Assuming temporary obstacles may have cleared.")
                print("Clearing blocked paths and attempting to replan the entire route.")
                self.blocked_paths.clear()
                # Retry planning from the current position, marking this as a retry attempt.
                return self.navigate_to(self.current_node, goal, self.heading, _is_retry=True)
            else:
                # If there were no blocked paths, or if this was already a retry attempt, the goal is truly unreachable.
                print(f"CRITICAL: No path from {origin} to {goal}. The goal is unreachable.")
                return

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
            print(
                f"At {self.current_node}, heading={self.heading} → next={next_node}, desired={desired}, turns={turns}")

            # 3) pivot if needed
            if turns is not None:
                if isinstance(turns, tuple) and len(turns) >= 2:
                    if turns[0] == turns[1]:
                        self.tracer.pivot_into_direction(direction='180')
                else:
                    for t in (turns if isinstance(turns, tuple) else (turns,)):
                        self.tracer.pivot_into_direction(direction=t)

                # now we’re aligned with the branch
                self.heading = desired

            # 4) drive straight to the next junction or bump
            if next_node.startswith('P'):
                self.tracer.drive_forward_until_bump()
            else:
                junction = self.tracer.follow_until_junction()

                if junction is False:
                    # treat as obstacle—mark path as blocked and re-plan
                    print(
                        f"Obstacle detected before junction {next_node}. Marking path ({self.current_node} -> {next_node}) as blocked and replanning.")
                    self.blocked_paths.add((self.current_node, next_node))
                    self.tracer.drive_backward_until_junction()
                    # Recursive call to replan from the current node
                    return self.navigate_to(self.current_node, goal, self.heading)

                expected = set()

                for rel in ('F', 'L', 'R'):
                    if rel == 'F':
                        abs_rel = self.heading
                    elif rel == 'L':
                        abs_rel = self._rotate_heading(self.heading, 'CCW')
                    else:  # 'right'
                        abs_rel = self._rotate_heading(self.heading, 'CW')

                    if self.grid_map[next_node][abs_rel] is not None:
                        expected.add(rel)

                seen = set(junction)
                if seen != expected:
                    print(f"Junction validation failed at {next_node}: "f"expected {expected}, saw {seen}")

                    loc = self.locate_self(known_heading=self.heading)
                    if loc:
                        self.current_node, self.heading = loc
                        print(f"Re-localised to {self.current_node}, heading={self.heading}, ""replanning…")
                        return self.navigate_to(self.current_node, goal, self.heading)
                    else:
                        print("Could not re-localize; aborting navigation")
                        self.tracer.robot.stop_and_close()
                        return

            # 5) update position
            self.current_node = next_node
            print(f" Arrived at {self.current_node}, heading={self.heading}")

    @staticmethod
    def _rotate_heading(heading: str, turn: str) -> str:
        """Rotate the heading based on the turn direction.

        Args:
            heading (str): The current heading ('N', 'E', 'S', 'W').
            turn (str): The turn direction ('CW' for clockwise, 'CCW' for counter-clockwise).

        Returns:
            str: The new heading after the turn.
        """

        order = ['N', 'E', 'S', 'W']
        i = order.index(heading)
        if turn == 'CW':
            return order[(i + 1) % 4]
        if turn == 'CCW':
            return order[(i - 1) % 4]
        return heading

    def _sense_junction(self) -> Dict[str, bool]:
        """Sense the junction by checking the line sensors. This is to validate the robot's position after each junction.

        Returns:
            Dict[str,bool]: A dictionary with keys 'front', 'left', 'right' indicating whether each direction has a line detected (True) or not (False).
        """

        flags = {'front': any(self.tracer.robot.read_line_sensors('front')),
                 'left': any(self.tracer.robot.read_line_sensors('left')),
                 'right': any(self.tracer.robot.read_line_sensors('right'))}
        return flags

    def _match_observation(self, node: str, heading: str, obs: Dict[str, bool]) -> bool:
        """Check if the observed branches at a junction match the expected branches in the grid map.

        Args:
            node (str): The current node name.
            heading (str): The current heading ('N', 'E', 'S', 'W').
            obs (Dict[str,bool]): A dictionary with keys 'front', 'left', 'right' indicating whether each direction has a line detected (True) or not (False).

        Returns:
            bool: True if the observed branches match the expected branches, False otherwise.
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

    def _get_opposite_heading(self, heading: str) -> Optional[str]:
        """Calculates the opposite cardinal direction."""
        if not heading:
            return None
        opposites = {'N': 'S', 'S': 'N', 'E': 'W', 'W': 'E'}
        return opposites.get(heading)

    # Replace the locate_self function in tomtom.py with this version.

    def locate_self(self, max_steps: int = 50, known_heading: Optional[str] = None) -> Optional[Tuple[str, str]]:
        """
        Locate the robot by attempting to find a junction and then exploring. If an
        obstacle is found before the first junction, it attempts one recovery by
        turning around, updating its heading, and trying again.

        Args:
            max_steps (int): The maximum number of steps for the main localization loop.
            known_heading (Optional[str]): The robot's initial known heading ('N','E','S','W').

        Returns:
            A tuple of (node, heading) if localization is successful, otherwise None.
        """
        # This outer loop allows for one initial attempt and one recovery attempt.
        current_heading_attempt = known_heading
        for attempt in range(2):
            if attempt > 0:
                if not current_heading_attempt:
                    print("Cannot attempt recovery without a known initial heading.")
                    return None
                print(f"\n--- Recovery Attempt (New Heading: {current_heading_attempt}) ---")
            else:
                print(f"--- Localization Attempt #1 (Heading: {current_heading_attempt}) ---")

            # --- Initialization for this attempt ---
            if current_heading_attempt:
                candidates = [(node, current_heading_attempt) for node in self.grid_map]
            else:
                candidates = [(node, h) for node in self.grid_map for h in ('N', 'E', 'S', 'W')]
            steps = 0
            blocked_paths = {}

            # --- PHASE 1: Initial Junction Finding ---
            print("Driving forward to find the first junction...")
            initial_junction_result = self.tracer.follow_until_junction()

            if initial_junction_result is False:
                # Obstacle found on this attempt. Prepare for the next one.
                print(f"Obstacle hit before first junction.")
                new_heading = self._get_opposite_heading(current_heading_attempt)

                if new_heading:
                    print(f"Turning 180 degrees. New heading will be {new_heading}.")
                    self.tracer.pivot_into_direction('180')
                    current_heading_attempt = new_heading
                    continue  # Starts the next iteration of the for loop
                else:
                    print("Could not determine opposite heading to recover. Halting.")
                    break  # Exits the for loop, leading to failure

            # --- If we get here, the first junction was found successfully ---
            print("First junction reached. Starting main localization process.")

            # --- PHASE 2: Main Localization Loop ---
            while len(candidates) > 1 and steps < max_steps:
                steps += 1
                obs = self._sense_junction()
                print(f"[Step {steps}] At junction, saw {obs}. Candidates: {len(candidates)}")
                candidates = [c for c in candidates if self._match_observation(c[0], c[1], obs)]

                if len(candidates) <= 1:
                    break

                candidate_key = tuple(sorted(candidates))
                excluded_dirs = blocked_paths.get(candidate_key, set())
                possible_moves = [r for r, saw in obs.items() if saw and r not in excluded_dirs]

                if not possible_moves and candidate_key in blocked_paths:
                    print(f"All known exits {excluded_dirs} are blocked. Resetting memory and retrying.")
                    del blocked_paths[candidate_key]
                    possible_moves = [r for r, saw in obs.items() if saw]

                if not possible_moves:
                    print("No valid paths to explore from this junction. Halting.")
                    break

                best_rel, best_score = None, float('inf')
                for rel in possible_moves:
                    count = sum(1 for (n, h) in candidates if self.grid_map[n][(
                        h if rel == 'front' else self._rotate_heading(h,
                                                                      'CCW') if rel == 'left' else self._rotate_heading(
                            h, 'CW'))] is not None)
                    score = abs(count - len(candidates) / 2)
                    if 0 < count < len(candidates) and score < best_score:
                        best_score, best_rel = score, rel

                if best_rel is None:
                    best_rel = possible_moves[0]
                    print(f"No discriminating branch found. Defaulting to explore '{best_rel}'.")

                candidates_before_move = list(candidates)
                print(f"Decision: Attempting relative path '{best_rel}'.")
                rel_turn = None
                if best_rel == 'left':
                    self.tracer.pivot_into_direction('CCW');
                    rel_turn = 'CCW'
                elif best_rel == 'right':
                    self.tracer.pivot_into_direction('CW');
                    rel_turn = 'CW'

                junction_result = self.tracer.follow_until_junction()

                if junction_result is False:
                    print(f"Obstacle on path '{best_rel}'. Reverting state and returning.")
                    candidates = candidates_before_move
                    if candidate_key not in blocked_paths: blocked_paths[candidate_key] = set()
                    blocked_paths[candidate_key].add(best_rel)
                    self.tracer.drive_backward_until_junction()
                    continue
                else:
                    new_cands = []
                    for node, head in candidates:
                        new_h = self._rotate_heading(head, rel_turn) if rel_turn else head
                        nbr = self.grid_map[node][new_h]
                        if nbr: new_cands.append((nbr, new_h))
                    candidates = new_cands
                    print(f"Move successful. Advanced to new state. Candidates: {len(candidates)}")

            # --- Final Result for this attempt ---
            if len(candidates) == 1:
                node, head = candidates[0]
                print(f"✅ Localization successful! Position: {node}, Heading: {head}")
                return node, head
            else:
                print(f"❌ Attempt failed to localize uniquely.")

        # If the for loop completes without a successful return
        print("All localization attempts failed. Halting.")
        return None

    def _last_junction_before(self, pnode: str) -> str:
        """Find the last junction before a pickup or dropoff node.

        Args:
            pnode (str): The pickup or dropoff node name.

        Raises:
            ValueError: If the pickup or dropoff node has more than one non-junction neighbor.

        Returns:
            str: The name of the last junction before the pickup or dropoff node.
        """

        nbrs = [n for n in self.grid_map[pnode].values() if n]
        if len(nbrs) != 1:
            raise ValueError(f"{pnode} has {len(nbrs)} non-junction neighbors!")
        return nbrs[0]

    def perform_box_run(self, pickup: str, dropoff: str, origin: str, heading: str):
        """Perform a box run by navigating to the pickup lane, picking up the box, navigating to the dropoff lane, and dropping off the box.

        Args:
            pickup (str): The name of the pickup lane node.
            dropoff (str): The name of the dropoff lane node.
            origin (str): The name of the origin node where the run starts.
            heading (str): The initial heading of the robot ('N', 'E', 'S', 'W').

        Steps:
            1) go to the last junction before `pickup`
            2) drive forward until bump (pick up)
            3) back up to that same junction
            4) go to the last junction before `dropoff`
            5) drive forward until bump (drop off)
            6) back up to that junction

        Raises:
            ValueError: If the pickup or dropoff node is not a valid pickup or dropoff lane.
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

        print("Box run complete")