import heapq
import struct

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

class Deque:
    """A list-like sequence optimized for data accesses near its endpoints."""
    def __init__(self, iterable=None):
        if iterable is None:
            self._data = []
        else:
            self._data = list(iterable)

    def append(self, item):
        """Add an element to the right side of the deque."""
        self._data.append(item)

    def appendleft(self, item):
        """Add an element to the left side of the deque."""
        self._data.insert(0, item)

    def popleft(self):
        """"Remove and return the leftmost element."""
        if not self._data:
            raise IndexError("popleft from an empty deque")
        return self._data.pop(0)

    def __len__(self):
        return len(self._data)


class TomTom:
    def __init__(self, tracer):
        """Initialize the TomTom navigation system with a tracer and a grid map.

        Args:
            tracer (obj): An instance of the LineTracer class for driving.
        """
        self.tracer = tracer
        self.grid_map = grid_map
        self.header = None
        self.heading = None
        self.current_node = None

        self.com = tracer.robot.com



    def plan_route(self, goal: str) -> list[str]:
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
        inf = float('inf')
        dist = {(n, h): inf for n in self.grid_map for h in headings}
        dist[start] = 0

        # to reconstruct the path
        prev = {}

        dq = Deque([start])
        while dq:
            u_node, u_head = dq.popleft()
            u_cost = dist[(u_node, u_head)]
            # explore all available moves
            for abs_dir, v in self.grid_map[u_node].items():
                if v is None:
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
        if best_cost == inf:
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

    def plan_route_astar(self, goal: str) -> list[str]:
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
        inf = float('inf')
        g_score = {(n, h): inf for n in self.grid_map for h in headings}
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
                if v is None:
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

    def _relative_turn(self, *args):
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
        return 'CW', 'CW'

    def navigate_to(self, origin: str, goal: str, start_heading: str = 'N'):
        """Navigate from the origin node to the goal node, starting with a given heading.

        Args:
            origin (str): The name of the starting node.
            goal (str): The name of the goal node.
            start_heading (str): The initial heading of the robot, default is 'N'.

        Raises:
            RuntimeError: If the next node is not found in the grid map.
        """

        self.current_node = origin
        self.heading = start_heading

        path = self.plan_route_astar(goal)
        if not path:
            print(f"No path from {origin} to {goal}")
            return

        format_str = '2s' * len(path)
        encoded_data = [s.encode('utf-8') for s in path]
        packed = struct.pack(format_str, *encoded_data)
        self.com.send_packet_to_socket(b'r', packed)

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
                    # treat as obstacle—remove next_node and replan
                    print(f"Obstacle detected before junction {next_node}. Removing {next_node} from map and replanning.")
                    for node, nbrs in self.grid_map.items():
                        for d, n in nbrs.items():
                            if n == next_node:
                                self.grid_map[node][d] = None
                    self.tracer.drive_backward_until_junction()
                    return self.navigate_to(self.current_node, goal, self.heading)
                    
                if junction is None:
                    return

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
            packed = struct.pack('2s', self.current_node)
            print(f"Sending update to robot: {packed}")
            self.com.send_packet_to_socket(b'n', packed)

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

    def _sense_junction(self) -> dict[str, bool]:
        """Sense the junction by checking the line sensors. This is to validate the robot's position after each junction.

        Returns:
            Dict[str,bool]: A dictionary with keys 'front', 'left', 'right' indicating whether each direction has a line detected (True) or not (False).
        """

        flags = {'front': any(self.tracer.robot.read_line_sensors('front')),
                 'left': any(self.tracer.robot.read_line_sensors('left')),
                 'right': any(self.tracer.robot.read_line_sensors('right'))}
        return flags

    def _match_observation(self, node: str, heading: str, obs: dict[str, bool]) -> bool:
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

    def locate_self(self, max_steps: int = 50, known_heading: str = None) -> tuple[str, str] | None:
        """Locate the robot's current position and heading in the grid map by following branches and observing junctions.

        Args:
            max_steps (int): The maximum number of steps to take while trying to localize, default is 50.
            known_heading (Optional[str]): The current heading of the robot ('N', 'E', 'S', 'W') or None if unknown.

        Returns:
            Optional[Tuple[str,str]]: A tuple containing the node name and heading if successfully localized, or None if unable to localize uniquely.
        """

        # 1) initialize candidate set
        if known_heading in ('N', 'E', 'S', 'W'):
            # only these headings
            candidates = [(node, known_heading)
                          for node in self.grid_map]
        else:
            # all four possible headings at every node
            candidates = [
                (node, h)
                for node in self.grid_map
                for h in ('N', 'E', 'S', 'W')
            ]
        steps = 0

        while len(candidates) > 1 and steps < max_steps:
            steps += 1

            # drive until the next junction
            self.tracer.follow_until_junction()

            # sense which branches exist here
            obs = self._sense_junction()
            print(f"[Step {steps}] saw {obs}, candidates={len(candidates)}")

            # 2) eliminate any candidate whose map‐pattern are not equal to obs
            candidates = [
                (node, head)
                for node, head in candidates
                if self._match_observation(node, head, obs)
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
                    1 for (n, h) in candidates
                    if self.grid_map[n][
                        (h if rel == 'front' else
                         self._rotate_heading(h, 'CCW') if rel == 'left' else
                         self._rotate_heading(h, 'CW'))
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
            for node, head in candidates:
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
            print(f"Located at {node}, heading={head}")
            return node, head
        else:
            print("Unable to localize uniquely")
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
            self.tracer.robot.stop()
            self.com.client_socket.close()
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