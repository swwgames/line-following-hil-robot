
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
    def __init__(self, iterable=None):
        if iterable is None:
            self._data = []
        else:
            self._data = list(iterable)

    def append(self, item):
        self._data.append(item)

    def appendleft(self, item):
        self._data.insert(0, item)

    def popleft(self):
        if not self._data:
            raise IndexError("popleft from an empty deque")
        return self._data.pop(0)

    def __len__(self):
        return len(self._data)


class TomTom:
    def __init__(self, tracer):
        self.tracer = tracer
        self.grid_map = grid_map

    def plan_route(self, goal: str):
        """
        0–1 BFS on (node,heading) states to minimize number of 90° turns.
        Returns list of node names from current_node to goal (inclusive),
        or [] if unreachable.
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

    def _relative_turn(self, *args):
        """
        Compute minimal 90° turn(s) to go from `current`→`desired`.
        Backward-compatible:
          _relative_turn(desired)      uses self.heading as current
          _relative_turn(current, desired)
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

    def navigate_to(self, origin: str, goal: str, start_heading: str = 'N'):
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
            print(
                f"At {self.current_node}, heading={self.heading} → next={next_node}, desired={desired}, turns={turns}")

            # 3) pivot if needed
            if turns is not None:
                for t in (turns if isinstance(turns, tuple) else (turns,)):
                    self.tracer.pivot_into_direction(direction=t)

                # now we’re aligned with the branch
                self.heading = desired

            # 4) drive straight to the next junction
            if next_node.startswith('P'):
                self.tracer.drive_forward_until_bump()
            else:
                junction = self.tracer.follow_until_junction()

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
                        self.tracer.robot.stop()
                        return

            # 5) update position
            self.current_node = next_node
            print(f" Arrived at {self.current_node}, heading={self.heading}")

    def _rotate_heading(self, heading: str, turn: str) -> str:
        """Rotate a compass heading by 'CW' or 'CCW'."""
        order = ['N', 'E', 'S', 'W']
        i = order.index(heading)
        if turn == 'CW':
            return order[(i + 1) % 4]
        if turn == 'CCW':
            return order[(i - 1) % 4]
        return heading

    def _sense_junction(self):
        """
        Return what the robot sees at the current junction:
          'front','left','right' → True if that branch exists.
        """
        flags = {}
        # front = any line under front array?
        flags['front'] = any(self.tracer.robot.read_line_sensors('front'))
        # left  = any line under left array?
        flags['left'] = any(self.tracer.robot.read_line_sensors('left'))
        # right = any line under right array?
        flags['right'] = any(self.tracer.robot.read_line_sensors('right'))
        return flags

    def _match_observation(self, node: str, heading: str, obs) -> bool:
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

    def locate_self(self, max_steps: int = 50, known_heading = None):
        """
        Drive through junctions until the robot deduces its exact
        (node, heading).  If you know your initial heading, pass it
        in via known_heading to filter candidates immediately.
        Returns that pair, or None if inconclusive.
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
        """
        In our grid every P-lane node has exactly one neighbor
        that is a junction. Return that neighbor.
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