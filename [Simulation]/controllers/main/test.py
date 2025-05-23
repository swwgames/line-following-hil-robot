# main.py
from types import SimpleNamespace
from tomtom import TomTom, grid_map  # your TomTom class + grid_map

def make_mock_navigator(start_node, start_heading):
    nav = TomTom(tracer=None)
    nav.grid_map     = grid_map
    nav.current_node = start_node
    nav.heading      = start_heading

    # 1) fake tracer
    fake_tracer = SimpleNamespace()
    fake_tracer.follow_until_junction = lambda turn_direction=None: None

    # 2) sense_junction remains on nav
    def sense_junction():
        obs = {}
        for rel in ('front','left','right'):
            if rel == 'front':
                abs_dir = nav.heading
            elif rel == 'left':
                abs_dir = nav._rotate_heading(nav.heading,'CCW')
            else:
                abs_dir = nav._rotate_heading(nav.heading,'CW')
            obs[rel] = (grid_map[nav.current_node][abs_dir] is not None)
        return obs
    nav._sense_junction = sense_junction

    # 3) pivotThat advances both nav._state and tracer
    def pivot(direction):
        new_h = nav._rotate_heading(nav.heading, direction)
        nxt   = grid_map[nav.current_node][new_h]
        if nxt is None:
            raise RuntimeError(f"bad pivot {nav.current_node}→{direction}")
        nav.heading      = new_h
        nav.current_node = nxt

    # attach pivot to the tracer mock, *not* to nav itself
    fake_tracer.pivot_into_direction = pivot

    nav.tracer = fake_tracer
    return nav

def test_locator():
    failures = []
    # test **every** node & heading
    for node in grid_map:
        for heading in ('N','E','S','W'):
            nav = make_mock_navigator(node,heading)
            result = nav.locate_self(max_steps=20, known_heading=heading)
            if result != (node,heading):
                failures.append(((node,heading), result))

    if failures:
        print("❌ Some localization tests failed:")
        for (exp_node,exp_h), got in failures:
            print(f"   Expected ({exp_node},{exp_h}) → got {got}")
    else:
        print("✅ All localization tests passed!")

if __name__ == "__main__":
    test_locator()
