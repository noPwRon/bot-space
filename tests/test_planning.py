import numpy as np
from src.planning.path_generator import make_circle_path, make_spiral_path
from tests.conftest import assert_close


# --- Test helpers ---

def norm_it(vec):
    return vec/np.linalg.norm(vec)

def radial_distance(pos, center, normal):
    # Distance from a point to the normal axis passing through center.
    # Project the offset vector onto the plane perpendicular to normal:
    #   1. find how much of the offset points along normal (scalar projection)
    #   2. subtract that component to get the in-plane part
    #   3. return the length of the in-plane part


    n_unit = norm_it(normal)

    offset = pos - center
    normal_dist = n_unit*offset.dot(n_unit)
    return np.linalg.norm(offset - normal_dist)
    
def the_steps():
    return np.linspace(0,1,10)
    
def circle_init():
    center = np.array([2,4,5])
    radius = 5
    normal = np.array([4,5,1])

    the_path = make_circle_path(center,radius,normal)

    return the_path, center, normal, radius

def spiral_init():
    center = np.array([2,4,5])
    radius_start = 2
    radius_end = 5
    height = 6
    normal = np.array([2,5,6])
    n_turns = 4

    the_path = make_spiral_path(center,radius_start,radius_end,height, normal, n_turns)

    return the_path, center, radius_start, radius_end, height, normal, n_turns

# --- make_circle_path tests ---

def test_circle_constant_radius():
    # Every point on the circle should be exactly `radius` away from center.
    # Hint: sample the path at several t values and check the distance each time.

    test_path, center, normal, radius = circle_init()

    for t in the_steps():
        assert_close(radial_distance(test_path(t)[:3,3],center,normal),radius)


def test_circle_lies_in_plane():
    # Every point should lie in the plane defined by normal.
    # A point lies in the plane if (pos - center) · normal == 0.
    # Hint: use a non-trivial center and a non-Z normal to make this test meaningful.

    test_path, center, normal, _ = circle_init()

    for t in the_steps():
        assert_close((test_path(t)[:3,3]-center).dot(normal),0)



def test_circle_full_revolution_returns_to_start():
    # At t=1 the path should return to the same position as t=0.
    # Hint: this follows from theta = start_angle + t*2*pi completing a full cycle.
    test_path, _, _, _ = circle_init()

    assert_close(test_path(0),test_path(1))


def test_circle_center_offset():
    # Shifting center should shift every point by the same amount.
    # Hint: create two paths with the same normal and radius but different centers,
    # then check that the difference between corresponding points equals the center offset.
    offset = np.array([1,2,3])
    test_path_1, center, normal, radius = circle_init()
    test_path_2 = make_circle_path(center+offset, radius,normal)

    for t in the_steps():
        assert_close(test_path_2(t)[:3,3]-test_path_1(t)[:3,3],offset)
    


# --- make_spiral_path tests ---

def test_spiral_start_radius():
    # At t=0 the radial distance from the normal axis should equal radius_start.
    # Use the radial_distance helper above — it strips out the height component.
    
    test_path, center, radius_start, _, _, normal, _ = spiral_init()

    assert_close(radial_distance(test_path(0)[:3,3],center,normal),radius_start)

    


def test_spiral_end_radius():
    # At t=1 the radial distance from the normal axis should equal radius_end.
    
    test_path, center, _ , radius_end,_,normal,_ = spiral_init()

    assert_close(radial_distance(test_path(1)[:3,3],center,normal),radius_end)

    


def test_spiral_height_at_start():
    # At t=0 the component of position along normal (relative to center) should be zero.
    # Hint: dot product of (pos - center) with the unit normal gives the height component.

    test_path, center, _,_,_,normal,_ = spiral_init()

    n_unit = norm_it(normal)

    assert_close((test_path(0)[:3,3] - center).dot(n_unit), 0)

    


def test_spiral_height_at_end():
    # At t=1 the component along normal should equal the full height parameter.
    
    test_path, center, _,_,height,normal,_ = spiral_init()

    n_unit = norm_it(normal)

    assert_close((test_path(1)[:3,3] - center).dot(n_unit), height)
