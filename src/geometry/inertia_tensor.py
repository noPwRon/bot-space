import sympy as smp

# Reference: references/inertia_tensors.ipynb


def inertia_cylinder(m, r, L):
    # Solid cylinder, symmetry axis along z.
    Ixx = Iyy = smp.Rational(1,12)*m*(3*r**2 + L**2)
    Izz = smp.Rational(1,2)*m*r**2
    return smp.diag(Ixx,Iyy,Izz)

def inertia_hollow_cylinder(m, r_outer, r_inner, L):
    # Hollow cylinder (tube), symmetry axis along z.
    Ixx = Iyy = smp.Rational(1,12)*m*(3*(r_outer**2 + r_inner**2) + L**2)
    Izz = smp.Rational(1,2)*m*(r_outer**2 + r_inner**2)
    return smp.diag(Ixx,Iyy,Izz)


def inertia_box(m, w, h, L):
    # Rectangular box: w along x, h along y, L along z.
    Ixx = smp.Rational(1,12)*m*(h**2 + L**2)
    Iyy = smp.Rational(1,12)*m*(w**2 + L**2)
    Izz = smp.Rational(1,12)*m*(w**2 + h**2)
    return smp.diag(Ixx,Iyy,Izz)
    


def get_center_of_mass(joint):
    # Returns the CoM position as a sympy Matrix [0, 0, L/2] in the joint frame.
    # Assumes mass is uniformly distributed along the primary (z) axis.
    L = joint["length"]
    if joint["type"] == "prismatic":
        offset = joint["d"]
        return smp.Matrix([0,0,smp.Rational(1,2) *(L + offset)])
    else:
        return smp.Matrix([0, 0, smp.Rational(1, 2) * L])


def get_inertia(joint):
    # Reads joint["geometry"] and dispatches to the appropriate function.
    # Returns a 3x3 diagonal sympy Matrix — the inertia tensor about the CoM.

    if (joint["geometry"]) == "box":
        return inertia_box(joint["mass"],joint["width"],joint["height"],joint["length"])
    elif (joint["geometry"]) == "cylinder":
        return inertia_cylinder(joint["mass"],joint["radius"],joint["length"])
    elif (joint["geometry"]) == "hollow_cylinder":
        return inertia_hollow_cylinder(joint["mass"], joint["radius"], joint["inner_radius"], joint["length"])
    else:
        raise ValueError(f"Unknown geometry: {joint['geometry']} Must be cylinder, hollow_cylinder, or box")
