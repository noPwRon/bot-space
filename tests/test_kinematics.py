# Reference: pytest basics — https://docs.pytest.org/en/stable/getting-started.html
# Reference: Craig, "Introduction to Robotics", Ch. 3 (DH transforms)

import numpy as np
from src.kinematics.transforms import build_dh_matrix, build_T_matrices

def numbify(sym_result):
    return np.array(sym_result.evalf(),dtype=float)

def testCheck(num_result,expected_result):
    assert np.allclose(num_result,expected_result)


# Import numpy for numerical comparisons and the functions you want to test.
# Import build_dh_matrix and build_T_matrices from the transforms module.


# --- Test 1: identity transform ---
# A joint with a=0, alpha=0, d=0, theta=0 should produce the 4x4 identity matrix.
# This is the "do nothing" case — no rotation, no translation.
# Compute the result and assert it equals numpy's identity matrix (np.eye(4)).
# Hint: SymPy matrices need to be converted to floats before comparing numerically —
# look up sympy.Matrix.evalf() and numpy's np.array().

def test_eyeTransform():
    sym_result =build_dh_matrix(0,0,0,0)
    
    num_result = numbify(sym_result=sym_result) 

    expected_result = np.eye(4)

    testCheck(num_result,expected_result)

    

# --- Test 2: pure translation ---
# A joint with a=1, alpha=0, d=0, theta=0 should translate exactly 1 unit along x.
# Check that the top-right element of the resulting T matrix (row 0, column 3) equals 1.
# Everything else in the rotation part should still look like an identity.

def test_pureTranslation():
    sym_result = build_dh_matrix(1,0,0,0)

    num_result = numbify(sym_result=sym_result)

    expected_result = np.eye(4)
    expected_result[0,3] = 1

    testCheck(num_result,expected_result)

# --- Test 3: pure rotation ---
# A joint with a=0, alpha=0, d=0, theta=pi/2 should rotate 90 degrees about z.
# After a 90-degree rotation, cos(theta)=0 and sin(theta)=1.
# Check the [0,0] and [1,0] elements of the matrix match those expected values.
# Import pi from sympy (not math) since build_dh_matrix uses symbolic trig.



# --- Test 4: chaining two joints ---
# Build a two-joint arm where each joint translates 1 unit along x (a=1) and does no rotation.
# The final position of the end-effector (top-right column of the last cumulative T matrix)
# should be [2, 0, 0] — two links of length 1 laid flat.
# Hint: you need the *cumulative* transform, not just the last T matrix.
#       Multiply the two T matrices together to get the full transform from base to tip.