# Naming Conventions

Follows [PEP 8](https://peps.python.org/pep-0008/) with project-specific rules below.

---

## Functions

| Pattern | Use for | Example |
|---|---|---|
| `build_*` | Constructs and returns a data structure | `build_T_matrices`, `build_symbolic_joints` |
| `load_*` | Reads from disk and returns parsed data | `load_robot` |
| `get_*` | Extracts a value from an existing structure | `get_center_of_mass`, `get_inertia` |

All function names use `snake_case`.

---

## Variables

| Pattern | Use for | Example |
|---|---|---|
| `snake_case` | All variables | `joint_list`, `theta_sym` |
| `_sym` suffix | A single SymPy symbol | `theta_sym`, `d_sym` |
| `_syms` suffix | A list of SymPy symbols | `theta_syms`, `d_syms` |
| `_list` suffix | A plain Python list of items | `joint_list`, `T_list` |

---

## Recommended Renames

The following functions do not follow the `build_*` / `load_*` / `get_*` patterns and should be renamed:

| File | Current name | Recommended name | Reason |
|---|---|---|---|
| `controller.py` | `theta_syms_builder` | `build_theta_syms` | Constructs a list — use `build_*` |
| `controller.py` | `setup` | `build_simulation` | Constructs the full simulation state — use `build_*` |
| `inertia_tensor.py` | `inertia_cylinder` | `build_inertia_cylinder` | Constructs a matrix — use `build_*` |
| `inertia_tensor.py` | `inertia_hollow_cylinder` | `build_inertia_hollow_cylinder` | Constructs a matrix — use `build_*` |
| `inertia_tensor.py` | `inertia_box` | `build_inertia_box` | Constructs a matrix — use `build_*` |
| `transforms.py` | `dh_matrix` | `build_dh_matrix` | Constructs a matrix — use `build_*` |
| `forward_kinematics.py` | `build_big_T` | `build_end_effector_T` | `big_T` is ambiguous — name should describe what it builds |
| `dynamics/lagrangian.py` | `mass_matrix` | `build_mass_matrix` | Constructs a matrix — use `build_*` |
| `dynamics/lagrangian.py` | `potential_energy` | `build_potential_energy` | Constructs an expression — use `build_*` |
| `dynamics/lagrangian.py` | `gravity_vector` | `build_gravity_vector` | Constructs a vector — use `build_*` |
| `dynamics/lagrangian.py` | `coriolis_matrix` | `build_coriolis_matrix` | Constructs a matrix — use `build_*` |
| `jacobian.py` | `jacobian_column_revolute` | `build_jacobian_column_revolute` | Constructs a column vector — use `build_*` |
| `jacobian.py` | `jacobian_column_prismatic` | `build_jacobian_column_prismatic` | Constructs a column vector — use `build_*` |
| `jacobian.py` | `link_jacobian` | `build_link_jacobian` | Constructs a matrix — use `build_*` |

---

## Notes

- `get_*` functions should not construct new objects — they extract from something that already exists.
- `load_*` functions should only handle I/O (reading files, parsing YAML). No computation.
- `build_*` functions do the heavy lifting: computation, construction, symbolic math.
