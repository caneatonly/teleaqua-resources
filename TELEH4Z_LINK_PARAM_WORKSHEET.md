# TeleH4Z Link Physical Parameter Worksheet

Source model:
- `/home/user/external/models/teleh4z/model.sdf`

Scope in this worksheet:
- `base_link`
- `buoyancy_shell_link`
- `arm_left_link`
- `arm_right_link`
- `waterprop_left_link`
- `waterprop_right_link`
- `waterprop_front_link`
- `waterprop_back_link`

## Units

| Field | Unit |
|---|---|
| pose xyz | m |
| pose rpy | rad |
| mass | kg |
| inertia terms | kg*m^2 |
| collision size | m |

## Fill Rules

| Item | What to fill |
|---|---|
| `mass` | Real mass of this link only |
| `inertial_pose_xyz/rpy` | COM pose relative to this link frame |
| `ixx iyy izz ixy ixz iyz` | Inertia tensor about COM, expressed in the link inertial frame |
| `collision_*` | Physical collision proxy to keep in SDF |
| `source` | CAD / weighing / estimate / spreadsheet / test |
| `status` | TODO / measured / estimated / verified |

## Link Inventory

| Link | Parent | Joint | Current link pose in SDF | Current inertial status |
|---|---|---|---|---|
| `base_link` | model root | canonical | `0 0 0 0 0 0` | already has non-placeholder values |
| `buoyancy_shell_link` | `base_link` | `buoyancy_shell_joint` fixed | `0 0 0 0 0 0` | already has non-placeholder values |
| `arm_left_link` | `base_link` | `arm_left_joint` revolute | `0.0219 0.0445 0.0705 0 0 0` | placeholder inertial |
| `arm_right_link` | `base_link` | `arm_right_joint` revolute | `0.0219 -0.0445 0.0705 0 0 0` | placeholder inertial |
| `waterprop_left_link` | `base_link` | `waterprop_left_joint` revolute | `-0.155 0.036 -0.007866 0 0 0` | placeholder inertial |
| `waterprop_right_link` | `base_link` | `waterprop_right_joint` revolute | `-0.155 -0.036 -0.007866 0 0 0` | placeholder inertial |
| `waterprop_front_link` | `base_link` | `waterprop_front_joint` revolute | `0.1139 0 0.04 0 1.57079632679 0` | placeholder inertial |
| `waterprop_back_link` | `base_link` | `waterprop_back_joint` revolute | `-0.10643 0 0.04 0 1.57079632679 0` | placeholder inertial |

---

## 1. base_link

Current SDF reference:
- link pose: `0 0 0 0 0 0`
- inertial pose: `0 0 -0.015 0 0 0`
- current mass: `2.25`
- current inertia: `ixx=0.00166875 iyy=0.01734375 izz=0.018075 ixy=0 ixz=0 iyz=0`
- current collision: box at `0 0 -0.03 0 0 0`, size `0.30 0.08 0.05`

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `2.25` |  | bare hull/body mass only, or body plus fixed accessories if intentionally merged |
| inertial_pose_xyz | `0 0 -0.015` |  | COM position in `base_link` |
| inertial_pose_rpy | `0 0 0` |  | usually `0 0 0` unless inertial frame rotated |
| ixx | `0.00166875` |  |  |
| iyy | `0.01734375` |  |  |
| izz | `0.018075` |  |  |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | `base_buoyancy_collision` |  | rename only if needed |
| collision_pose_xyz | `0 0 -0.03` |  |  |
| collision_pose_rpy | `0 0 0` |  |  |
| collision_geometry | `box` |  | box / mesh / compound proxy |
| collision_size_or_mesh | `0.30 0.08 0.05` |  | if mesh, write mesh path |
| source |  |  |  |
| status |  |  |  |

## 2. buoyancy_shell_link

Current SDF reference:
- link pose: `0 0 0 0 0 0` relative to `base_link`
- inertial pose: not set in SDF
- current mass: `0.5`
- current inertia: `ixx=0.0011360133 iyy=0.0020696800 izz=0.0027333333 ixy=0 ixz=0 iyz=0`
- current collision: box at `0 0 0.05 0 0 0`, size `0.24 0.16 0.0408854167`

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `0.5` |  | shell mass only |
| inertial_pose_xyz | not set |  | if omitted in SDF, defaults to `0 0 0` |
| inertial_pose_rpy | not set |  | if omitted in SDF, defaults to `0 0 0` |
| ixx | `0.0011360133` |  |  |
| iyy | `0.0020696800` |  |  |
| izz | `0.0027333333` |  |  |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | `shell_buoyancy_collision` |  |  |
| collision_pose_xyz | `0 0 0.05` |  |  |
| collision_pose_rpy | `0 0 0` |  |  |
| collision_geometry | `box` |  |  |
| collision_size_or_mesh | `0.24 0.16 0.0408854167` |  | this strongly affects buoyancy proxy shape |
| source |  |  |  |
| status |  |  |  |

## 3. arm_left_link

Current SDF reference:
- link pose: `0.0219 0.0445 0.0705 0 0 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to `arm_left_link` frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  |  |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional, but recommended if arm contact matters |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | box / cylinder / mesh / compound proxy |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## 4. arm_right_link

Current SDF reference:
- link pose: `0.0219 -0.0445 0.0705 0 0 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to `arm_right_link` frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  |  |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional, but recommended if arm contact matters |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | box / cylinder / mesh / compound proxy |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## 5. waterprop_left_link

Current SDF reference:
- link pose: `-0.155 0.036 -0.007866 0 0 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none
- plugin family: bidirectional water thruster

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to water thruster link frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  | rotor-axis inertia matters |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional if you want thruster contact/obstruction |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | cylinder / mesh proxy are common choices |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## 6. waterprop_right_link

Current SDF reference:
- link pose: `-0.155 -0.036 -0.007866 0 0 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none
- plugin family: bidirectional water thruster

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to water thruster link frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  | rotor-axis inertia matters |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional if you want thruster contact/obstruction |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | cylinder / mesh proxy are common choices |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## 7. waterprop_front_link

Current SDF reference:
- link pose: `0.1139 0 0.04 0 1.57079632679 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none
- plugin family: bidirectional water thruster

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to water thruster link frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  | rotor-axis inertia matters |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional if you want thruster contact/obstruction |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | cylinder / mesh proxy are common choices |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## 8. waterprop_back_link

Current SDF reference:
- link pose: `-0.10643 0 0.04 0 1.57079632679 0` relative to `base_link`
- current mass: `1e-4`
- current inertia: `ixx=1e-8 iyy=1e-8 izz=1e-8 ixy=0 ixz=0 iyz=0`
- current collision: none
- plugin family: bidirectional water thruster

| Field | Current SDF | Fill value | Notes |
|---|---|---|---|
| mass | `1e-4` |  | replace placeholder |
| inertial_pose_xyz | not set |  | COM relative to water thruster link frame |
| inertial_pose_rpy | not set |  |  |
| ixx | `1e-8` |  |  |
| iyy | `1e-8` |  |  |
| izz | `1e-8` |  | rotor-axis inertia matters |
| ixy | `0` |  |  |
| ixz | `0` |  |  |
| iyz | `0` |  |  |
| collision_name | none |  | optional if you want thruster contact/obstruction |
| collision_pose_xyz | none |  |  |
| collision_pose_rpy | none |  |  |
| collision_geometry | none |  | cylinder / mesh proxy are common choices |
| collision_size_or_mesh | none |  |  |
| source |  |  |  |
| status |  |  |  |

## Suggested Fill Order

1. Fill `base_link` and `buoyancy_shell_link` mass + COM first.
2. Fill both arms, because arm mass distribution changes folded/deployed inertia.
3. Fill all four water thrusters with consistent measurement assumptions.
4. Review whether any fixed accessories were merged into `base_link`.
5. After values are stable, copy them into `model.sdf`.

## Merge Decision Log

| Accessory not modeled here | Merge into which link | Decision | Notes |
|---|---|---|---|
| camera assembly |  |  |  |
| waterprop mounts |  |  |  |
| wires / ESC / fasteners |  |  |  |
| sealed electronics inside hull |  |  |  |

