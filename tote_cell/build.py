"""Build the tote-handling cell: fixed scene + KLT tote stacks + Unitree G1.

    python -m tote_cell.build --view      # open the cell in the MuJoCo viewer
"""

import argparse
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SCENE_XML = ROOT / "tote_cell" / "scene.xml"
G1_XML = (ROOT / "third_party" / "unitree-g1-locomotion" / "unitree_g1_description"
          / "urdf_xml" / "g1.xml")

# VDA KLT outer dimensions (length, width, height) in metres
KLT_4147 = (0.396, 0.297, 0.147)
KLT_3147 = (0.297, 0.198, 0.147)
WALL = 0.004  # wall half-thickness

# Stacks: (name prefix, stack site, tote size, material, tote mass in kg, number of totes)
STACKS = [
    ("klt4147", "stack_a", KLT_4147, "klt", 0.9, 3),
    ("klt3147", "stack_b", KLT_3147, "klt_small", 0.5, 3),
]

# Nominal standing pose of the mjlab velocity policy (default_joint_pos in its ONNX metadata),
# in actuator order of g1.xml
G1_DEFAULT_JOINT_POS = np.array([
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,    # left leg
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,    # right leg
    0.0, 0.0, 0.0,                           # waist
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,       # left arm
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,      # right arm
])
ROBOT_PREFIX = "g1/"


def add_tote(spec, name, pos, size, material, mass):
    """Open-top box tote with a free joint, handle sites on the short sides and a centre site."""
    length, width, height = size
    hx, hy, hz = length / 2, width / 2, height / 2
    body = spec.worldbody.add_body(name=name, pos=pos)
    body.add_freejoint(name=f"{name}_free")
    box = mujoco.mjtGeom.mjGEOM_BOX
    panels = {
        "bottom": ([hx, hy, WALL], [0, 0, WALL]),
        "wall_px": ([WALL, hy, hz], [hx - WALL, 0, hz]),
        "wall_nx": ([WALL, hy, hz], [-hx + WALL, 0, hz]),
        "wall_py": ([hx, WALL, hz], [0, hy - WALL, hz]),
        "wall_ny": ([hx, WALL, hz], [0, -hy + WALL, hz]),
    }
    # Spread the tote mass over the panels by area
    areas = {k: 4 * (s[0] * s[1] + s[1] * s[2] + s[0] * s[2]) for k, (s, _) in panels.items()}
    total = sum(areas.values())
    for part, (half, p) in panels.items():
        geom = body.add_geom(name=f"{name}_{part}", type=box, size=half, pos=p,
                             mass=mass * areas[part] / total, material=material)
        geom.friction = [0.9, 0.01, 0.001]
        geom.condim = 4
    body.add_site(name=f"{name}_center", pos=[0, 0, hz], size=[0.01, 0.01, 0.01])
    body.add_site(name=f"{name}_handle_px", pos=[hx, 0, 0.75 * height], size=[0.012, 0.012, 0.012],
                  rgba=[1, 0.5, 0, 1])
    body.add_site(name=f"{name}_handle_nx", pos=[-hx, 0, 0.75 * height], size=[0.012, 0.012, 0.012],
                  rgba=[1, 0.5, 0, 1])
    return body


def tote_names():
    return [f"{prefix}_{i}" for prefix, *_rest, count in STACKS for i in range(count)]


def build_spec(with_robot=True):
    spec = mujoco.MjSpec.from_file(str(SCENE_XML))

    # Tote stacks: positions come from the stack sites on the pallets
    scene_model = spec.compile()
    scene_data = mujoco.MjData(scene_model)
    mujoco.mj_forward(scene_model, scene_data)
    for prefix, site, size, material, mass, count in STACKS:
        base = scene_data.site(site).xpos.copy()
        for i in range(count):
            pos = base + np.array([0.0, 0.0, i * (size[2] + 0.002)])
            add_tote(spec, f"{prefix}_{i}", pos, size, material, mass)

    if with_robot:
        if not G1_XML.exists():
            raise FileNotFoundError(
                f"{G1_XML} not found. Run: git submodule update --init")
        g1 = mujoco.MjSpec.from_file(str(G1_XML))
        spec.attach(g1, prefix=ROBOT_PREFIX, frame="robot_mount")
    return spec


def robot_home_qpos(model):
    """Full qpos with totes at rest and the G1 in its nominal pose, feet on the floor."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    free = model.joint(f"{ROBOT_PREFIX}floating_base_joint")
    qadr = free.qposadr[0]
    data.qpos[qadr:qadr + 7] = [0, 0, 1.0, 1, 0, 0, 0]
    data.qpos[qadr + 7:qadr + 7 + 29] = G1_DEFAULT_JOINT_POS

    # Lower the robot until its lowest collision geom touches the floor
    mujoco.mj_kinematics(model, data)
    robot_root = model.body(f"{ROBOT_PREFIX}pelvis").id
    lowest = np.inf
    for g in range(model.ngeom):
        if model.body_rootid[model.geom_bodyid[g]] != robot_root or model.geom_contype[g] == 0:
            continue
        center, half = model.geom_aabb[g][:3], model.geom_aabb[g][3:]
        corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * half + center
        world = data.geom_xpos[g] + corners @ data.geom_xmat[g].reshape(3, 3).T
        lowest = min(lowest, world[:, 2].min())
    data.qpos[qadr + 2] -= lowest
    return data.qpos.copy()


def build_model(with_robot=True):
    spec = build_spec(with_robot)
    model = spec.compile()
    if with_robot:
        spec.add_key(name="home", qpos=robot_home_qpos(model))
        model = spec.compile()
    return model


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--view", action="store_true", help="open the MuJoCo viewer")
    parser.add_argument("--no-robot", action="store_true", help="build the cell without the G1")
    args = parser.parse_args()

    model = build_model(with_robot=not args.no_robot)
    print(f"tote cell: {model.nbody} bodies, {model.nu} actuators, totes: {', '.join(tote_names())}")
    if args.view:
        import mujoco.viewer
        data = mujoco.MjData(model)
        if model.nkey:
            mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.viewer.launch(model, data)


if __name__ == "__main__":
    main()
