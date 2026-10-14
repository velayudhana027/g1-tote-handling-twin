"""Success checks for the tote-to-conveyor task, computed from simulator state."""

import numpy as np

from tote_cell.build import ROBOT_PREFIX, tote_names


def in_box_site(model, data, site, point):
    """True if a world point lies inside a box-shaped site."""
    sid = model.site(site).id
    local = data.site_xmat[sid].reshape(3, 3).T @ (np.asarray(point) - data.site_xpos[sid])
    return bool(np.all(np.abs(local) <= model.site_size[sid]))


def tote_speed(data, tote):
    """Linear speed of a tote (com-based velocity, world frame) in m/s."""
    return float(np.linalg.norm(data.body(tote).cvel[3:]))


def tote_placed(model, data, tote, zone="place_zone", max_speed=0.05):
    """A tote counts as placed when its centre is inside the zone and it has come to rest."""
    centre = data.site(f"{tote}_center").xpos
    return in_box_site(model, data, zone, centre) and tote_speed(data, tote) < max_speed


def tote_dropped(data, tote, floor_height=0.1):
    """A tote on the floor was dropped."""
    return bool(data.body(tote).xpos[2] < floor_height)


def remaining_on_stacks(model, data, stack_radius=0.35):
    """Totes still within reach of their pallet site."""
    remaining = []
    for tote in tote_names():
        stack = "stack_a" if tote.startswith("klt4147") else "stack_b"
        offset = data.body(tote).xpos[:2] - data.site(stack).xpos[:2]
        if np.linalg.norm(offset) < stack_radius and data.body(tote).xpos[2] > 0.4:
            remaining.append(tote)
    return remaining


def robot_fallen(data, min_pelvis_height=0.45):
    """The humanoid has fallen when its pelvis drops below a height threshold."""
    return bool(data.body(f"{ROBOT_PREFIX}pelvis").xpos[2] < min_pelvis_height)
