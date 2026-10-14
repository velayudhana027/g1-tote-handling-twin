"""Render the tote cell with the G1 in its home pose to docs/media/.

Headless: MUJOCO_GL=egl python scripts/render_cell.py   (or MUJOCO_GL=osmesa)
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tote_cell.build import build_model  # noqa: E402

OUT = ROOT / "docs" / "media"


def save_png(pixels, path):
    try:
        from PIL import Image
        Image.fromarray(pixels).save(path)
    except ImportError:  # fall back to imageio if Pillow is missing
        import imageio.v3 as iio
        iio.imwrite(path, pixels)


def main():
    model = build_model(with_robot=True)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)

    OUT.mkdir(parents=True, exist_ok=True)
    renderer = mujoco.Renderer(model, height=800, width=1280)
    for camera in ("overview", "front"):
        renderer.update_scene(data, camera=camera)
        pixels = renderer.render()
        path = OUT / f"cell_{camera}.png"
        save_png(np.asarray(pixels), path)
        print(f"wrote {path}")


if __name__ == "__main__":
    main()
