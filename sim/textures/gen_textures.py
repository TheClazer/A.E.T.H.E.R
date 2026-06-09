"""
Texture generator for the tunnel_walls model (U4 world v2).

Writes two deterministic 512x512 PNGs into
sim/models/tunnel_walls/materials/textures/:

  feature_rich.png  high-contrast checker + gaussian noise + speckles —
                    dense corner food for the KLT tracker on every panel
  bare.png          near-uniform grey, noise well below the FAST threshold —
                    the feature-starvation stretch (x = 120..140 m)

Seeded RNG so the world (and therefore every recorded bag) is reproducible.
Requires numpy + Pillow only. Run from anywhere:  python3 sim/textures/gen_textures.py
"""
import os

import numpy as np
from PIL import Image

SIZE = 512
SEED = 1729
OUT_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', 'models', 'tunnel_walls', 'materials', 'textures'))


def feature_rich(rng):
    """High-contrast checkerboard + noise + speckles (grayscale -> RGB)."""
    sq = 32                                          # 16x16 squares per tile
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    checker = (((xx // sq) + (yy // sq)) % 2).astype(np.float32)
    base = 40.0 + 175.0 * checker                    # dark/bright squares
    noise = rng.normal(0.0, 22.0, (SIZE, SIZE))      # per-pixel texture
    # sparse +/-120 speckles so corners exist inside the squares too
    mask = rng.random((SIZE, SIZE)) < 0.02
    sign = rng.choice(np.array([-120.0, 120.0]), (SIZE, SIZE))
    img = np.clip(base + noise + mask * sign, 0.0, 255.0).astype(np.uint8)
    return np.stack([img, img, img], axis=-1)


def bare(rng):
    """Near-uniform mid-grey: essentially nothing for the tracker to grab."""
    base = np.full((SIZE, SIZE), 128.0, dtype=np.float32)
    noise = rng.normal(0.0, 1.5, (SIZE, SIZE))       # < FAST threshold
    img = np.clip(base + noise, 0.0, 255.0).astype(np.uint8)
    return np.stack([img, img, img], axis=-1)


def main():
    rng = np.random.default_rng(SEED)
    os.makedirs(OUT_DIR, exist_ok=True)
    for name, fn in (('feature_rich.png', feature_rich), ('bare.png', bare)):
        path = os.path.join(OUT_DIR, name)
        Image.fromarray(fn(rng)).save(path)
        print(f'wrote {path}')


if __name__ == '__main__':
    main()
