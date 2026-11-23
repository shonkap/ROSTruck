"""OccupancyGrid -> images and nav2 map files. Python 3.6."""
import math
import os

import cv2
import numpy as np

UNKNOWN, FREE_MAX, OCCUPIED_MIN = -1, 25, 65     # OccupancyGrid cell values


def grid_array(msg):
    i = msg.info
    return np.asarray(msg.data, dtype=np.int16).reshape(i.height, i.width)


def preview_png(msg):
    """Coloured PNG for the web page (row 0 = top = max y)."""
    g = grid_array(msg)
    img = np.full(g.shape + (3,), (60, 56, 52), np.uint8)                 # unknown (BGR)
    img[(g >= 0) & (g < FREE_MAX)] = (240, 240, 236)                    # free
    img[(g >= FREE_MAX) & (g < OCCUPIED_MIN)] = (170, 170, 170)        # uncertain
    img[g >= OCCUPIED_MIN] = (40, 40, 40)                                # occupied
    ok, buf = cv2.imencode('.png', np.flipud(img))
    return buf.tobytes() if ok else b''


def trinary_image(g):
    """nav2 trinary image (top row first): white free, black occupied, grey unknown."""
    img = np.full(g.shape, 205, np.uint8)
    img[(g >= 0) & (g < FREE_MAX)] = 254
    img[g >= OCCUPIED_MIN] = 0
    return np.flipud(img)


def save_nav2_map(msg, base):
    """Write <base>.png + <base>.yaml (nav2 map_server format). Returns a summary line."""
    base = os.path.expanduser(base)
    d = os.path.dirname(base)
    if d and not os.path.isdir(d):
        os.makedirs(d)
    g = grid_array(msg)
    cv2.imwrite(base + '.png', trinary_image(g))
    i, q = msg.info, msg.info.origin.orientation
    yaw = 2.0 * math.atan2(q.z, q.w)
    with open(base + '.yaml', 'w') as f:
        f.write('image: {}\nmode: trinary\nresolution: {}\norigin: [{}, {}, {}]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n'.format(
                    os.path.basename(base) + '.png', i.resolution,
                    i.origin.position.x, i.origin.position.y, yaw))
    cell = i.resolution ** 2
    return 'saved {0}.png/.yaml: {1}x{2} cells @ {3:.3f} m, known {4:.1f} m^2 (free {5:.1f}, occupied {6:.1f})'.format(
        base, i.width, i.height, i.resolution, (g >= 0).sum() * cell,
        ((g >= 0) & (g < FREE_MAX)).sum() * cell, (g >= OCCUPIED_MIN).sum() * cell)
