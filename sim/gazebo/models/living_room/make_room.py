#!/usr/bin/env python3
"""Generates a furnished, textured room (model.sdf + textures) for stereo SLAM.

Plain-coloured Gazebo walls give stereo matching and visual odometry nothing
to lock onto, so every surface here gets a *unique* procedurally generated
texture (no repeating tiles -> no false loop closures). Deterministic seed,
so the room is identical on every machine.

    python3 sim/gazebo/models/living_room/make_room.py      (needs numpy + opencv)

The truck spawns at the origin facing +x. Room is ROOM_X by ROOM_Y metres.
"""
import os
import random

import cv2
import numpy as np

ROOM_X, ROOM_Y, ROOM_H = 6.0, 5.0, 2.4
WALL_T = 0.10
SEED = 7
HERE = os.path.dirname(os.path.abspath(__file__))
TEX_DIR = os.path.join(HERE, 'materials', 'textures')


def noise(h, w, rng, scale):
    """Smooth value noise in [0,1]."""
    small = rng.rand(max(2, h // scale), max(2, w // scale)).astype(np.float32)
    return cv2.resize(small, (w, h), interpolation=cv2.INTER_CUBIC).clip(0, 1)


def busy_texture(name, w, h, base_bgr, seed, label=None, density=1.0):
    """Wallpaper-ish texture: base colour + noise + random shapes + text."""
    rng = np.random.RandomState(seed)
    r = random.Random(seed)
    img = np.zeros((h, w, 3), np.float32)
    img[:] = base_bgr
    img *= (0.75 + 0.5 * noise(h, w, rng, 64))[..., None]
    img = img.clip(0, 255).astype(np.uint8)
    for _ in range(int(60 * density)):
        color = tuple(int(c) for c in rng.randint(0, 255, 3))
        kind = r.random()
        x, y = r.randrange(w), r.randrange(h)
        s = r.randrange(8, max(9, w // 10))
        if kind < 0.35:
            cv2.rectangle(img, (x, y), (x + s, y + r.randrange(8, s + 9)), color, -1)
        elif kind < 0.65:
            cv2.circle(img, (x, y), s // 2, color, r.choice([-1, 3, 6]))
        elif kind < 0.85:
            cv2.line(img, (x, y), (x + r.randrange(-s, s), y + r.randrange(-s, s)), color,
                     r.randrange(2, 8))
        else:
            pts = np.array([[x + r.randrange(-s, s), y + r.randrange(-s, s)] for _ in range(3)],
                           np.int32)
            cv2.fillPoly(img, [pts], color)
    if label:
        for i in range(3):
            cv2.putText(img, label, (r.randrange(10, max(11, w // 2)), r.randrange(60, h - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 2.0 + i, (20, 20, 20), 6, cv2.LINE_AA)
    # fine grain so even "flat" areas have texture
    grain = rng.randint(-18, 18, (h, w, 1))
    img = (img.astype(np.int16) + grain).clip(0, 255).astype(np.uint8)
    path = os.path.join(TEX_DIR, name + '.png')
    cv2.imwrite(path, img)
    return 'model://living_room/materials/textures/{}.png'.format(name)


def floor_texture(name, w, h, seed):
    """Wooden planks with grain and knots."""
    rng = np.random.RandomState(seed)
    r = random.Random(seed)
    img = np.zeros((h, w, 3), np.uint8)
    plank_h = h // 24
    for i in range(0, h, plank_h):
        tone = np.array([40, 90, 140]) * (0.75 + 0.5 * r.random())
        strip = np.ones((plank_h, w, 3), np.float32) * tone
        grain = noise(plank_h, w, rng, 6)[..., None] * 0.35 + 0.8
        strip *= grain
        img[i:i + plank_h] = strip[:min(plank_h, h - i)].clip(0, 255).astype(np.uint8)
        cv2.line(img, (0, i), (w, i), (20, 40, 60), 2)
        x = r.randrange(w)
        while x < w:                      # staggered butt joints
            cv2.line(img, (x, i), (x, i + plank_h), (20, 40, 60), 2)
            x += r.randrange(w // 4, w // 2)
    for _ in range(80):                   # knots
        cv2.ellipse(img, (r.randrange(w), r.randrange(h)), (r.randrange(4, 14), r.randrange(2, 6)),
                    0, 0, 360, (15, 45, 80), -1)
    grain = rng.randint(-12, 12, (h, w, 1))
    img = (img.astype(np.int16) + grain).clip(0, 255).astype(np.uint8)
    cv2.imwrite(os.path.join(TEX_DIR, name + '.png'), img)
    return 'model://living_room/materials/textures/{}.png'.format(name)


def box(name, pose, size, tex=None, color=(0.8, 0.8, 0.8)):
    if tex:
        mat = ('<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
               '<pbr><metal><albedo_map>{}</albedo_map><roughness>0.9</roughness>'
               '<metalness>0</metalness></metal></pbr></material>'.format(tex))
    else:
        mat = '<material><ambient>{0} {1} {2} 1</ambient><diffuse>{0} {1} {2} 1</diffuse></material>'.format(*color)
    geom = '<geometry><box><size>{} {} {}</size></box></geometry>'.format(*size)
    return '''
    <link name="{n}">
      <pose>{p}</pose>
      <collision name="c">{g}</collision>
      <visual name="v">{g}{m}</visual>
    </link>'''.format(n=name, p=' '.join(str(v) for v in pose), g=geom, m=mat)


def main():
    os.makedirs(TEX_DIR, exist_ok=True)
    links = []
    hx, hy = ROOM_X / 2.0, ROOM_Y / 2.0

    # Floor in 1 m x 1 m tiles so each tile gets its own unique texture.
    tile = 0
    for ix in range(int(ROOM_X)):
        for iy in range(int(ROOM_Y)):
            tex = floor_texture('floor_{}'.format(tile), 512, 512, SEED + 100 + tile)
            links.append(box('floor_{}'.format(tile),
                             (-hx + ix + 0.5, -hy + iy + 0.5, -0.005, 0, 0, 0), (1.0, 1.0, 0.01), tex))
            tile += 1

    # Walls, 1 m wide panels, each unique. (name, x, y, yaw, length)
    walls = [('north', hx + WALL_T / 2, 0.0, 0.0, ROOM_Y, (180, 140, 110)),
             ('south', -hx - WALL_T / 2, 0.0, 0.0, ROOM_Y, (120, 170, 150)),
             ('east', 0.0, -hy - WALL_T / 2, 1.5708, ROOM_X, (150, 130, 180)),
             ('west', 0.0, hy + WALL_T / 2, 1.5708, ROOM_X, (110, 160, 190))]
    for wi, (wname, wx, wy, yaw, length, base) in enumerate(walls):
        for k in range(int(round(length))):
            off = -length / 2.0 + k + 0.5
            # yaw 0: panel at x=wx spanning y; yaw 90: panel at y=wy spanning x
            px, py = (wx, off) if not yaw else (off, wy)
            tex = busy_texture('wall_{}_{}'.format(wname, k), 512, 1024, base,
                               SEED + 1000 * (wi + 1) + k, label='{}{}'.format(wname[0].upper(), k))
            links.append(box('wall_{}_{}'.format(wname, k), (px, py, ROOM_H / 2, 0, 0, yaw),
                             (WALL_T, 1.0, ROOM_H), tex))

    # Furniture: (name, x, y, yaw, size, base colour)
    furniture = [
        ('couch', -2.2, 1.6, 0.0, (0.9, 2.0, 0.8), (60, 60, 140)),
        ('bookshelf', 2.75, -1.4, 0.0, (0.35, 1.2, 1.8), (40, 80, 120)),
        ('cabinet', 1.2, 2.2, 0.0, (1.0, 0.45, 0.9), (90, 130, 160)),
        ('armchair', -2.3, -1.6, 0.4, (0.8, 0.8, 0.8), (80, 140, 80)),
        ('crate_a', 1.6, -2.0, 0.3, (0.5, 0.5, 0.5), (60, 120, 200)),
        ('crate_b', -1.5, 2.15, -0.2, (0.4, 0.6, 0.35), (200, 120, 60)),
        ('plant_pot', 2.5, 1.9, 0.0, (0.35, 0.35, 0.6), (50, 150, 50)),
    ]
    for i, (fname, fx, fy, yaw, size, base) in enumerate(furniture):
        tex = busy_texture('furn_' + fname, 512, 512, base, SEED + 500 + i, density=1.5)
        links.append(box(fname, (fx, fy, size[2] / 2, 0, 0, yaw), size, tex))

    # Coffee table: top + 4 legs (open space underneath is a good SLAM test).
    tex = busy_texture('furn_table', 512, 512, (70, 100, 150), SEED + 600, density=1.5)
    links.append(box('table_top', (2.15, 0.6, 0.45, 0, 0, 0), (1.2, 0.6, 0.04), tex))
    for j, (dx, dy) in enumerate([(0.55, 0.25), (0.55, -0.25), (-0.55, 0.25), (-0.55, -0.25)]):
        links.append(box('table_leg_{}'.format(j), (2.15 + dx, 0.6 + dy, 0.215, 0, 0, 0),
                         (0.04, 0.04, 0.43), color=(0.25, 0.15, 0.1)))

    # A solid red bin: target for the demo colour detector (detector_node).
    links.append(box('red_bin', (-2.5, -0.2, 0.25, 0, 0, 0), (0.3, 0.3, 0.5), color=(0.9, 0.02, 0.02)))

    # Posters on the walls (big distinctive features at camera height).
    posters = [('poster_n', hx - 0.01, 0.8, 0.0), ('poster_s', -hx + 0.01, -0.6, 0.0),
               ('poster_e', 0.9, -hy + 0.01, 1.5708), ('poster_w', -1.2, hy - 0.01, 1.5708)]
    for i, (pname, px, py, yaw) in enumerate(posters):
        tex = busy_texture(pname, 512, 512, (230, 230, 230), SEED + 700 + i, label=pname[-1].upper(),
                           density=2.0)
        links.append(box(pname, (px, py, 0.6, 0, 0, yaw), (0.01, 0.8, 0.6), tex))

    sdf = '''<?xml version="1.0"?>
<!-- GENERATED by make_room.py - edit that file, not this one. -->
<sdf version="1.9">
  <model name="living_room">
    <static>true</static>{links}
  </model>
</sdf>
'''.format(links=''.join(links))
    with open(os.path.join(HERE, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(HERE, 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>living_room</name><version>1.0</version>'
                '<sdf version="1.9">model.sdf</sdf>'
                '<description>Textured {}x{} m room for stereo SLAM tests.</description></model>\n'
                .format(ROOM_X, ROOM_Y))
    print('wrote living_room model ({} links) + textures in {}'.format(len(links), HERE))


if __name__ == '__main__':
    main()
