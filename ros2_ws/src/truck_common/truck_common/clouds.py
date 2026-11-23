"""2D obstacle points <-> sensor_msgs/PointCloud2 (x, y float32; z = 0). Python 3.6.

The guards publish what they currently see as obstacle points in base_link
(safety/<source>/points) so the navigator can plan around things that aren't
in the SLAM map yet - a cat, a person, a box someone just put down.
"""
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from truck_common.ros import u8

_FIELDS = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
           PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
           PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]


def thin(points, cell=0.05, min_hits=1):
    """One point per cell x cell square (stereo gives thousands); cells with
    fewer than min_hits points are dropped as noise."""
    if len(points) == 0:
        return points
    keys = np.floor(points / cell).astype(np.int64)
    _, idx, counts = np.unique(keys, axis=0, return_index=True, return_counts=True)
    return points[np.sort(idx[counts >= min_hits])]


def solid_objects(points, link=0.03, min_size=0.01):
    """Drop things only a few mm in size. Points closer than `link` (3 cm,
    about the lidar's spacing at 1-2 m) are one object; an object counts if
    its points span at least min_size (1 cm). A lone return or a speck is
    usually sensor noise, not something to drive around."""
    if len(points) == 0:
        return points
    keys = [tuple(k) for k in np.floor(points / link).astype(np.int64)]
    members = {}
    for i, k in enumerate(keys):
        members.setdefault(k, []).append(i)
    label, sizes = {}, []
    for c in members:
        if c in label:
            continue
        label[c] = len(sizes)
        stack, idx = [c], []
        while stack:
            r = stack.pop()
            idx.extend(members[r])
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    n = (r[0] + dx, r[1] + dy)
                    if n in members and n not in label:
                        label[n] = label[c]
                        stack.append(n)
        p = points[idx]
        sizes.append(max(np.ptp(p[:, 0]), np.ptp(p[:, 1])))
    return points[np.array([sizes[label[k]] >= min_size for k in keys], dtype=bool)]


def xy_to_cloud(points, frame_id, stamp):
    pts = np.zeros((len(points), 3), np.float32)
    if len(points):
        pts[:, :2] = points
    msg = PointCloud2()
    msg.header.frame_id, msg.header.stamp = frame_id, stamp
    msg.height, msg.width = 1, len(pts)
    msg.fields, msg.is_bigendian, msg.is_dense = _FIELDS, False, True
    msg.point_step, msg.row_step = 12, 12 * len(pts)
    msg.data = u8(pts.tobytes())
    return msg


def cloud_to_xy(msg):
    if msg.width * msg.height == 0:
        return np.zeros((0, 2))
    a = np.frombuffer(bytes(msg.data), np.float32).reshape(-1, msg.point_step // 4)
    return a[:, :2].astype(np.float64)
