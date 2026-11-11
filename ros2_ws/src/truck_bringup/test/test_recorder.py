import os

from truck_bringup.recorder_node import PRESETS, bag_name, dir_size_mb


def test_presets_are_absolute_topics_and_light_has_no_images():
    for topics in PRESETS.values():
        assert all(t.startswith('/') for t in topics)
    assert not any('image' in t for t in PRESETS['light'])
    assert '/stereo/left/image_raw' in PRESETS['slam'] and '/fc/imu' in PRESETS['slam']


def test_bag_name_and_size(tmp_path):
    assert bag_name('slam', 0).endswith('_slam')
    d = tmp_path / 'bag'
    d.mkdir()
    (d / 'a.db3').write_bytes(b'x' * 2000000)
    assert abs(dir_size_mb(str(d)) - 2.0) < 0.01
    assert dir_size_mb(os.path.join(str(tmp_path), 'missing')) == 0
