#!/usr/bin/env python3
"""Step-by-step stereo camera calibration for the truck. Python 3.6 compatible.

Run inside a ROS2 environment that can see the cameras (the Jetson container,
or any machine with ROS2 Humble + the cameras plugged in):

    python3 scripts/stereo_calibrate.py board          # 1. make a printable checkerboard
    python3 scripts/stereo_calibrate.py cameras        # 2. find your camera devices
    python3 scripts/stereo_calibrate.py run            # 3. the guided calibration
    python3 scripts/stereo_calibrate.py check          # 4. re-check an existing calibration

`run` options (all have prompts if left out):
    --mode dual|side_by_side   two cameras, or one USB device with a side-by-side image
    --left 0 --right 1         V4L2 index / /dev/videoN / csi:0 / csi:1
    --width 640 --height 480 --fps 15
    --board 9x6 --square 0.025 inner corners, square size in metres (MEASURE IT)
    --baseline 0.10            measured distance between the two lens centres, m
    --remote                   cameras on the Jetson, calibrator GUI on this PC:
                               don't start the camera node here, use the
                               compressed topics from the Jetson

The result is written to ros2_ws/src/truck_bringup/config/camera_info/{left,right}.yaml,
which truck.launch.py loads by default.
"""
import argparse
import glob
import os
import shutil
import signal
import subprocess
import sys
import tarfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CALIB_DIR = os.path.join(REPO, 'ros2_ws', 'src', 'truck_bringup', 'config', 'camera_info')
LEFT_YAML = os.path.join(CALIB_DIR, 'left.yaml')
RIGHT_YAML = os.path.join(CALIB_DIR, 'right.yaml')


def say(text=''):
    print(text, flush=True)


def header(n, title):
    say('\n' + '=' * 72)
    say(' STEP {}: {}'.format(n, title))
    say('=' * 72)


def ask(prompt, default=None):
    suffix = ' [{}]'.format(default) if default is not None else ''
    try:
        val = input('{}{}: '.format(prompt, suffix)).strip()
    except EOFError:
        val = ''
    return val or (str(default) if default is not None else '')


def pause(text='Press Enter to continue (Ctrl+C to quit)'):
    ask(text)


def run(cmd, **kw):
    say('$ ' + ' '.join(cmd))
    return subprocess.call(cmd, **kw)


def have(cmd):
    return shutil.which(cmd) is not None


# ---------------------------------------------------------------- 1. board

def cmd_board(args):
    import cv2
    import numpy as np
    cols, rows = (int(v) for v in args.board.lower().split('x'))   # inner corners
    sq_mm = args.square * 1000.0
    dpi = 300
    px = int(round(sq_mm / 25.4 * dpi))
    w, h = (cols + 1) * px, (rows + 1) * px
    margin = int(15 / 25.4 * dpi)
    img = np.full((h + 2 * margin, w + 2 * margin), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y, x = margin + r * px, margin + c * px
                img[y:y + px, x:x + px] = 0
    label = '{}x{} inner corners, {:.1f} mm squares - print at 100% / "actual size"'.format(
        cols, rows, sq_mm)
    cv2.putText(img, label, (margin, h + margin + int(margin * 0.7)), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, 0, 2, cv2.LINE_AA)
    out = args.out or os.path.join(REPO, 'checkerboard_{}x{}_{:.0f}mm.png'.format(cols, rows, sq_mm))
    cv2.imwrite(out, img)
    say('Wrote {} ({:.0f} x {:.0f} mm at {} dpi)'.format(
        out, img.shape[1] / dpi * 25.4, img.shape[0] / dpi * 25.4, dpi))
    say('''
  * Print at 100% scale ("actual size", NOT "fit to page") - it fits on A4/Letter
    landscape with the defaults.
  * Glue/tape it perfectly FLAT to something rigid (foam board, clipboard, a box
    lid). A wavy board is the #1 cause of bad calibrations.
  * MEASURE one square with a ruler/calipers (across 5 squares, divide by 5) and
    use that number for --square: printers scale slightly.
  * Matte paper is better than glossy (reflections hide corners).''')
    return 0


# -------------------------------------------------------------- 2. cameras

def cmd_cameras(args):
    say('V4L2 devices (USB cameras):')
    if have('v4l2-ctl'):
        run(['v4l2-ctl', '--list-devices'])
        for dev in sorted(glob.glob('/dev/video*')):
            say('\n--- {} formats:'.format(dev))
            subprocess.call(['v4l2-ctl', '-d', dev, '--list-formats-ext'],
                            stdout=sys.stdout, stderr=subprocess.DEVNULL)
    else:
        say('  (install v4l-utils for details: sudo apt install v4l-utils)')
        for dev in sorted(glob.glob('/dev/video*')):
            say('  ' + dev)
    if not glob.glob('/dev/video*'):
        say('  none found')
    say('''
Notes:
  * Each USB webcam usually creates TWO /dev/video nodes; the first (even
    number) is the image stream. So two webcams are typically 0 and 2.
  * Jetson CSI cameras (IMX219 etc.) also show up as /dev/video0/1 but must be
    read with --left csi:0 --right csi:1 (nvarguscamerasrc), not V4L2.
  * A single "stereo USB camera" that shows ONE device with a double-width
    format (e.g. 2560x720) -> --mode side_by_side --left 0.''')
    if have('gst-inspect-1.0'):
        ok = subprocess.call(['gst-inspect-1.0', 'nvarguscamerasrc'], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL) == 0
        say('  nvarguscamerasrc (CSI) available: {}'.format('yes' if ok else 'no'))
    return 0


# ----------------------------------------------------------------- helpers

def start_camera_node(args, calibrated):
    cmd = ['ros2', 'run', 'truck_bringup', 'stereo_camera_node', '--ros-args',
           '-p', 'mode:=' + args.mode, '-p', 'left_source:=' + args.left,
           '-p', 'right_source:=' + args.right,
           '-p', 'width:={}'.format(args.width), '-p', 'height:={}'.format(args.height),
           '-p', 'fps:={}'.format(args.fps),
           '-p', 'left_calibration:=' + (LEFT_YAML if calibrated else LEFT_YAML),
           '-p', 'right_calibration:=' + (RIGHT_YAML if calibrated else RIGHT_YAML)]
    say('$ ' + ' '.join(cmd))
    return subprocess.Popen(cmd, preexec_fn=os.setsid)


def stop(proc):
    if proc is not None and proc.poll() is None:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)


def topic_rates_and_brightness(seconds):
    """Subscribe to both cameras for `seconds`: (left_hz, right_hz, left_mean, right_mean)."""
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('calib_probe')
    stats = {'left': [0, []], 'right': [0, []]}

    def cb(side, msg):
        stats[side][0] += 1
        if stats[side][0] % 3 == 0:
            stats[side][1].append(float(np.frombuffer(msg.data, np.uint8)[::97].mean()))
    for s in ('left', 'right'):
        node.create_subscription(Image, 'stereo/{}/image_raw'.format(s),
                                 lambda m, s=s: cb(s, m), qos_profile_sensor_data)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()
    mean = lambda v: sum(v) / len(v) if v else 0.0  # noqa: E731
    return (stats['left'][0] / seconds, stats['right'][0] / seconds,
            mean(stats['left'][1]), mean(stats['right'][1]))


def recover_from_tarball():
    """If COMMIT wasn't pressed, SAVE leaves the result in /tmp/calibrationdata.tar.gz."""
    tb = '/tmp/calibrationdata.tar.gz'
    if not os.path.exists(tb):
        return False
    with tarfile.open(tb) as t:
        names = t.getnames()
        if 'left.yaml' not in names or 'right.yaml' not in names:
            return False
        for side, dst in (('left', LEFT_YAML), ('right', RIGHT_YAML)):
            with t.extractfile(side + '.yaml') as src, open(dst, 'wb') as out:
                out.write(src.read())
    say('Recovered left.yaml/right.yaml from ' + tb)
    return True


# --------------------------------------------------------------------- run

def cmd_run(args):
    header(0, 'pre-flight')
    if not have('ros2'):
        say('ros2 not found: source ROS first (/opt/ros/humble/setup.bash or '
            '/opt/ros/humble/install/setup.bash on the Jetson) and the workspace.')
        return 1
    ok = subprocess.call(['ros2', 'pkg', 'prefix', 'camera_calibration'],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
    if not ok:
        say('camera_calibration is missing: sudo apt install ros-humble-camera-calibration '
            '(it is part of image_pipeline).')
        return 1
    if not os.environ.get('DISPLAY'):
        say('No DISPLAY: the calibrator needs a screen. Either run this on a machine with a '
            'monitor, or run the cameras on the Jetson and this with --remote on your PC.')
        return 1
    os.makedirs(CALIB_DIR, exist_ok=True)
    say('Calibration files will be written to ' + CALIB_DIR)

    header(1, 'your rig')
    args.mode = args.mode or ask('Mode: dual (two cameras) or side_by_side (one stereo device)', 'dual')
    if not args.remote:
        args.left = args.left or ask('LEFT camera source (as seen from BEHIND the truck)', '0')
        if args.mode == 'dual':
            args.right = args.right or ask('RIGHT camera source', '2')
    args.board = args.board or ask('Checkerboard INNER corners (cols x rows)', '9x6')
    args.square = args.square or float(ask('Measured square size in metres', '0.025'))
    args.baseline = args.baseline or float(ask('Measured lens-centre distance (baseline) in m', '0.10'))
    say('''
Mounting checklist (before calibrating - any change later invalidates it):
  * cameras bolted rigidly in their FINAL position on the truck, same height,
    looking the same direction (parallel, not toed in)
  * focus locked (twist-to-focus lenses: set focus at ~1.5 m, then glue/tape)
  * same resolution/fps you will use for SLAM ({}x{} @ {} Hz)'''.format(
        args.width, args.height, args.fps))
    pause()

    cam = None
    try:
        if not args.remote:
            header(2, 'start the cameras')
            for f in (LEFT_YAML, RIGHT_YAML):   # calibrate from scratch
                if os.path.exists(f):
                    shutil.move(f, f + '.bak')
                    say('old {} -> .bak'.format(os.path.basename(f)))
            cam = start_camera_node(args, calibrated=False)
            time.sleep(4.0)
            if cam.poll() is not None:
                say('Camera node exited - check the sources with: {} cameras'.format(sys.argv[0]))
                return 1
        else:
            header(2, 'remote cameras')
            say('On the Jetson:  ros2 launch truck_bringup cameras.launch.py publish_compressed:=true')
            say('Here we decompress them locally (WiFi can\'t carry two raw streams):')
            for s in ('left', 'right'):
                subprocess.Popen(['ros2', 'run', 'image_transport', 'republish', 'compressed', 'raw',
                                  '--ros-args', '-r', 'in/compressed:=/stereo/{}/image_raw/compressed'.format(s),
                                  '-r', 'out:=/stereo/{}/image_raw'.format(s)], preexec_fn=os.setsid)
            pause('Start the Jetson side, then press Enter')

        lh, rh, _, _ = topic_rates_and_brightness(3.0)
        say('  left  {:.1f} Hz,  right {:.1f} Hz'.format(lh, rh))
        if lh < 2 or rh < 2:
            say('Not receiving both cameras. Check `ros2 topic list` and the camera node log.')
            return 1

        header(3, 'left/right check')
        say('COVER THE LEFT LENS with your hand (left = left side when standing BEHIND the truck).')
        pause('Covering it? Press Enter, keep it covered for 3 s')
        _, _, lb, rb = topic_rates_and_brightness(3.0)
        say('  brightness: left {:.0f}, right {:.0f}'.format(lb, rb))
        if lb > rb:
            say('!! The RIGHT image went dark: your cameras are SWAPPED.')
            say('   Re-run with --left/--right exchanged (or swap:=true in cameras.launch.py).')
            return 1
        say('  OK, left is left.')

        header(4, 'calibrate')
        say('''A window opens with both images. Hold the board ~0.5-2 m from the cameras,
fully visible in BOTH images, and move SLOWLY (motion blur ruins samples):
  * left/right and up/down across the whole image, including the corners -> X, Y bars
  * near and far                                                       -> Size bar
  * tilted ~30-45 deg left/right and up/down                           -> Skew bar
  * pause briefly at each pose so both cameras capture the same instant
When all bars are green and CALIBRATE lights up (aim for 40+ samples), click
CALIBRATE (takes up to a minute). Check the console: "epi" error should be
< 0.5 px (good) - redo if it's > 1. Then click SAVE, then COMMIT, then close
the window.''')
        pause('Press Enter to open the calibrator')
        cols_rows = args.board
        run(['ros2', 'run', 'camera_calibration', 'cameracalibrator',
             '--size', cols_rows, '--square', str(args.square),
             '--no-service-check', '--ros-args',
             '-r', 'left:=/stereo/left/image_raw', '-r', 'right:=/stereo/right/image_raw',
             '-r', 'left_camera:=/stereo/left', '-r', 'right_camera:=/stereo/right'])

        header(5, 'save')
        if not (os.path.exists(LEFT_YAML) and os.path.exists(RIGHT_YAML)):
            if not recover_from_tarball():
                say('No calibration found (did you press CALIBRATE + SAVE/COMMIT?). Nothing written.')
                return 1
        say('  ' + LEFT_YAML)
        say('  ' + RIGHT_YAML)
        if args.remote:
            say('Remote mode: copy these two files to the same path on the Jetson '
                '(or commit them to git and pull there).')
            return 0

        header(6, 'verify')
        stop(cam)
        cam = start_camera_node(args, calibrated=True)
        time.sleep(4.0)
        say('Point the truck at a textured scene (and/or hold the board in view)...')
        run(['ros2', 'run', 'truck_bringup', 'stereo_check', '--ros-args',
             '-p', 'board:=' + args.board, '-p', 'square:={}'.format(args.square),
             '-p', 'expected_baseline:={}'.format(args.baseline),
             '-p', 'preview:=' + os.path.join(CALIB_DIR, 'check_preview.png')])
        say('''
Next:
  * open {}/check_preview.png - the green lines must cross the same features
    in both rectified images
  * baseline should be within ~2% of what you measured
  * run SLAM:  ros2 launch truck_bringup truck.launch.py slam:=true'''.format(CALIB_DIR))
        return 0
    finally:
        stop(cam)


def cmd_check(args):
    if not (os.path.exists(LEFT_YAML) and os.path.exists(RIGHT_YAML)):
        say('No calibration in {} yet - run `{} run` first.'.format(CALIB_DIR, sys.argv[0]))
        return 1
    say('Make sure the cameras are running (truck.launch.py or cameras.launch.py).')
    return run(['ros2', 'run', 'truck_bringup', 'stereo_check', '--ros-args',
                '-p', 'board:=' + (args.board or ''), '-p', 'square:={}'.format(args.square or 0.0),
                '-p', 'expected_baseline:={}'.format(args.baseline or 0.0),
                '-p', 'preview:=' + os.path.join(CALIB_DIR, 'check_preview.png')])


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('step', choices=['board', 'cameras', 'run', 'check'])
    ap.add_argument('--mode', choices=['dual', 'side_by_side'])
    ap.add_argument('--left')
    ap.add_argument('--right', default=None)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=480)
    ap.add_argument('--fps', type=int, default=15)
    ap.add_argument('--board', default=None, help='inner corners, e.g. 9x6')
    ap.add_argument('--square', type=float, default=None, help='square size in m')
    ap.add_argument('--baseline', type=float, default=None, help='measured baseline in m')
    ap.add_argument('--remote', action='store_true')
    ap.add_argument('--out', help='board: output PNG path')
    args = ap.parse_args()
    if args.step == 'board':
        args.board = args.board or '9x6'
        args.square = args.square or 0.025
        return cmd_board(args)
    if args.right is None:
        args.right = '1' if args.mode != 'side_by_side' else ''
    try:
        return {'cameras': cmd_cameras, 'run': cmd_run, 'check': cmd_check}[args.step](args)
    except KeyboardInterrupt:
        say('\naborted')
        return 130


if __name__ == '__main__':
    sys.exit(main())
