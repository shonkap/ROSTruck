"""Turns Gazebo's two independent camera sensors into a proper ROS stereo pair.

Gazebo publishes each camera on its own, with a Gazebo frame name and a
CameraInfo whose projection matrix has no baseline (Tx = 0). stereo_image_proc
and RTAB-Map read the baseline from the *right* CameraInfo (P[3] = -fx * B),
so without this they'd see "two cameras at the same spot" and fail.

In:   gz/stereo/{left,right}/image, gz/stereo/{left,right}/camera_info
Out:  stereo/{left,right}/image_raw, stereo/{left,right}/camera_info
      (frames {left,right}_camera_optical_frame, identical stamps per pair)

Same topic names as the real truck's stereo_camera_node, so everything
downstream (SLAM launch, calibration checks) is shared between sim and truck.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class StereoRelay(Node):
    def __init__(self):
        super().__init__('stereo_relay')
        self.declare_parameter('baseline', 0.10)
        self._baseline = float(self.get_parameter('baseline').value)
        self._info = {'left': None, 'right': None}
        self._latest = {'left': None, 'right': None}
        self._img_pub = {}
        self._info_pub = {}
        for side in ('left', 'right'):
            self._img_pub[side] = self.create_publisher(
                Image, 'stereo/{}/image_raw'.format(side), qos_profile_sensor_data)
            self._info_pub[side] = self.create_publisher(
                CameraInfo, 'stereo/{}/camera_info'.format(side), qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, 'gz/stereo/{}/camera_info'.format(side),
                lambda m, s=side: self._on_info(s, m), qos_profile_sensor_data)
            self.create_subscription(
                Image, 'gz/stereo/{}/image'.format(side),
                lambda m, s=side: self._on_image(s, m), qos_profile_sensor_data)
        self._pairs = 0
        self.create_timer(10.0, self._report)

    def _on_info(self, side, msg):
        frame = '{}_camera_optical_frame'.format(side)
        msg.header.frame_id = frame
        p = list(msg.p)
        # Left camera is the stereo reference; right sits baseline to its +x
        # (optical frame x = image right) -> Tx = -fx * B.
        p[3] = -p[0] * self._baseline if side == 'right' else 0.0
        msg.p = p
        # Gazebo leaves R as zeros on some versions; rectified pinhole -> identity.
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        if not msg.distortion_model:
            msg.distortion_model = 'plumb_bob'
            msg.d = [0.0] * 5
        self._info[side] = msg

    def _on_image(self, side, msg):
        if self._info[side] is None:
            return
        self._latest[side] = msg
        left, right = self._latest['left'], self._latest['right']
        if left is None or right is None:
            return
        # Both sensors render on the same sim step, so stamps should match.
        # Wait for the partner frame (either one can arrive first) and never
        # feed SLAM a mismatched pair.
        dt = abs((left.header.stamp.sec - right.header.stamp.sec) +
                 (left.header.stamp.nanosec - right.header.stamp.nanosec) * 1e-9)
        if dt > 0.005:
            return
        self._latest = {'left': None, 'right': None}
        stamp = left.header.stamp
        for s, img in (('left', left), ('right', right)):
            img.header.stamp = stamp
            img.header.frame_id = '{}_camera_optical_frame'.format(s)
            info = self._info[s]
            info.header.stamp = stamp
            self._img_pub[s].publish(img)
            self._info_pub[s].publish(info)
        self._pairs += 1

    def _report(self):
        self.get_logger().info('{} stereo pairs in the last 10 s ({:.1f} Hz)'.format(
            self._pairs, self._pairs / 10.0))
        self._pairs = 0


def main(args=None):
    rclpy.init(args=args)
    node = StereoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
