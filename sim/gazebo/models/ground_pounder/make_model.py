#!/usr/bin/env python3
"""Generates model.sdf for the Redcat Ground Pounder (4-wheel steer) in Gazebo.

Edit the numbers below to match the real truck / camera rig, then re-run:
    python3 sim/gazebo/models/ground_pounder/make_model.py

Dimensions are approximate (1/10 scale crawler) - measure yours.

How ArduPilot drives it (ardupilot_gazebo ArduPilotPlugin, channels 0-based):
    ch0 = SERVO1 (GroundSteering)          -> front-left/right steering joints
    ch1 = SERVO2 (GroundSteering, REVERSED) -> rear-left/right steering joints
    ch2 = SERVO3 (Throttle)                 -> all four wheels (4WD)
Exactly like the real FC: the rear counter-steers because SERVO2_REVERSED=1
on the FC, not because of anything in this model.
"""
import os

# ----------------------------------------------------------------- dimensions
WHEELBASE = 0.32          # m, front axle to rear axle
TRACK = 0.25              # m, left wheel centre to right wheel centre
WHEEL_RADIUS = 0.062      # m
WHEEL_WIDTH = 0.055       # m
WHEEL_MASS = 0.15         # kg each
CHASSIS = (0.42, 0.20, 0.08)   # m, body box (x, y, z)
CHASSIS_MASS = 2.2        # kg
CHASSIS_Z = 0.03          # m, chassis bottom above axle height
MAX_STEER = 0.50          # rad (~29 deg) at full servo deflection
# TEST ONLY: simulate a steering linkage that hits its stop early when turning
# RIGHT (e.g. RCTRUCK_RIGHT_STEER_LIMIT=0.25 python3 make_model.py). Re-run
# without it to restore the normal truck.
RIGHT_STEER_LIMIT = float(os.environ.get('RCTRUCK_RIGHT_STEER_LIMIT', MAX_STEER))

# Max wheel speed at full throttle (PWM 2000). 133 rad/s * 0.062 m ~= 4.1 m/s.
# With CRUISE_THROTTLE=50 -> 2.07 m/s, i.e. CRUISE_SPEED 2 like the real FC.
WHEEL_MAX_RAD_S = 66.7
WHEEL_TORQUE_MAX = 0.35   # N*m per wheel (ESC/motor limit)

# ----------------------------------------------------------- stereo camera rig
CAM_BASELINE = 0.10       # m between left and right camera centres
CAM_X = 0.16              # m forward of chassis centre
CAM_Z = 0.09              # m above chassis centre (camera bar top ~0.24 m off the ground,
                          # below the lidar's scan plane so it doesn't block the beams)
CAM_PITCH = 0.10          # rad, tilted down a little (+ = down)
CAM_WIDTH, CAM_HEIGHT = 640, 480
CAM_HFOV = 1.40           # rad (~80 deg)
CAM_RATE = 15             # Hz
IMU_RATE = 1000           # Hz (SITL lock-step expects a fast IMU)

# -------------------------------------------------- RPLidar A1 (2D, 360 deg)
LIDAR_X = 0.12            # m forward of chassis centre ("near the front", behind the cameras)
LIDAR_GROUND_Z = 0.26     # m above the ground (260 mm mount)
LIDAR_RATE = 5.5          # Hz (A1 default scan rate)
LIDAR_SAMPLES = 360       # ~1 deg resolution
LIDAR_RANGE = (0.15, 12.0)
LIDAR_NOISE = 0.01        # m std (A1: <1% of range up to 3 m)

MODEL = 'ground_pounder'
HERE = os.path.dirname(os.path.abspath(__file__))


def box_inertia(m, x, y, z):
    return (m * (y * y + z * z) / 12.0, m * (x * x + z * z) / 12.0, m * (x * x + y * y) / 12.0)


def inertial(m, ixx, iyy, izz):
    return '''      <inertial>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.6g}</ixx><iyy>{iyy:.6g}</iyy><izz>{izz:.6g}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>'''.format(m=m, ixx=ixx, iyy=iyy, izz=izz)


def wheel_corner(name, x, y):
    """Steering knuckle (yaw joint) + wheel (spin joint) at one corner."""
    z = WHEEL_RADIUS
    r, w = WHEEL_RADIUS, WHEEL_WIDTH
    i_axis = WHEEL_MASS * r * r / 2.0
    i_other = WHEEL_MASS * (3 * r * r + w * w) / 12.0
    return '''
    <link name="{n}_knuckle">
      <pose>{x} {y} {z} 0 0 0</pose>
{knuckle_inertial}
    </link>
    <joint name="{n}_steer" type="revolute">
      <parent>base_link</parent>
      <child>{n}_knuckle</child>
      <axis>
        <xyz>0 0 1</xyz>
        <limit><lower>{lo}</lower><upper>{hi}</upper><effort>10</effort><velocity>8</velocity></limit>
        <dynamics><damping>0.05</damping></dynamics>
      </axis>
    </joint>
    <link name="{n}_wheel">
      <pose>{x} {y} {z} 0 0 0</pose>
{wheel_inertial}
      <collision name="collision">
        <pose>0 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{w}</length></cylinder></geometry>
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          <contact><ode><kp>1e5</kp><kd>50</kd><min_depth>0.001</min_depth></ode></contact>
        </surface>
      </collision>
      <visual name="tire">
        <pose>0 0 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{r}</radius><length>{w}</length></cylinder></geometry>
        <material><ambient>0.08 0.08 0.08 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <visual name="rim">
        <pose>0 {rim_y} 0 1.5708 0 0</pose>
        <geometry><cylinder><radius>{rim_r}</radius><length>0.004</length></cylinder></geometry>
        <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
      </visual>
    </link>
    <joint name="{n}_spin" type="revolute">
      <parent>{n}_knuckle</parent>
      <child>{n}_wheel</child>
      <axis>
        <xyz>0 1 0</xyz>
        <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        <dynamics><damping>0.002</damping></dynamics>
      </axis>
    </joint>'''.format(
        n=name, x=x, y=y, z=z, r=r, w=w,
        # right turn = front wheels negative yaw, rear wheels positive (counter-steer)
        lo=-(RIGHT_STEER_LIMIT if x > 0 else MAX_STEER),
        hi=(MAX_STEER if x > 0 else RIGHT_STEER_LIMIT),
        rim_y=(w / 2.0 + 0.001) * (1 if y > 0 else -1), rim_r=r * 0.6,
        knuckle_inertial=inertial(0.05, 1e-5, 1e-5, 1e-5),
        wheel_inertial=inertial(WHEEL_MASS, i_other, i_axis, i_other))


def lidar(chassis_z):
    z = LIDAR_GROUND_Z - chassis_z
    return '''
      <sensor name="lidar" type="gpu_lidar">
        <pose>{x} 0 {z} 0 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{rate}</update_rate>
        <topic>scan</topic>
        <gz_frame_id>laser</gz_frame_id>
        <lidar>
          <scan><horizontal><samples>{n}</samples><resolution>1</resolution>
            <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal></scan>
          <range><min>{rmin}</min><max>{rmax}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>{noise}</stddev></noise>
        </lidar>
      </sensor>
      <visual name="lidar_puck">
        <pose>{x} 0 {zp} 0 0 0</pose>
        <geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
        <material><ambient>0.05 0.05 0.05 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>'''.format(x=LIDAR_X, z=z, zp=z - 0.025, rate=LIDAR_RATE, n=LIDAR_SAMPLES,
                          rmin=LIDAR_RANGE[0], rmax=LIDAR_RANGE[1], noise=LIDAR_NOISE)


def camera(side, y):
    return '''
      <sensor name="{s}_camera" type="camera">
        <pose>{x} {y} {z} 0 {pitch} 0</pose>
        <always_on>1</always_on>
        <update_rate>{rate}</update_rate>
        <topic>stereo/{s}/image</topic>
        <camera name="{s}">
          <camera_info_topic>stereo/{s}/camera_info</camera_info_topic>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>30</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.004</stddev></noise>
        </camera>
      </sensor>'''.format(s=side, x=CAM_X, y=y, z=CAM_Z, pitch=CAM_PITCH, rate=CAM_RATE,
                          hfov=CAM_HFOV, w=CAM_WIDTH, h=CAM_HEIGHT)


def control(channel, joint, ctype, multiplier, p, d, cmd_max, comment):
    return '''
      <!-- {comment} -->
      <control channel="{ch}">
        <jointName>{joint}</jointName>
        <useForce>1</useForce>
        <type>{ctype}</type>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <offset>-0.5</offset>
        <multiplier>{mul:.4g}</multiplier>
        <rotorVelocitySlowdownSim>1</rotorVelocitySlowdownSim>
        <p_gain>{p}</p_gain><i_gain>0</i_gain><d_gain>{d}</d_gain>
        <i_max>0</i_max><i_min>0</i_min>
        <cmd_max>{cm}</cmd_max><cmd_min>{cmn}</cmd_min>
      </control>'''.format(comment=comment, ch=channel, joint=joint, ctype=ctype,
                           mul=multiplier, p=p, d=d, cm=cmd_max, cmn=-cmd_max)


def command_control(channel, joint, multiplier, topic, comment):
    return '''
      <!-- {comment} -->
      <control channel="{ch}">
        <jointName>{joint}</jointName>
        <type>COMMAND</type>
        <cmd_topic>{topic}</cmd_topic>
        <servo_min>1000</servo_min>
        <servo_max>2000</servo_max>
        <offset>-0.5</offset>
        <multiplier>{mul:.4g}</multiplier>
      </control>'''.format(comment=comment, ch=channel, joint=joint, topic=topic, mul=multiplier)


def position_controller(joint, topic):
    # ~ a hobby servo: 0.5 rad in ~0.1 s
    return '''
    <plugin filename="gz-sim-joint-position-controller-system"
            name="gz::sim::systems::JointPositionController">
      <joint_name>{joint}</joint_name>
      <topic>{topic}</topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>20</p_gain>
      <i_gain>0</i_gain>
      <d_gain>0</d_gain>
      <cmd_max>6</cmd_max>
      <cmd_min>-6</cmd_min>
    </plugin>'''.format(joint=joint, topic=topic)


def main():
    cx, cy, cz = CHASSIS
    chassis_z = WHEEL_RADIUS + CHASSIS_Z + cz / 2.0
    hx, hy = WHEELBASE / 2.0, TRACK / 2.0
    corners = [('front_left', hx, hy), ('front_right', hx, -hy),
               ('rear_left', -hx, hy), ('rear_right', -hx, -hy)]

    # PWM 1000..2000 -> raw 0..1 -> (raw - 0.5) * multiplier.
    # Steering: PWM > trim = steer right = negative yaw (z up) -> negative multiplier.
    steer_mul = -2.0 * MAX_STEER
    wheel_mul = 2.0 * WHEEL_MAX_RAD_S

    controls = []
    steer_ctrls = []
    # Steering: ArduPilotPlugin only RELAYS the target angle (COMMAND type) and
    # Gazebo's JointPositionController drives the knuckle with velocity
    # commands. (Force-PID on a 50 g knuckle chattered and the wheels never
    # actually turned.)
    for ch, names, what in ((0, ('front_left', 'front_right'), 'SERVO1 front steering'),
                            (1, ('rear_left', 'rear_right'), 'SERVO2 rear steering (reversed on the FC)')):
        for n in names:
            topic = '/model/{}/{}_steer/cmd'.format(MODEL, n)
            controls.append(command_control(ch, n + '_steer', steer_mul, topic,
                                            '{} -> {}'.format(what, n)))
            steer_ctrls.append(position_controller(n + '_steer', topic))
    for n, _, _ in corners:
        controls.append(control(2, n + '_spin', 'VELOCITY', wheel_mul, 0.05, 0.0,
                                WHEEL_TORQUE_MAX, 'SERVO3 throttle -> ' + n + ' wheel'))

    ixx, iyy, izz = box_inertia(CHASSIS_MASS, cx, cy, cz)
    sdf = '''<?xml version="1.0"?>
<!-- GENERATED by make_model.py - edit that file, not this one. -->
<sdf version="1.9">
  <model name="{model}">
    <pose>0 0 0 0 0 0</pose>
    <link name="base_link">
      <pose>0 0 {chz} 0 0 0</pose>
{chassis_inertial}
      <collision name="collision">
        <geometry><box><size>{cx} {cy} {cz}</size></box></geometry>
      </collision>
      <visual name="body">
        <pose>0 0 0.03 0 0 0</pose>
        <geometry><box><size>{bx} {by} 0.07</size></box></geometry>
        <material><ambient>0.6 0.05 0.05 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
      </visual>
      <visual name="frame">
        <geometry><box><size>{cx} 0.08 {cz}</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
      </visual>
      <visual name="camera_bar">
        <pose>{camx} 0 {camz} 0 0 0</pose>
        <geometry><box><size>0.03 {barw} 0.03</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
{cameras}
{lidar}
    </link>

    <link name="imu_link">
      <pose>0 0 {chz} 0 0 0</pose>
{imu_inertial}
      <!-- Rolled 180 deg so the sensor measures in ArduPilot's FRD body frame
           (ArduPilotPlugin forwards IMU data as-is; same as ardupilot_gazebo's iris). -->
      <sensor name="imu_sensor" type="imu">
        <pose degrees="true">0 0 0 180 0 0</pose>
        <always_on>1</always_on>
        <update_rate>{imu_rate}</update_rate>
      </sensor>
    </link>
    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
    </joint>
{corners}

    <plugin name="ArduPilotPlugin" filename="ArduPilotPlugin">
      <fdm_addr>127.0.0.1</fdm_addr>
      <fdm_port_in>9002</fdm_port_in>
      <connectionTimeoutMaxCount>5</connectionTimeoutMaxCount>
      <lock_step>1</lock_step>
      <!-- model: x forward, y left, z up -> ArduPilot body FRD -->
      <modelXYZToAirplaneXForwardZDown degrees="true">0 0 0 180 0 0</modelXYZToAirplaneXForwardZDown>
      <gazeboXYZToNED degrees="true">0 0 0 180 0 90</gazeboXYZToNED>
      <imuName>imu_link::imu_sensor</imuName>
{controls}
    </plugin>
{steer_plugins}

    <!-- Ground truth pose, ONLY for grading SLAM in sim (the truck has no such thing). -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>{model}</robot_base_frame>
      <odom_topic>/model/{model}/odometry</odom_topic>
      <tf_topic>/model/{model}/odometry_tf</tf_topic>
      <odom_publish_frequency>20</odom_publish_frequency>
      <dimensions>3</dimensions>
    </plugin>
  </model>
</sdf>
'''.format(
        model=MODEL, chz=chassis_z, cx=cx, cy=cy, cz=cz, bx=cx * 1.05, by=TRACK + WHEEL_WIDTH,
        camx=CAM_X, camz=CAM_Z, barw=CAM_BASELINE + 0.06,
        chassis_inertial=inertial(CHASSIS_MASS, ixx, iyy, izz),
        imu_inertial=inertial(0.01, 1e-5, 1e-5, 1e-5), imu_rate=IMU_RATE,
        cameras=camera('left', CAM_BASELINE / 2.0) + camera('right', -CAM_BASELINE / 2.0),
        lidar=lidar(chassis_z),
        corners=''.join(wheel_corner(n, x, y) for n, x, y in corners),
        controls=''.join(controls), steer_plugins=''.join(steer_ctrls))

    with open(os.path.join(HERE, 'model.sdf'), 'w') as f:
        f.write(sdf)
    with open(os.path.join(HERE, 'model.config'), 'w') as f:
        f.write('''<?xml version="1.0"?>
<model>
  <name>{0}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Redcat Ground Pounder, front + rear steer, stereo camera pair,
    driven by ArduPilot SITL (Rover) through ArduPilotPlugin.</description>
</model>
'''.format(MODEL))
    # Camera geometry for the ROS side (TF + stereo baseline), so the launch
    # files never drift from the model.
    with open(os.path.join(HERE, 'rig.yaml'), 'w') as f:
        f.write('# GENERATED by make_model.py\n'
                'base_link_z: {:.4f}\ncam_x: {}\ncam_z: {}\ncam_pitch: {}\n'
                'baseline: {}\nwidth: {}\nheight: {}\nhfov: {}\n'
                'lidar_x: {}\nlidar_z: {:.4f}\n'.format(
                    chassis_z, CAM_X, CAM_Z, CAM_PITCH, CAM_BASELINE,
                    CAM_WIDTH, CAM_HEIGHT, CAM_HFOV, LIDAR_X, LIDAR_GROUND_Z - chassis_z))
    print('wrote model.sdf, model.config, rig.yaml in ' + HERE)


if __name__ == '__main__':
    main()
