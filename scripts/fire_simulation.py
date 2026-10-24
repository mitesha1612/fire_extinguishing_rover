#!/usr/bin/env python3

import math
import json
import subprocess
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool


WORLD = 'default'
FIRE_NAME = 'fire_target'
WATER_NAME = 'fire_water_stream'
FIRE_X = 2.5
FIRE_Y = 0.0
MAX_WATER_RANGE = 1.8
FIRE_HIT_RADIUS = 0.24
WATER_HIT_RADIUS = 0.025
EXTINGUISH_SECONDS = 2.0


FIRE_SDF = (
    "<sdf version='1.8'><model name='fire_target'><static>true</static>"
    "<link name='fire_visuals'>"
    "<visual name='flame_outer'><pose>0 0 0.34 0 0 0</pose><geometry>"
    "<sphere><radius>0.24</radius></sphere></geometry><material>"
    "<ambient>1 0.16 0.01 1</ambient><diffuse>1 0.25 0.01 1</diffuse>"
    "<emissive>1 0.12 0.005 1</emissive></material></visual>"
    "<visual name='flame_inner'><pose>0 0 0.28 0 0 0</pose><geometry>"
    "<sphere><radius>0.15</radius></sphere></geometry><material>"
    "<ambient>1 0.75 0.02 1</ambient><diffuse>1 0.55 0.01 1</diffuse>"
    "<emissive>1 0.45 0.01 1</emissive></material></visual>"
    "<visual name='smoke'><pose>0 0 0.72 0 0 0</pose><geometry>"
    "<sphere><radius>0.17</radius></sphere></geometry><material>"
    "<ambient>0.22 0.22 0.22 0.65</ambient><diffuse>0.22 0.22 0.22 0.65</diffuse>"
    "</material></visual></link></model></sdf>"
)


def call_gazebo_service(service, request_type, response_type, request):
    result = subprocess.run(
        [
            'gz', 'service', '-s', service,
            '--reqtype', request_type,
            '--reptype', response_type,
            '--timeout', '2000',
            '--req', request,
        ],
        capture_output=True,
        text=True,
        check=False,
        timeout=3,
    )
    output = f'{result.stdout}\n{result.stderr}'.strip()
    if result.returncode != 0 or 'data: true' not in output:
        raise RuntimeError(f'Gazebo service {service} failed: {output}')


def spawn_model(name, sdf, x, y, z, orientation=''):
    if not orientation:
        orientation = 'w: 1'
    request = (
        f'name: "{name}" sdf: {json.dumps(sdf)} '
        f'pose {{ position {{ x: {x} y: {y} z: {z} }} '
        f'orientation {{ {orientation} }} }}'
    )
    call_gazebo_service(
        f'/world/{WORLD}/create',
        'gz.msgs.EntityFactory',
        'gz.msgs.Boolean',
        request,
    )


def remove_model(name):
    request = f'name: "{name}" type: MODEL'
    call_gazebo_service(
        f'/world/{WORLD}/remove',
        'gz.msgs.Entity',
        'gz.msgs.Boolean',
        request,
    )


class FireSimulation(Node):
    def __init__(self):
        super().__init__('fire_simulation')
        self.rover_pose = None
        self.spraying = False
        self.fire_burning = True
        self.spray_started = None
        self.water_visible = False
        self.create_subscription(
            Odometry, '/diff_drive_controller/odom', self.on_odometry, 10
        )
        self.create_subscription(Bool, '/fire/water_spray', self.on_spray, 10)
        self.create_timer(0.1, self.update_extinguishing)

        last_error = None
        for _ in range(10):
            try:
                spawn_model(FIRE_NAME, FIRE_SDF, FIRE_X, FIRE_Y, 0.0)
                self.get_logger().info(
                    f'Fire target spawned at ({FIRE_X:.1f}, {FIRE_Y:.1f}). '
                    'Drive within range and hold Space while facing it.'
                )
                break
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                last_error = error
                time.sleep(1)
        else:
            raise RuntimeError(f'Could not spawn the fire target: {last_error}')

    def on_odometry(self, message):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.rover_pose = (pose.position.x, pose.position.y, yaw)
        if self.spraying and self.fire_burning and not self.water_visible:
            try:
                self.spawn_water_stream()
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                self.get_logger().error(f'Could not show water spray: {error}')

    def on_spray(self, message):
        if self.spraying == message.data:
            return
        self.spraying = message.data
        self.spray_started = time.monotonic() if self.spraying else None
        self.get_logger().info('Water spray on.' if self.spraying else 'Water spray off.')
        try:
            if self.spraying and not self.water_visible:
                self.spawn_water_stream()
            elif not self.spraying and self.water_visible:
                remove_model(WATER_NAME)
                self.water_visible = False
        except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f'Could not update water spray visual: {error}')

    def spawn_water_stream(self):
        if self.rover_pose is None:
            return
        x, y, yaw = self.rover_pose
        length = MAX_WATER_RANGE
        sdf = (
            "<sdf version='1.8'><model name='fire_water_stream'><static>true</static>"
            "<link name='stream'><visual name='water'><geometry><cylinder>"
            f"<radius>0.025</radius><length>{length:.3f}</length>"
            "</cylinder></geometry><material><ambient>0.05 0.45 1 0.8</ambient>"
            "<diffuse>0.05 0.45 1 0.8</diffuse><emissive>0.01 0.12 0.4 1</emissive>"
            "</material></visual></link></model></sdf>"
        )
        half_sine = math.sqrt(0.5)
        qx = -math.sin(yaw) * half_sine
        qy = math.cos(yaw) * half_sine
        orientation = f'x: {qx:.6f} y: {qy:.6f} z: 0 w: {half_sine:.6f}'
        stream_x = x + math.cos(yaw) * length / 2
        stream_y = y + math.sin(yaw) * length / 2
        spawn_model(WATER_NAME, sdf, stream_x, stream_y, 0.4, orientation)
        self.water_visible = True

    def update_extinguishing(self):
        if not self.spraying or self.rover_pose is None or not self.fire_burning:
            return

        x, y, yaw = self.rover_pose
        dx, dy = FIRE_X - x, FIRE_Y - y
        along_stream = dx * math.cos(yaw) + dy * math.sin(yaw)
        across_stream = abs(-dx * math.sin(yaw) + dy * math.cos(yaw))
        stream_hits_fire = (
            0.0 <= along_stream <= MAX_WATER_RANGE + FIRE_HIT_RADIUS
            and across_stream <= FIRE_HIT_RADIUS + WATER_HIT_RADIUS
        )
        if not stream_hits_fire:
            self.spray_started = None
            return

        if self.spray_started is None:
            self.spray_started = time.monotonic()
            return

        if self.spray_started is not None and (
            time.monotonic() - self.spray_started >= EXTINGUISH_SECONDS
        ):
            try:
                remove_model(FIRE_NAME)
            except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
                self.get_logger().error(f'Could not extinguish fire target: {error}')
                self.spray_started = time.monotonic()
                return
            self.fire_burning = False
            self.get_logger().info('Fire extinguished!')


def main():
    rclpy.init()
    node = None
    try:
        node = FireSimulation()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        if node is not None:
            node.get_logger().fatal(str(error))
        else:
            print(f'Fire simulation failed: {error}')
        raise SystemExit(1) from error
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
