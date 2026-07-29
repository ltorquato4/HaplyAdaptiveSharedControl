#!/usr/bin/env python3

import asyncio
import json
import sys
import threading
import time
import traceback
import urllib.request

import orjson
import rclpy
import websockets
from geometry_msgs.msg import Point, Quaternion, Vector3
from haply_msgs.msg import (
    HandleButtons,
    HandleState,
    HaplyControl,
    HaplyState,
    Inverse3State,
)
from rclpy.node import Node


class HaplyDriverNode(Node):
    def __init__(self):
        super().__init__("haply_driver_node")

        # Parameters
        self.declare_parameter("frequency", 200.0)
        self.declare_parameter("max_force", 10.0)   # Maximum allowed force in N
        self.frequency = self.get_parameter("frequency").value
        self.max_force = float(self.get_parameter("max_force").value)

        # PID controller variables
        self.proportional_gain = 30.0
        self.integral_gain = 5.0
        self.derivative_gain = 0.9
        self.integral_error = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_update_time = time.perf_counter()

        # State variables
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.velocity = Vector3(x=0.0, y=0.0, z=0.0)
        self.quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.buttons = HandleButtons(a=False, b=False, c=False)

        # Control variables
        self.control_active = False
        self.use_target_position = False
        self.target_force = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.target_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_message_time = time.perf_counter()
        self.timeout_seconds = 50

        self.start_time = time.time()
        self.last_device_seen_time = time.perf_counter()

        # Device availability flags
        self.inverse_available = False
        self.handle_available = False
        self.inverse_warned = False
        self.handle_warned = False

        # Subscriber
        self.create_subscription(HaplyControl, 'haply_target', self.control_message_received, 10)

        # Publishers
        self.inverse3_state_publisher = self.create_publisher(Inverse3State, "inverse3_state", 10)
        self.handle_state_publisher = self.create_publisher(HandleState, "handle_state", 10)
        self.haply_state_publisher = self.create_publisher(HaplyState, "haply_state", 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_state)

        self.inverse3_device_identifier = None
        self.websocket_uniform_resource_identifier = 'ws://localhost:10001'

        # WebSocket loop in a separate thread
        self.run_execution = True
        self.websocket_thread = threading.Thread(target=self.start_asynchronous_loop, daemon=True)
        self.websocket_thread.start()

    def control_message_received(self, message: HaplyControl):
        self.last_message_time = time.perf_counter()
        self.control_active = True
        self.use_target_position = bool(message.use_position)

        if self.use_target_position:
            self.target_position = {
                "x": float(message.target_position.x),
                "y": float(message.target_position.y),
                "z": float(message.target_position.z),
            }
            self.target_force = self.calculate_force()
        else:
            self.target_force = {
                "x": float(message.force.x),
                "y": float(message.force.y),
                "z": float(message.force.z),
            }

    def calculate_force(self):
        current_time = time.perf_counter()
        time_delta = current_time - self.last_update_time
        if time_delta <= 0.0:
            time_delta = 1e-6
        self.last_update_time = current_time

        error_x = float(self.target_position["x"]) - float(self.position.x)
        error_y = float(self.target_position["y"]) - float(self.position.y)
        error_z = float(self.target_position["z"]) - float(self.position.z)

        self.integral_error["x"] += error_x * time_delta
        self.integral_error["y"] += error_y * time_delta
        self.integral_error["z"] += error_z * time_delta

        velocity_x = float(self.velocity.x)
        velocity_y = float(self.velocity.y)
        velocity_z = float(self.velocity.z)

        force_x = self.proportional_gain * error_x + self.integral_gain * self.integral_error["x"] - self.derivative_gain * velocity_x
        force_y = self.proportional_gain * error_y + self.integral_gain * self.integral_error["y"] - self.derivative_gain * velocity_y
        force_z = self.proportional_gain * error_z + self.integral_gain * self.integral_error["z"] - self.derivative_gain * velocity_z

        def clamp_value(value, limit):
            return max(min(value, limit), -limit)

        force_x_saturated = clamp_value(force_x, self.max_force)
        force_y_saturated = clamp_value(force_y, self.max_force)
        force_z_saturated = clamp_value(force_z, self.max_force)

        return {"x": force_x_saturated, "y": force_y_saturated, "z": force_z_saturated}

    def publish_state(self):
        inverse_message = Inverse3State()
        inverse_message.position = self.position
        inverse_message.velocity = self.velocity
        self.inverse3_state_publisher.publish(inverse_message)

        handle_message = HandleState()
        handle_message.quaternion = self.quaternion
        handle_message.buttons = self.buttons
        self.handle_state_publisher.publish(handle_message)

        haply_message = HaplyState()
        haply_message.position = self.position
        haply_message.velocity = self.velocity
        haply_message.quaternion = self.quaternion
        haply_message.buttons = self.buttons
        self.haply_state_publisher.publish(haply_message)

        elapsed_time = int(time.time() - self.start_time)
        sys.stdout.write(f"\rhaply_driver_node is running: {elapsed_time} s")
        sys.stdout.flush()

    def configure_gravity_compensation(self, device_identifier: str, scaling_factor: float) -> None:
        service_url = "http://localhost:10000/gravity_compensation"
        payload = {
            "device_id": device_identifier,
            "enable": True,
            "gravity_scaling_factor": scaling_factor
        }
        encoded_data = json.dumps(payload).encode("utf-8")
        request = urllib.request.Request(service_url, data=encoded_data, headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(request, timeout=2.0) as response:
                if response.getcode() == 200:
                    self.get_logger().info(f"Gravity compensation configured for {device_identifier}.")
                else:
                    self.get_logger().warning("Failed to configure gravity compensation.")
        except Exception as exception:
            self.get_logger().error(f"HTTP request failed: {exception}")

    def start_asynchronous_loop(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        try:
            async with websockets.connect(self.websocket_uniform_resource_identifier) as websocket_connection:
                await websocket_connection.send(orjson.dumps({
                    "session": {"force_render_full_state": {}}
                }))

                first_message = True

                while self.run_execution:
                    data_json = await websocket_connection.recv()
                    data = orjson.loads(data_json)

                    inverse = data.get("inverse3", []) or []
                    handle  = data.get("wireless_verse_grip", []) or []

                    self.inverse_available = len(inverse) > 0
                    self.handle_available  = len(handle)  > 0

                    if self.inverse_available or self.handle_available:
                        self.last_device_seen_time = time.perf_counter()

                    if not inverse:
                        if not self.inverse_warned:
                            self.get_logger().warn("Inverse3 not found!")
                            self.inverse_warned = True
                    else:
                        self.inverse_warned = False

                    if not handle:
                        if not self.handle_warned:
                            self.get_logger().warn("Handle not found!")
                            self.handle_warned = True
                    else:
                        self.handle_warned = False

                    if first_message:
                        if self.inverse_available:
                            inverse_data = inverse[0]
                            self.inverse3_device_identifier = inverse_data.get("device_id")

                            self.configure_gravity_compensation(self.inverse3_device_identifier, 0.025)

                            self.get_logger().info(
                                f"\nInverse3:\n"
                                f"\tid: {inverse_data.get('device_id')}\n"
                                f"\tport: {inverse_data.get('config', {}).get('port')}\n"
                                f"\t\"status\": {{\n"
                                f"\t\tcalibrated: {inverse_data.get('status', {}).get('calibrated')},\n"
                                f"\t\tin_use: {inverse_data.get('status', {}).get('in_use')},\n"
                                f"\t\tpower_supply: {inverse_data.get('status', {}).get('power_supply')},\n"
                                f"\t\tready: {inverse_data.get('status', {}).get('ready')},\n"
                                f"\t\tstarted: {inverse_data.get('status', {}).get('started')},\n"
                                f"\t}}"
                            )
                        else:
                            self.get_logger().info("\nInverse3:\n\tNot available!")

                        if self.handle_available:
                            handle_data = handle[0]
                            self.get_logger().info(
                                f"\nHandle:\n"
                                f"\tid: {handle_data.get('device_id')}\n"
                                f"\tport: {handle_data.get('config', {}).get('port')}\n"
                                f"\tbattery_level: {handle_data.get('state', {}).get('battery_level', 0.0) * 100:.0f}%\n"
                                f"\t\"status\": {{\n"
                                f"\t\tconnected: {handle_data.get('status', {}).get('connected')},\n"
                                f"\t\tawake: {handle_data.get('status', {}).get('awake')},\n"
                                f"\t\tready: {handle_data.get('status', {}).get('ready')}\n"
                                f"\t}}"
                            )
                            if not handle_data.get('status', {}).get('awake'):
                                self.get_logger().warning("Warning: Handle is not awake")
                        else:
                            self.get_logger().info("\nHandle:\n\tNot available!")

                        first_message = False

                    if self.inverse_available:
                        inverse_data = inverse[0]
                        position_dictionary = inverse_data["state"].get("cursor_position", {})
                        velocity_dictionary = inverse_data["state"].get("cursor_velocity", {})
                        self.position = Point(
                            x=float(position_dictionary.get("x", 0.0) or 0.0),
                            y=float(position_dictionary.get("y", 0.0) or 0.0),
                            z=float(position_dictionary.get("z", 0.0) or 0.0)
                        )
                        self.velocity = Vector3(
                            x=float(velocity_dictionary.get("x", 0.0) or 0.0),
                            y=float(velocity_dictionary.get("y", 0.0) or 0.0),
                            z=float(velocity_dictionary.get("z", 0.0) or 0.0)
                        )

                    if self.handle_available:
                        handle_state = handle[0].get("state", {})
                        orientation_dictionary = handle_state.get("orientation", {})
                        self.quaternion = Quaternion(
                            x=float(orientation_dictionary.get("x", 0.0) or 0.0),
                            y=float(orientation_dictionary.get("y", 0.0) or 0.0),
                            z=float(orientation_dictionary.get("z", 0.0) or 0.0),
                            w=float(orientation_dictionary.get("w", 1.0) or 1.0),
                        )
                        buttons_dictionary = handle_state.get("buttons", {})
                        self.buttons.a = bool(buttons_dictionary.get("a", False))
                        self.buttons.b = bool(buttons_dictionary.get("b", False))
                        self.buttons.c = bool(buttons_dictionary.get("c", False))

                    if time.perf_counter() - self.last_message_time > self.timeout_seconds and self.control_active and not self.use_target_position:
                        self.control_active = False
                        self.target_force = {"x": 0.0, "y": 0.0, "z": 0.0}
                        self.get_logger().warn(f"No force command received for {self.timeout_seconds:.1f/100}s, disabling control.")

                    if self.use_target_position and self.control_active:
                        self.target_force = self.calculate_force()

                    if self.inverse_available:
                        request_payload = {
                            "inverse3": [
                                {
                                    "device_id": self.inverse3_device_identifier,
                                    "commands": {
                                        "set_cursor_force": {"values": self.target_force}
                                    }
                                }
                            ]
                        }
                        await websocket_connection.send(orjson.dumps(request_payload))

                    if not self.inverse_available and not self.handle_available:
                        if (time.perf_counter() - self.last_device_seen_time) > 200:
                            self.get_logger().warn("No devices available. Shutting down node.")
                            self.run_execution = False
                            rclpy.shutdown()
                            return

        except Exception as exception:
            self.get_logger().error(f"WebSocket error: {exception}")
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        self.run_execution = False
        self.websocket_thread.join()
        super().destroy_node()

def main(arguments=None):
    rclpy.init(args=arguments)
    node = HaplyDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
