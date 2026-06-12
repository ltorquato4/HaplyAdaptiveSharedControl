#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import asyncio
import threading
import time
import websockets
import orjson
import traceback
import sys

from geometry_msgs.msg import Point, Vector3
from haply_msgs.msg import Inverse3State, HaplyControl


class Inverse3DriverNode(Node):
    def __init__(self):
        super().__init__("inverse3_driver_node")

        # Parameters
        self.declare_parameter("frequency", 200.0)
        self.declare_parameter("max_force", 10.0)      # Maximum allowed force in N
        self.frequency = float(self.get_parameter("frequency").value)
        self.max_force = float(self.get_parameter("max_force").value)

        # PID controller
        self.Kp = 30.0   # proportional gain [N/m]
        self.Ki = 5.0    # integral gain [N/(m·s)]
        self.Kd = 0.9    # derivative gain [N·s/m]
        self.integral_error = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_update_time = time.perf_counter()

        # State variables
        self.position = Point(x=0.0, y=0.0, z=0.0)
        self.velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # Control variables
        self.control_active = False
        self.use_target_position = False
        self.target_position = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.target_force = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.last_msg_time = time.perf_counter()
        self.timeout_s = 50

        self.start_time = time.time()

        # WebSocket state
        self.inverse_available = False
        self.inverse3_device_id = None
        self.ws_uri = "ws://localhost:10001"

        # Subscriber
        self.create_subscription(HaplyControl, "haply_target", self.control_msg, 10)

        # Publisher
        self.inverse3_state_pub = self.create_publisher(Inverse3State, "inverse3_state", 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_state)

        # WebSocket loop in a separate thread
        self.run = True
        self.ws_thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.ws_thread.start()

    def control_msg(self, msg: HaplyControl):
        """Callback for HaplyControl topic. Updates control mode and target values."""
        self.last_msg_time = time.perf_counter()
        self.control_active = True
        self.use_target_position = bool(msg.use_position)

        if self.use_target_position:
            self.target_position = {
                "x": float(msg.target_position.x),
                "y": float(msg.target_position.y),
                "z": float(msg.target_position.z),
            }
        else:
            self.target_force = {
                "x": float(msg.force.x),
                "y": float(msg.force.y),
                "z": float(msg.force.z),
            }

    def calculate_force(self):
        """PID control: F = Kp*e + Ki*∫e dt - Kd*v."""
        now = time.perf_counter()
        dt = now - self.last_update_time
        if dt <= 0.0:
            dt = 1e-6
        self.last_update_time = now

        # position error
        ex = float(self.target_position["x"]) - float(self.position.x)
        ey = float(self.target_position["y"]) - float(self.position.y)
        ez = float(self.target_position["z"]) - float(self.position.z)

        # integrate error
        self.integral_error["x"] += ex * dt
        self.integral_error["y"] += ey * dt
        self.integral_error["z"] += ez * dt

        # derivative term uses measured velocity directly (v)
        vx = float(self.velocity.x)
        vy = float(self.velocity.y)
        vz = float(self.velocity.z)

        # raw PID forces
        fx = self.Kp*ex + self.Ki*self.integral_error["x"] - self.Kd*vx
        fy = self.Kp*ey + self.Ki*self.integral_error["y"] - self.Kd*vy
        fz = self.Kp*ez + self.Ki*self.integral_error["z"] - self.Kd*vz

        # saturation
        def clamp(v, lim): 
            return max(min(v, lim), -lim)
        
        fx_sat, fy_sat, fz_sat = clamp(fx, self.max_force), clamp(fy, self.max_force), clamp(fz, self.max_force)

        return {"x": fx_sat, "y": fy_sat, "z": fz_sat}

    def publish_state(self):
        # Inverse3 state
        msg = Inverse3State()
        msg.position = self.position
        msg.velocity = self.velocity
        self.inverse3_state_pub.publish(msg)

        # Uptime print
        elapsed = int(time.time() - self.start_time)
        sys.stdout.write(f"\rinverse3_driver_node is running: {elapsed} s")
        sys.stdout.flush()

    def start_async_loop(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        try:
            async with websockets.connect(self.ws_uri) as ws:
                await ws.send(orjson.dumps({
                    "session": {"force_render_full_state": {"device_types": ["inverse3"]}}
                }))

                first_message = True

                while self.run:
                    data_json = await ws.recv()
                    data = orjson.loads(data_json)

                    inverse = data.get("inverse3", []) or []
                    self.inverse_available = len(inverse) > 0
                    if not self.inverse_available:
                        self.get_logger().warn("Inverse3 not found")
                        continue

                    # First message: print info
                    if first_message:
                        if self.inverse_available:
                            inverse_data = inverse[0]
                            self.inverse3_device_id = inverse_data.get("device_id")
                            self.get_logger().info(
                                f"\nInverse3:\n"
                                f"\tid: {inverse_data.get('device_id')}\n"
                                f"\tport: {inverse_data.get('config', {}).get('port')}\n"
                                f"\t\"status\": {{\n"
                                f"\t\tcalibrated: {inverse_data.get('status', {}).get('calibrated')},\n"
                                f"\t\tin_use: {inverse_data.get('status', {}).get('in_use')},\n"
                                f"\t\tpower_supply: {inverse_data.get('status', {}).get('power_supply')},\n"
                                f"\t\tready: {inverse_data.get('status', {}).get('ready')},\n"
                                f"\t\tstarted: {inverse_data.get('status', {}).get('started')}\n"
                                f"\t}}"
                            )
                        else:
                            self.get_logger().info("\nInverse3:\n\tNot available!")

                    first_message = False

                    # Update current state
                    inverse_data = inverse[0]
                    pos = inverse_data["state"].get("cursor_position", {})
                    vel = inverse_data["state"].get("cursor_velocity", {})
                    self.position = Point(
                        x=float(pos.get("x", 0.0) or 0.0),
                        y=float(pos.get("y", 0.0) or 0.0),
                        z=float(pos.get("z", 0.0) or 0.0),
                    )
                    self.velocity = Vector3(
                        x=float(vel.get("x", 0.0) or 0.0),
                        y=float(vel.get("y", 0.0) or 0.0),
                        z=float(vel.get("z", 0.0) or 0.0),
                    )

                    # Safety: stop force mode if no new force cmd for 0.5s
                    if (time.perf_counter() - self.last_msg_time) > self.timeout_s and \
                       self.control_active and not self.use_target_position:
                        self.control_active = False
                        self.target_force = {"x": 0.0, "y": 0.0, "z": 0.0}
                        self.get_logger().warn(
                            f"No force command for {self.timeout_s:.1f / 100}s, zeroing force."
                        )

                    # Position control force update
                    if self.use_target_position and self.control_active:
                        self.target_force = self.calculate_force()

                    # Send force command to device
                    if self.inverse3_device_id:
                        request = {
                            "inverse3": [
                                {
                                    "device_id": self.inverse3_device_id,
                                    "commands": {
                                        "set_cursor_force": {"values": self.target_force}
                                    }
                                }
                            ]
                        }
                        await ws.send(orjson.dumps(request))

                    # If the device is missing for > 200s, shutdown
                    if not self.inverse_available and not self.handle_available:
                        if (time.perf_counter() - self.last_device_seen_time) > 200:
                            self.get_logger().warn("No devices available. Shutting down node.")
                            self.run = False
                            rclpy.shutdown()
                            return

        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
            self.get_logger().error(traceback.format_exc())
            self.run = False
            rclpy.shutdown()

    def destroy_node(self):
        self.run = False
        self.ws_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Inverse3DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
