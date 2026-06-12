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

from geometry_msgs.msg import Quaternion
from haply_msgs.msg import HandleState, HandleButtons


class HandleDriverNode(Node):
    def __init__(self):
        super().__init__("handle_driver_node")

        # Parameters
        self.declare_parameter("frequency", 200.0)
        self.frequency = float(self.get_parameter("frequency").value)

        # State variables
        self.quaternion = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.buttons = HandleButtons(a=False, b=False, c=False)

        self.start_time = time.time()

        # WebSocket state
        self.handle_available = False
        self.handle_device_id = None
        self.ws_uri = "ws://localhost:10001"

        # Publishers
        self.handle_state_pub = self.create_publisher(HandleState, "handle_state", 10)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_state)

        # WebSocket loop in a separate thread
        self.run = True
        self.ws_thread = threading.Thread(target=self.start_async_loop, daemon=True)
        self.ws_thread.start()

    def publish_state(self):
        # Handle state
        msg = HandleState()
        msg.quaternion = self.quaternion
        msg.buttons = self.buttons
        self.handle_state_pub.publish(msg)

        # Uptime print
        elapsed = int(time.time() - self.start_time)
        sys.stdout.write(f"\rhandle_driver_node is running: {elapsed} s")
        sys.stdout.flush()

    def start_async_loop(self):
        asyncio.run(self.websocket_loop())

    async def websocket_loop(self):
        try:
            async with websockets.connect(self.ws_uri) as ws:
                await ws.send(orjson.dumps({
                    "session": {"force_render_full_state": {"device_types": ["wireless_verse_grip"]}}
                }))

                first_message = True

                while self.run:
                    data_json = await ws.recv()
                    data = orjson.loads(data_json)

                    handle = data.get("wireless_verse_grip", []) or []

                    self.handle_available = len(handle) > 0

                    if not self.handle_available:
                        self.get_logger().warn("Handle not found")
                        continue

                    handle_data = handle[0]
                    handle_state = handle_data.get("state", {})

                    if first_message:
                        if self.handle_available:
                            handle_data = handle[0]
                            self.handle_device_id = handle_data.get("device_id")
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

                    # Handle state
                    if self.handle_available:
                        handle_state = handle[0].get("state", {})
                        orientation = handle_state.get("orientation", {})
                        self.quaternion = Quaternion(
                            x=float(orientation.get("x", 0.0) or 0.0),
                            y=float(orientation.get("y", 0.0) or 0.0),
                            z=float(orientation.get("z", 0.0) or 0.0),
                            w=float(orientation.get("w", 1.0) or 1.0),
                        )
                        buttons_dict = handle_state.get("buttons", {})
                        self.buttons.a = bool(buttons_dict.get("a", False))
                        self.buttons.b = bool(buttons_dict.get("b", False))
                        self.buttons.c = bool(buttons_dict.get("c", False))

                    if self.handle_device_id:
                        keep_alive = {
                            "wireless_verse_grip": [
                                {
                                    "device_id": self.handle_device_id,
                                    "commands": {"keep_alive": True}
                                }
                            ]
                        }
                        await ws.send(orjson.dumps(keep_alive))

        except Exception as e:
            self.get_logger().error(f"WebSocket error: {e}")
            self.get_logger().error(traceback.format_exc())

    def destroy_node(self):
        self.run = False
        self.ws_thread.join()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandleDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
