#!/usr/bin/env python3
"""
Velocity Arbiter (Multiplexer) for YILDIZ USV

Priority-based velocity command selector that prevents
conflicts between multiple control sources.

Priority Levels (lower = higher priority):
1. avoid  - Emergency obstacle avoidance (highest)
2. nav    - Waypoint navigation
3. task   - Mission tasks (kamikaze, target tracking)
4. explore - Autonomous exploration (lowest)

Features:
- Timeout-based source detection
- Smooth handoff between sources
- Status reporting

Subscribes:
- /cmd_vel_avoid (Twist): Obstacle avoidance commands
- /cmd_vel_nav (Twist): Navigation commands
- /cmd_vel_task (Twist): Task/mission commands

Publishes:
- /cmd_vel_mux (Twist): Selected velocity command

Author: YILDIZ USV Team
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from typing import Optional, Dict
from dataclasses import dataclass


@dataclass
class VelocitySource:
    """Represents a velocity command source."""
    name: str
    priority: int  # Lower = higher priority
    topic: str
    last_cmd: Optional[Twist] = None
    last_time: Optional[Time] = None
    active: bool = False


class VelocityArbiter(Node):
    def __init__(self):
        super().__init__('velocity_arbiter')

        # Declare parameters
        self.declare_parameter('rate', 50.0)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('output_topic', '/cmd_vel_mux')

        # Priority parameters
        self.declare_parameter('priority_avoid', 1)
        self.declare_parameter('priority_nav', 2)
        self.declare_parameter('priority_task', 3)

        # Activity threshold (commands below this are considered "inactive")
        self.declare_parameter('activity_threshold', 0.01)

        # Get parameters
        self.rate = self.get_parameter('rate').value
        self.timeout = self.get_parameter('timeout').value
        output_topic = self.get_parameter('output_topic').value
        self.activity_threshold = self.get_parameter('activity_threshold').value

        priority_avoid = self.get_parameter('priority_avoid').value
        priority_nav = self.get_parameter('priority_nav').value
        priority_task = self.get_parameter('priority_task').value

        self.dt = 1.0 / self.rate

        # QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Define velocity sources
        self.sources: Dict[str, VelocitySource] = {
            'avoid': VelocitySource(
                name='avoid',
                priority=priority_avoid,
                topic='/cmd_vel_avoid'
            ),
            'nav': VelocitySource(
                name='nav',
                priority=priority_nav,
                topic='/cmd_vel_nav'
            ),
            'task': VelocitySource(
                name='task',
                priority=priority_task,
                topic='/cmd_vel_task'
            ),
        }

        # Create subscribers for each source
        self.subscribers = {}
        for name, source in self.sources.items():
            self.subscribers[name] = self.create_subscription(
                Twist,
                source.topic,
                lambda msg, n=name: self._cmd_callback(msg, n),
                reliable_qos
            )

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, output_topic, reliable_qos)
        self.status_pub = self.create_publisher(String, '/arbiter/status', reliable_qos)

        # Current active source
        self.active_source = 'none'
        self.last_published_cmd = Twist()

        # Control timer
        self.timer = self.create_timer(self.dt, self.control_loop)

        self.get_logger().info(
            f'Velocity Arbiter started: -> {output_topic} @ {self.rate}Hz'
        )
        self.get_logger().info(f'Sources: {list(self.sources.keys())}')

    def _cmd_callback(self, msg: Twist, source_name: str):
        """Receive velocity command from a source."""
        source = self.sources.get(source_name)
        if source is None:
            return

        source.last_cmd = msg
        source.last_time = self.get_clock().now()

        # Check if command is "active" (non-zero velocity)
        is_active = (
            abs(msg.linear.x) > self.activity_threshold or
            abs(msg.angular.z) > self.activity_threshold
        )
        source.active = is_active

    def control_loop(self):
        """Select highest priority active source and publish."""
        current_time = self.get_clock().now()

        # Update source activity based on timeout
        for source in self.sources.values():
            if source.last_time is None:
                source.active = False
                continue

            age = (current_time - source.last_time).nanoseconds / 1e9
            if age > self.timeout:
                source.active = False

        # Select highest priority active source
        selected_source = None
        selected_cmd = Twist()  # Default: zero velocity

        # Sort by priority (lower = higher priority)
        sorted_sources = sorted(
            self.sources.values(),
            key=lambda s: s.priority
        )

        for source in sorted_sources:
            if source.active and source.last_cmd is not None:
                selected_source = source
                selected_cmd = source.last_cmd
                break

        # Update active source name
        if selected_source is not None:
            self.active_source = selected_source.name
        else:
            self.active_source = 'none'

        # Publish selected command
        self.cmd_pub.publish(selected_cmd)
        self.last_published_cmd = selected_cmd

        # Publish status
        self._publish_status()

    def _publish_status(self):
        """Publish arbiter status for debugging."""
        status_parts = []

        for name, source in self.sources.items():
            state = 'A' if source.active else '-'
            if source.last_cmd is not None:
                lin = source.last_cmd.linear.x
                ang = source.last_cmd.angular.z
                status_parts.append(f"{name}[{state}]:{lin:.1f}/{ang:.1f}")
            else:
                status_parts.append(f"{name}[{state}]:---")

        status = String()
        status.data = f"SEL:{self.active_source} | {' '.join(status_parts)}"
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publish zero velocity on shutdown
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
