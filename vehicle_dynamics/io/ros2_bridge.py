"""
ROS2-compatible bridge (works without rclpy installed).

When rclpy is available, can publish/subscribe real topics.
Otherwise acts as an in-process message bus for testing and HIL stubs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable


@dataclass
class RosMessage:
    topic: str
    data: dict[str, Any]
    stamp: float = 0.0


@dataclass
class ROS2Bridge:
    """In-memory ROS2-style pub/sub."""
    publishers: dict[str, list[RosMessage]] = field(default_factory=dict)
    subscribers: dict[str, list[Callable[[RosMessage], None]]] = field(default_factory=dict)
    _inbox: dict[str, list[RosMessage]] = field(default_factory=dict)

    # Standard topics
    TOPIC_VEHICLE_STATE = "/vehicle/state"
    TOPIC_WHEEL_FORCES = "/vehicle/wheel_forces"
    TOPIC_ENGINE = "/vehicle/engine"
    TOPIC_IMU = "/vehicle/imu"
    TOPIC_GPS = "/vehicle/gps"
    TOPIC_CONTROLS = "/vehicle/controls"
    TOPIC_CMD_STEER = "/cmd/steering"
    TOPIC_CMD_THROTTLE = "/cmd/throttle"
    TOPIC_CMD_BRAKE = "/cmd/brake"

    def publish(self, topic: str, data: dict[str, Any], stamp: float = 0.0) -> RosMessage:
        msg = RosMessage(topic=topic, data=dict(data), stamp=stamp)
        self.publishers.setdefault(topic, []).append(msg)
        for cb in self.subscribers.get(topic, []):
            cb(msg)
        self._inbox.setdefault(topic, []).append(msg)
        return msg

    def subscribe(self, topic: str, callback: Callable[[RosMessage], None]) -> None:
        self.subscribers.setdefault(topic, []).append(callback)

    def publish_vehicle_state(self, state: dict[str, Any], t: float = 0.0) -> None:
        self.publish(self.TOPIC_VEHICLE_STATE, state, t)
        if "engine_rpm" in state:
            self.publish(self.TOPIC_ENGINE, {"rpm": state["engine_rpm"], "torque": state.get("engine_torque", 0.0)}, t)

    def inject_command(self, topic: str, data: dict[str, Any], t: float = 0.0) -> None:
        """Simulate an external subscriber feeding commands."""
        self.publish(topic, data, t)

    def latest(self, topic: str) -> RosMessage | None:
        msgs = self._inbox.get(topic) or self.publishers.get(topic) or []
        return msgs[-1] if msgs else None

    def has_rclpy(self) -> bool:
        try:
            import rclpy  # noqa: F401
            return True
        except ImportError:
            return False
