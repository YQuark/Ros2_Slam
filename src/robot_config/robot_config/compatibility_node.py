#!/usr/bin/env python3
"""Publish the machine-readable runtime compatibility gate."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import FirmwareInfo, PlatformCompatibilityState, WheelObservation

from robot_config.compatibility import evaluate_compatibility


class PlatformCompatibilityNode(Node):
    def __init__(self) -> None:
        super().__init__("platform_compatibility")
        defaults = {
            "firmware_info_topic": "chassis/firmware_info",
            "wheel_observation_topic": "wheel/observation",
            "compatibility_state_topic": "platform/compatibility_state",
            "expected_firmware_commit": "366a0385290d526009e6cd3bbdaa7b74b2fecad6",
            "expected_hardware_revision": 0x00020000,
            "required_capabilities": 0x1F,
            "expected_parameter_crc32": 0,
            "expected_enabled_mask": 0b0110,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PlatformCompatibilityState,
            str(self.get_parameter("compatibility_state_topic").value),
            qos,
        )
        self.firmware = None
        self.enabled_mask = None
        self.create_subscription(
            FirmwareInfo,
            str(self.get_parameter("firmware_info_topic").value),
            self._on_firmware,
            qos,
        )
        self.create_subscription(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            self._on_wheel,
            10,
        )

    def _on_firmware(self, msg) -> None:
        self.firmware = msg
        self._publish()

    def _on_wheel(self, msg) -> None:
        self.enabled_mask = int(msg.motor_enabled_mask)
        self._publish()

    def _publish(self) -> None:
        output = PlatformCompatibilityState()
        output.header.stamp = self.get_clock().now().to_msg()
        if self.firmware is None or (self.enabled_mask is None and not self.firmware.simulated):
            output.state = PlatformCompatibilityState.STATE_UNKNOWN
            output.reason = "waiting for HELLO and wheel layout"
        else:
            result = evaluate_compatibility(
                simulated=self.firmware.simulated,
                firmware_commit=self.firmware.firmware_commit,
                expected_firmware_commit=self.get_parameter("expected_firmware_commit").value,
                hardware_revision=self.firmware.hardware_revision,
                expected_hardware_revision=self.get_parameter("expected_hardware_revision").value,
                capabilities=self.firmware.capabilities,
                required_capabilities=self.get_parameter("required_capabilities").value,
                parameter_crc32=self.firmware.parameter_crc32,
                expected_parameter_crc32=self.get_parameter("expected_parameter_crc32").value,
                enabled_mask=self.enabled_mask or 0,
                expected_enabled_mask=self.get_parameter("expected_enabled_mask").value,
            )
            output.state = (
                PlatformCompatibilityState.STATE_SIMULATION
                if result.simulated
                else (
                    PlatformCompatibilityState.STATE_COMPATIBLE
                    if result.compatible
                    else PlatformCompatibilityState.STATE_INCOMPATIBLE
                )
            )
            output.mismatch_flags = result.mismatch_flags
            output.reason = result.reason
            output.permit_formal_odometry = result.compatible
            output.permit_navigation = result.compatible
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlatformCompatibilityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
