#!/usr/bin/env python3
"""Fake base provider with the same Platform API as the real bridge."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from robot_interfaces.msg import (
    ChassisCommand,
    ChassisState,
    FirmwareInfo,
    ImuObservation,
    WheelObservation,
)
from sensor_msgs.msg import BatteryState
from std_srvs.srv import SetBool, Trigger

from robot_verification.fake_base_model import FakeBaseModel, StatusSampleLatch


STATE_QOS = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
)


def _sequence_is_forward(new: int, old: int) -> bool:
    delta = (int(new) - int(old)) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class FakeBaseNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_base")
        defaults = {
            "config_sha256": "development-uncompiled",
            "chassis_command_topic": "chassis/command",
            "wheel_observation_topic": "wheel/observation",
            "imu_observation_topic": "imu/observation",
            "chassis_state_topic": "chassis/state",
            "firmware_info_topic": "chassis/firmware_info",
            "diagnostics_topic": "diagnostics",
            "base_frame_id": "base_link",
            "imu_frame_id": "imu_link",
            "wheel_radius": 0.035,
            "wheel_track_width": 0.176,
            "encoder_counts_per_revolution": 2464.0,
            "publish_hz": 50.0,
            "cmd_timeout": 0.15,
            "response_tau_sec": 0.10,
            "scenario": "normal",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.config_sha256 = str(self.get_parameter("config_sha256").value)
        self.base_frame = str(self.get_parameter("base_frame_id").value)
        self.imu_frame = str(self.get_parameter("imu_frame_id").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.scenario = str(self.get_parameter("scenario").value)
        self.model = FakeBaseModel(
            wheel_radius_m=float(self.get_parameter("wheel_radius").value),
            track_width_m=float(self.get_parameter("wheel_track_width").value),
            counts_per_revolution=float(self.get_parameter("encoder_counts_per_revolution").value),
            response_tau_sec=float(self.get_parameter("response_tau_sec").value),
        )
        self.transport_session_id = 0xFA4E000000000001
        self.status_latch = StatusSampleLatch()
        self.status_sequence = 0
        self.imu_sequence = 0
        self.tick_count = 0
        self.mcu_time_ms = 0
        self.last_tick = time.monotonic()
        self.started_at = self.last_tick
        self.last_command_at = None
        self.command_session_id = 0
        self.command_sequence = 0
        self.selected_source = 0
        self.enabled = False
        # Match a newly connected real transport: motion is closed until an
        # explicit upper-layer disable is followed by a fresh enable command.
        self.rearm_required = True
        self.rearm_reason_flags = 1
        self.estop = False
        self.disconnect_recovered = False
        self.static_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.wheel_pub = self.create_publisher(
            WheelObservation,
            str(self.get_parameter("wheel_observation_topic").value),
            STATE_QOS,
        )
        self.imu_pub = self.create_publisher(
            ImuObservation,
            str(self.get_parameter("imu_observation_topic").value),
            qos_profile_sensor_data,
        )
        self.state_pub = self.create_publisher(
            ChassisState,
            str(self.get_parameter("chassis_state_topic").value),
            STATE_QOS,
        )
        self.firmware_pub = self.create_publisher(
            FirmwareInfo, str(self.get_parameter("firmware_info_topic").value), self.static_qos
        )
        self.battery_pub = self.create_publisher(BatteryState, "battery_state", 10)
        self.diag_pub = self.create_publisher(
            DiagnosticArray,
            str(self.get_parameter("diagnostics_topic").value),
            STATE_QOS,
        )
        self.create_subscription(
            ChassisCommand,
            str(self.get_parameter("chassis_command_topic").value),
            self._on_command,
            10,
        )
        self.create_service(SetBool, "chassis/estop", self._on_estop)
        self.create_service(Trigger, "chassis/clear_fault", self._on_clear_fault)
        self.create_service(SetBool, "chassis/line_ctrl", self._on_line_ctrl)
        hz = max(float(self.get_parameter("publish_hz").value), 1.0)
        self.create_timer(1.0 / hz, self._tick)
        self.create_timer(1.0, self._publish_diagnostics)
        self._publish_firmware_info()

    def _on_command(self, msg: ChassisCommand) -> None:
        session, sequence = int(msg.session_id), int(msg.sequence)
        if session == 0:
            return
        if session == self.command_session_id and not _sequence_is_forward(
            sequence, self.command_sequence
        ):
            return
        self.command_session_id, self.command_sequence = session, sequence
        if not msg.enable:
            self.enabled = False
            self.selected_source = 0
            self.rearm_required = False
            self.rearm_reason_flags = 0
            self.last_command_at = None
            self.model.release()
            return
        if self.rearm_required or self.estop:
            return
        self.enabled = True
        self.selected_source = int(msg.source)
        self.last_command_at = time.monotonic()
        self.model.set_target(msg.twist.linear.x, msg.twist.angular.z)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(min(now - self.last_tick, 0.25), 0.0)
        self.last_tick = now
        if self.last_command_at is not None and now - self.last_command_at > self.cmd_timeout:
            self.enabled = False
            self.model.release()
            self.last_command_at = None
            self.rearm_required = True
            self.rearm_reason_flags |= 1 << 6
        elapsed = now - self.started_at
        if self.scenario == "disconnect" and elapsed >= 2.0:
            return
        if self.scenario == "disconnect_reconnect":
            if 2.0 <= elapsed < 3.0:
                return
            if elapsed >= 3.0 and not self.disconnect_recovered:
                self.disconnect_recovered = True
                self.transport_session_id = (self.transport_session_id + 1) & 0xFFFFFFFFFFFFFFFF
                self.status_latch = StatusSampleLatch()
                self.status_sequence = 0
                self.enabled = False
                self.selected_source = 0
                self.last_command_at = None
                self.model.release()
                self.rearm_required = True
                self.rearm_reason_flags |= 1
        if self.scenario == "status_freeze" and elapsed >= 2.0:
            return
        slip = 0.55 if self.scenario == "wheel_slip" and elapsed >= 2.0 else 1.0
        sample = self.model.step(dt, slip_scale=slip)
        self.mcu_time_ms = (self.mcu_time_ms + int(round(dt * 1000.0))) & 0xFFFFFFFF
        self.tick_count += 1
        duplicate = self.scenario == "duplicate_status" and self.tick_count % 10 == 0
        anomaly_mask = 0x01 if self.scenario == "encoder_fault" and elapsed >= 2.0 else 0
        candidate_snapshot = (
            sample,
            self.mcu_time_ms,
            self.enabled,
            self.estop,
            anomaly_mask,
        )
        self.status_sequence, wheel_snapshot = self.status_latch.update(
            candidate_snapshot, repeat=duplicate
        )
        self.imu_sequence = (self.imu_sequence + 1) & 0xFFFFFFFF
        self._publish_observations(wheel_snapshot, sample, elapsed)
        self._publish_state()

    def _publish_observations(self, wheel_snapshot, imu_sample, elapsed: float) -> None:
        stamp = self.get_clock().now().to_msg()
        wheel_sample, wheel_time_ms, wheel_enabled, wheel_estop, anomaly_mask = wheel_snapshot
        wheel = WheelObservation()
        wheel.header.stamp, wheel.header.frame_id = stamp, self.base_frame
        wheel.schema_version = WheelObservation.SCHEMA_VERSION
        wheel.transport_session_id = self.transport_session_id
        wheel.status_sequence = self.status_sequence
        wheel.mcu_sample_time_ms = wheel_time_ms
        wheel.encoder_count = list(wheel_sample.encoder_counts)
        wheel.wheel_speed_mps = list(wheel_sample.wheel_speeds)
        wheel.wheel_target_mps = list(wheel_sample.wheel_targets)
        wheel.motor_current_a = [0.0] * 4
        wheel.motor_output_permille = [0] * 4
        wheel.motor_enabled_mask = 0x0F if wheel_enabled else 0
        wheel.speed_valid_mask = 0x0F
        wheel.encoder_anomaly_mask = anomaly_mask
        wheel.status_flags = 1 if wheel_estop else 0
        self.wheel_pub.publish(wheel)

        imu = ImuObservation()
        imu.header.stamp, imu.header.frame_id = stamp, self.imu_frame
        imu.schema_version = ImuObservation.SCHEMA_VERSION
        imu.transport_session_id = self.transport_session_id
        imu.mcu_sample_time_ms = self.mcu_time_ms
        imu.sensor_time = self.mcu_time_ms
        imu.sample_sequence = self.imu_sequence
        imu.acceleration_g = [0.0, 0.0, 1.0]
        drift = 0.5 if self.scenario == "imu_drift" and elapsed >= 2.0 else 0.0
        imu.angular_velocity_dps = [0.0, 0.0, math.degrees(imu_sample.wz + drift)]
        imu.orientation.w = 1.0
        imu.status_flags = 0x0B
        self.imu_pub.publish(imu)

        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage, battery.present = 12.0, True
        self.battery_pub.publish(battery)

    def _publish_state(self) -> None:
        msg = ChassisState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.bridge_state = ChassisState.STATE_ACTIVE if self.enabled else ChassisState.STATE_READY
        msg.selected_source = self.selected_source
        msg.command_session_id, msg.command_sequence = (
            self.command_session_id,
            self.command_sequence,
        )
        msg.upper_enabled = self.enabled
        msg.protocol_version = 3
        msg.wire_session_id = self.transport_session_id
        msg.wire_sent_sequence = self.command_sequence
        msg.firmware_ack_available = True
        msg.firmware_session_id = self.command_session_id
        msg.firmware_received_sequence = self.command_sequence
        msg.firmware_applied_sequence = self.command_sequence
        msg.rearm_required = self.rearm_required
        msg.rearm_reason_flags = self.rearm_reason_flags
        msg.slip_score, msg.supervisor_level = -1.0, 0
        msg.config_sha256 = self.config_sha256
        self.state_pub.publish(msg)

    def _publish_firmware_info(self) -> None:
        msg = FirmwareInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.protocol_version = 3
        msg.schema_version = 1
        msg.capabilities = 0x1F
        msg.firmware_commit = ""
        msg.hardware_revision = 0
        msg.parameter_crc32 = 0
        msg.simulated = True
        self.firmware_pub.publish(msg)

    def _publish_diagnostics(self) -> None:
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus(
            level=DiagnosticStatus.WARN if self.rearm_required else DiagnosticStatus.OK,
            name="fake_base/platform",
            message="rearm required" if self.rearm_required else self.scenario,
            hardware_id="simulated",
        )
        status.values = [KeyValue(key="scenario", value=self.scenario)]
        array.status = [status]
        self.diag_pub.publish(array)

    def _on_estop(self, request, response):
        if not request.data:
            response.success, response.message = False, "remote ESTOP release is forbidden"
            return response
        self.estop = True
        self.enabled = False
        self.model.release()
        self.rearm_required = True
        self.rearm_reason_flags |= 1 << 4
        response.success, response.message = True, "fake ESTOP set"
        return response

    def _on_clear_fault(self, _request, response):
        self.estop = False
        response.success, response.message = True, "fake fault cleared; rearm still required"
        return response

    @staticmethod
    def _on_line_ctrl(_request, response):
        response.success, response.message = True, "fake line control accepted"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
