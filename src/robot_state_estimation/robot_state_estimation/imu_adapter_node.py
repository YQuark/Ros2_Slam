#!/usr/bin/env python3
"""Convert raw Upper-v3 IMU observations to field-valid sensor_msgs/Imu."""

import math

import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import ImuObservation
from sensor_msgs.msg import Imu

from robot_state_estimation.imu_processing import (
    GRAVITY_MPS2,
    IMU_QUALITY_SENSOR_RESET,
    classify_quality,
    finite_vector,
    normalize_quaternion,
)
from robot_state_estimation.time_mapper import (
    McuClockMapper,
    SampleDisposition,
    SampleOrderTracker,
)


class ImuAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("imu_adapter")
        defaults = {
            "config_sha256": "development-uncompiled",
            "observation_topic": "imu/observation",
            "imu_topic": "imu/data",
            "frame_id": "imu_link",
            "use_orientation": False,
            "orientation_stddev": 0.2,
            "angular_velocity_stddev": [0.02, 0.02, 0.02],
            "linear_acceleration_stddev": [0.2, 0.2, 0.2],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.use_orientation = bool(self.get_parameter("use_orientation").value)
        self.orientation_stddev = float(self.get_parameter("orientation_stddev").value)
        self.gyro_stddev = tuple(
            float(value) for value in self.get_parameter("angular_velocity_stddev").value
        )
        self.accel_stddev = tuple(
            float(value) for value in self.get_parameter("linear_acceleration_stddev").value
        )
        self.order = SampleOrderTracker()
        self.clock_mapper = McuClockMapper()
        self.publisher = self.create_publisher(
            Imu, str(self.get_parameter("imu_topic").value), qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            ImuObservation,
            str(self.get_parameter("observation_topic").value),
            self._on_observation,
            qos_profile_sensor_data,
        )
        self.rejected_count = 0

    def _on_observation(self, msg: ImuObservation) -> None:
        if int(msg.schema_version) != ImuObservation.SCHEMA_VERSION:
            self.rejected_count += 1
            return
        validity = classify_quality(msg.quality_flags, msg.status_flags)
        if int(msg.quality_flags) & IMU_QUALITY_SENSOR_RESET:
            self.order.reset()
            self.clock_mapper.reset()
        disposition = self.order.update(msg.transport_session_id, msg.sample_sequence)
        if disposition in (SampleDisposition.DUPLICATE, SampleDisposition.OUT_OF_ORDER):
            self.rejected_count += 1
            return
        if disposition is SampleDisposition.FIRST:
            self.clock_mapper.reset()
        receive_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        gyro = finite_vector(msg.angular_velocity_dps, 3)
        if not validity.gyro_valid or gyro is None:
            self.rejected_count += 1
            return
        sample_ros_sec = receive_sec
        if validity.timestamp_valid:
            sample_ros_sec = self.clock_mapper.update(
                msg.mcu_sample_time_ms, receive_sec
            ).sample_ros_sec
        else:
            self.clock_mapper.reset()
        imu = Imu()
        imu.header.stamp = self._time_message(sample_ros_sec)
        imu.header.frame_id = self.frame_id
        warning_multiplier = 4.0 if validity.warning else 1.0
        orientation = normalize_quaternion(
            (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        )
        if not self.use_orientation or not validity.orientation_valid or orientation is None:
            imu.orientation_covariance[0] = -1.0
        else:
            (
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
                imu.orientation.w,
            ) = orientation
            for index in (0, 4, 8):
                imu.orientation_covariance[index] = self.orientation_stddev**2 * warning_multiplier
        imu.angular_velocity.x = math.radians(gyro[0])
        imu.angular_velocity.y = math.radians(gyro[1])
        imu.angular_velocity.z = math.radians(gyro[2])
        for index, stddev in zip((0, 4, 8), self.gyro_stddev):
            imu.angular_velocity_covariance[index] = stddev**2 * warning_multiplier
        accel = finite_vector(msg.acceleration_g, 3)
        if validity.accel_valid and accel is not None:
            imu.linear_acceleration.x = accel[0] * GRAVITY_MPS2
            imu.linear_acceleration.y = accel[1] * GRAVITY_MPS2
            imu.linear_acceleration.z = accel[2] * GRAVITY_MPS2
            for index, stddev in zip((0, 4, 8), self.accel_stddev):
                imu.linear_acceleration_covariance[index] = stddev**2 * warning_multiplier
        else:
            imu.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(imu)

    @staticmethod
    def _time_message(seconds: float) -> TimeMessage:
        sec = math.floor(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec, nanosec = sec + 1, nanosec - 1_000_000_000
        return TimeMessage(sec=int(sec), nanosec=nanosec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
