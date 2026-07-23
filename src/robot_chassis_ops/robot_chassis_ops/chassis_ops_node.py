#!/usr/bin/env python3
"""Expose chassis operations without claiming queue admission means applied."""

import asyncio
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from robot_interfaces.action import ChassisOperation
from robot_interfaces.msg import FirmwareControlState, FirmwareInfo
from std_srvs.srv import SetBool, Trigger


class ChassisOpsNode(Node):
    def __init__(self) -> None:
        super().__init__("chassis_ops")
        self.state = None
        self.info_generation = 0
        self.estop = self.create_client(SetBool, "chassis/estop")
        self.clear = self.create_client(Trigger, "chassis/clear_fault")
        self.line = self.create_client(SetBool, "chassis/line_ctrl")
        self.info = self.create_client(Trigger, "chassis/get_info")
        self.create_subscription(
            FirmwareControlState, "chassis/firmware_control_state", self._state, 10
        )
        self.create_subscription(FirmwareInfo, "chassis/firmware_info", self._info, 10)
        self.server = ActionServer(
            self,
            ChassisOperation,
            "chassis/operation",
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )

    def _state(self, msg):
        self.state = msg

    def _info(self, _msg):
        self.info_generation += 1

    def _goal(self, request):
        valid = request.operation in (
            ChassisOperation.Goal.OP_ESTOP_SET,
            ChassisOperation.Goal.OP_CLEAR_FAULT,
            ChassisOperation.Goal.OP_LINE_CONTROL,
            ChassisOperation.Goal.OP_GET_INFO,
        )
        if request.operation == ChassisOperation.Goal.OP_ESTOP_SET and not request.enable:
            valid = False
        return GoalResponse.ACCEPT if valid else GoalResponse.REJECT

    def _cancel(self, _goal):
        return CancelResponse.ACCEPT

    async def _execute(self, goal):
        op = goal.request.operation
        if op == ChassisOperation.Goal.OP_CLEAR_FAULT and self.state and self.state.estop_active:
            return self._finish(
                goal, ChassisOperation.Result.RESULT_LOCAL_REJECTED, "ESTOP is active"
            )
        before_info = self.info_generation
        before_status_sequence = None if self.state is None else int(self.state.status_sequence)
        if op == ChassisOperation.Goal.OP_ESTOP_SET:
            future = self.estop.call_async(SetBool.Request(data=True))
        elif op == ChassisOperation.Goal.OP_CLEAR_FAULT:
            future = self.clear.call_async(Trigger.Request())
        elif op == ChassisOperation.Goal.OP_LINE_CONTROL:
            future = self.line.call_async(SetBool.Request(data=goal.request.enable))
        else:
            future = self.info.call_async(Trigger.Request())
        deadline = time.monotonic() + 1.0
        while not future.done() and time.monotonic() < deadline:
            if goal.is_cancel_requested:
                goal.canceled()
                return self._result(ChassisOperation.Result.RESULT_CANCELLED, "cancelled")
            await asyncio.sleep(0.01)
        if not future.done() or future.result() is None or not future.result().success:
            return self._finish(
                goal, ChassisOperation.Result.RESULT_TIMEOUT, "request was not queued"
            )
        while time.monotonic() < deadline:
            status_is_new = self.state is not None and (
                before_status_sequence is None
                or int(self.state.status_sequence) != before_status_sequence
            )
            if (
                op == ChassisOperation.Goal.OP_ESTOP_SET
                and status_is_new
                and self.state.estop_active
            ):
                return self._finish(goal, ChassisOperation.Result.RESULT_APPLIED, "ESTOP observed")
            if (
                op == ChassisOperation.Goal.OP_CLEAR_FAULT
                and status_is_new
                and not self.state.fault_stop_active
            ):
                return self._finish(
                    goal, ChassisOperation.Result.RESULT_APPLIED, "fault clear observed"
                )
            if (
                op == ChassisOperation.Goal.OP_LINE_CONTROL
                and status_is_new
                and bool(self.state.status_flags & 4) == goal.request.enable
            ):
                return self._finish(
                    goal, ChassisOperation.Result.RESULT_APPLIED, "line state observed"
                )
            if op == ChassisOperation.Goal.OP_GET_INFO and self.info_generation > before_info:
                return self._finish(
                    goal, ChassisOperation.Result.RESULT_APPLIED, "new HELLO observed"
                )
            await asyncio.sleep(0.01)
        status = (
            ChassisOperation.Result.RESULT_CONDITION_NOT_CLEARED
            if op == ChassisOperation.Goal.OP_CLEAR_FAULT
            else ChassisOperation.Result.RESULT_TIMEOUT
        )
        return self._finish(goal, status, "no confirming firmware evidence")

    def _finish(self, goal, status, message):
        goal.succeed()
        return self._result(status, message)

    @staticmethod
    def _result(status, message):
        result = ChassisOperation.Result()
        result.status = status
        result.message = message
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ChassisOpsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
