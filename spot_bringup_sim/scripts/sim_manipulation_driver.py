#!/usr/bin/env python3
"""Simulation stand-in for spot_manipulation_driver's SpotManipulationDriverROS.

Exposes the same node name, topics, services, and action servers as the real
driver, but fulfills motion requests by streaming JointTrajectory setpoints to
the Gazebo JointTrajectoryController via the bridged /spot/joint_trajectory
topic (the same mechanism CHAMP uses for the legs).

Design notes:
  * Setpoints are streamed as single-point trajectories at STREAM_RATE_HZ,
    linearly interpolated in (sim) time along the goal trajectory. This makes
    the driver robust to CHAMP's continuous leg-only messages replacing the
    active gz trajectory: the gz controller retains per-joint targets for
    joints not named in a message, so arm targets persist between our updates.
  * Cartesian interfaces (~/cmd_vel, ~/solve_ik, ~/arm_cartesian_command) are
    stubbed pending an IK layer (v2). Hardware-only interfaces
    (image_to_grasp, mobile/body manipulation) are stubbed permanently.
"""

import math
import threading

import rclpy
import rclpy.callback_groups
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, TwistStamped, WrenchStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from spot_msgs.msg import ManipulatorCarryState, ManipulatorStowState
from spot_msgs.srv import GripperAngleMove, InverseKinematics
from spot_msgs.action import ArmCartesianCommand, ImageToGrasp

ARM_JOINT_ORDER = [
    "arm0_shoulder_yaw",
    "arm0_shoulder_pitch",
    "arm0_elbow_pitch",
    "arm0_elbow_roll",
    "arm0_wrist_pitch",
    "arm0_wrist_roll",
]
GRIPPER_JOINT = "arm0_fingers"
ALL_JOINTS = ARM_JOINT_ORDER + [GRIPPER_JOINT]

# Matches spot_bringup_sim/stow_arm.py (6 arm joints; gripper handled separately)
STOW_CONFIG = [-0.0011, -3.1360, 3.1337, 1.5699, -0.0027, -1.5684]

# TUNE: the real driver uses bosdyn's named ARM_READY command, so no explicit
# config exists in the source. Read /joint_states on the real robot after an
# unstow and replace these values for exact parity.
UNSTOW_CONFIG = [0.0, -0.9, 1.8, 0.0, -0.9, 0.0]

# Verbatim from spot_manipulation_driver.py MINI_UNSTOWN_ARM_CONFIG
MINI_UNSTOW_CONFIG = [
    -0.03200817108154297,
    -2.671809196472168,
    2.8863441944122314,
    0.019753217697143555,
    -0.23188066482543945,
    -0.019681930541992188,
]

# Sim gripper convention, matching the URDF limit [-1.57, 0.0].
# NOTE: intentionally different from the real driver's (closed=0.349066,
# opened=-1.396263) mapping, which exceeds the sim model's joint limits.
GRIPPER_CLOSED = 0.0
GRIPPER_OPEN = -1.5708

STREAM_RATE_HZ = 20.0        # setpoint streaming rate
SETPOINT_HORIZON_S = 0.1     # time_from_start for each streamed setpoint
NAMED_POSE_DURATION_S = 4.0  # duration for stow/unstow/gripper moves
GOAL_TOLERANCE_RAD = 0.05    # matches stow_arm.py
SETTLE_TIMEOUT_S = 3.0       # extra time allowed after trajectory end


class SimSpotManipulationDriverROS(Node):
    def __init__(self):
        super().__init__("spot_manipulation_driver")

        # Parameter parity with the real driver so shared launch args don't crash
        self.declare_parameter("hostname", "simulation")
        self.declare_parameter("rates.robot_state", 10.0)
        self.declare_parameter("action_namespace", "")
        self.declare_parameter("data_capture_mode", False)
        self.declare_parameter("publish_joint_states", False)

        self._joint_positions = {}
        self._joint_lock = threading.Lock()
        self._traj_pub = self.create_publisher(JointTrajectory, "/spot/joint_trajectory", 10)
        self.create_subscription(JointState, "/spot/joint_states", self._joint_state_cb, 10)

        # Cancel events, mirroring the real driver
        self._arm_cancel = threading.Event()
        self._finger_cancel = threading.Event()
        self._arm_and_finger_cancel = threading.Event()

        motion_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        gripper_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()

        # --- State publishers (same relative names as the real driver) ---
        self._arm_wrench_pub = self.create_publisher(WrenchStamped, "~/manipulator_state/wrench", 10)
        self._arm_vel_pub = self.create_publisher(TwistStamped, "~/manipulator_state/velocity", 10)
        self._carry_pub = self.create_publisher(ManipulatorCarryState, "~/manipulator_state/carry_state", 10)
        self._stow_pub = self.create_publisher(ManipulatorStowState, "~/manipulator_state/stow_state", 10)
        self._gripper_pub = self.create_publisher(Float32, "~/manipulator_state/gripper_open_percentage", 10)
        self._holding_pub = self.create_publisher(Bool, "~/manipulator_state/is_gripper_carrying_item", 10)
        self._collision_pub = self.create_publisher(Bool, "~/manipulator_state/is_hand_in_collision", 10)

        rate = self.get_parameter("rates.robot_state").value
        self.create_timer(1.0 / rate, self._state_timer_cb)

        # --- Command subscriptions (Cartesian: stubbed, v2 needs IK) ---
        self.create_subscription(Twist, "~/cmd_vel", self._ee_vel_cb, 10, callback_group=motion_group)
        self.create_subscription(TwistStamped, "/ee_twist_cmds", self._ap_ee_vel_cb, 10, callback_group=motion_group)

        # --- Services ---
        self.create_service(Trigger, "~/claim", self._trivial_success("Claimed (sim: no-op)"))
        self.create_service(Trigger, "~/release", self._trivial_success("Released (sim: no-op)"))
        self.create_service(Trigger, "~/power_on", self._trivial_success("Powered on (sim: no-op)"))
        self.create_service(Trigger, "~/stow_arm", self._stow_cb, callback_group=motion_group)
        self.create_service(Trigger, "~/unstow_arm", self._unstow_cb, callback_group=motion_group)
        self.create_service(Trigger, "~/mini_unstow_arm", self._mini_unstow_cb, callback_group=motion_group)
        self.create_service(Trigger, "~/open_gripper", self._open_gripper_cb, callback_group=gripper_group)
        self.create_service(Trigger, "~/close_gripper", self._close_gripper_cb, callback_group=gripper_group)
        self.create_service(GripperAngleMove, "~/set_gripper_angle", self._gripper_angle_cb, callback_group=gripper_group)
        self.create_service(InverseKinematics, "~/solve_ik", self._solve_ik_stub)

        # --- Action servers ---
        action_ns = self.get_parameter("action_namespace").value
        ActionServer(
            self, FollowJointTrajectory,
            f"{action_ns}/arm_controller/follow_joint_trajectory",
            self._arm_goal_cb, callback_group=motion_group,
            cancel_callback=self._make_cancel_cb(self._arm_cancel),
        )
        ActionServer(
            self, FollowJointTrajectory,
            f"{action_ns}/finger_controller/follow_joint_trajectory",
            self._finger_goal_cb, callback_group=gripper_group,
            cancel_callback=self._make_cancel_cb(self._finger_cancel),
        )
        ActionServer(
            self, FollowJointTrajectory,
            f"{action_ns}/arm_and_finger_controller/follow_joint_trajectory",
            self._arm_and_finger_goal_cb, callback_group=motion_group,
            cancel_callback=self._make_cancel_cb(self._arm_and_finger_cancel),
        )
        ActionServer(
            self, FollowJointTrajectory,
            f"{action_ns}/mobile_manipulation_controller/follow_joint_trajectory",
            self._unsupported_fjt_cb("mobile manipulation"), callback_group=motion_group,
        )
        ActionServer(
            self, FollowJointTrajectory,
            f"{action_ns}/body_manipulation_controller/follow_joint_trajectory",
            self._unsupported_fjt_cb("body manipulation"), callback_group=motion_group,
        )
        ActionServer(
            self, ImageToGrasp, "image_to_grasp",
            self._image_to_grasp_stub, callback_group=motion_group,
        )
        ActionServer(
            self, ArmCartesianCommand, "~/arm_cartesian_command",
            self._arm_cartesian_stub, callback_group=motion_group,
        )

        self.get_logger().info("Sim manipulation driver ready")

    # ------------------------------------------------------------------ #
    # State handling
    # ------------------------------------------------------------------ #
    def _joint_state_cb(self, msg: JointState):
        with self._joint_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ALL_JOINTS:
                    self._joint_positions[name] = pos

    def _get_positions(self, joint_names):
        with self._joint_lock:
            return [self._joint_positions.get(j) for j in joint_names]

    def _state_timer_cb(self):
        now = self.get_clock().now().to_msg()
        positions = self._get_positions(ALL_JOINTS)
        if any(p is None for p in positions):
            return  # joint states not yet received

        # Gripper open percentage from finger joint position
        finger = positions[-1]
        pct = (finger - GRIPPER_CLOSED) / (GRIPPER_OPEN - GRIPPER_CLOSED) * 100.0
        self._gripper_pub.publish(Float32(data=max(0.0, min(100.0, pct))))

        # Stow state from proximity to the stow configuration
        at_stow = all(
            abs(p - t) < 2 * GOAL_TOLERANCE_RAD
            for p, t in zip(positions[:6], STOW_CONFIG)
        )
        stow_state = (
            ManipulatorStowState.STOWSTATE_STOWED if at_stow
            else ManipulatorStowState.STOWSTATE_DEPLOYED
        )
        self._stow_pub.publish(ManipulatorStowState(state=stow_state))

        # Honest defaults for signals the sim cannot measure
        self._carry_pub.publish(ManipulatorCarryState())
        self._holding_pub.publish(Bool(data=False))
        self._collision_pub.publish(Bool(data=False))
        wrench = WrenchStamped()
        wrench.header.stamp = now
        wrench.header.frame_id = "arm0_hand"
        self._arm_wrench_pub.publish(wrench)
        twist = TwistStamped()
        twist.header.stamp = now
        twist.header.frame_id = "arm0_hand"
        self._arm_vel_pub.publish(twist)

    # ------------------------------------------------------------------ #
    # Core motion primitive: stream interpolated setpoints
    # ------------------------------------------------------------------ #
    def _execute_trajectory(self, joint_names, waypoints, cancel_event=None, feedback_cb=None):
        """Stream a joint trajectory to the gz JointTrajectoryController.

        joint_names: joints to command (subset of ALL_JOINTS)
        waypoints:   list of (positions, time_from_start_seconds), ascending
        Returns (success: bool, message: str).
        """
        unknown = [j for j in joint_names if j not in ALL_JOINTS]
        if unknown:
            return False, f"Unknown joints for sim arm: {unknown}"
        if not waypoints:
            return False, "Empty trajectory"

        start_positions = self._get_positions(joint_names)
        if any(p is None for p in start_positions):
            return False, "Joint states not yet available"

        # Prepend current state at t=0 so interpolation starts from reality
        if waypoints[0][1] > 1e-6:
            waypoints = [(start_positions, 0.0)] + list(waypoints)

        total_time = waypoints[-1][1]
        clock = self.get_clock()
        t_start = clock.now()
        period = Duration(seconds=1.0 / STREAM_RATE_HZ)

        n = len(joint_names)
        zero_vel = [0.0] * n

        def interpolate(t):
            """Return (position, velocity) at time t along the path.

            Velocity is the local segment slope (feed-forward), not zero,
            so successive streamed setpoints are velocity-continuous instead
            of each one asking the controller to brake to a stop.
            """
            if t >= total_time:
                return list(waypoints[-1][0]), zero_vel
            for i in range(len(waypoints) - 1):
                p0, t0 = waypoints[i]
                p1, t1 = waypoints[i + 1]
                if t0 <= t <= t1:
                    dt = t1 - t0
                    a = 0.0 if dt <= 0.0 else (t - t0) / dt
                    pos = [x0 + a * (x1 - x0) for x0, x1 in zip(p0, p1)]
                    vel = zero_vel if dt <= 0.0 else [
                        (x1 - x0) / dt for x0, x1 in zip(p0, p1)]
                    return pos, vel
            return list(waypoints[-1][0]), zero_vel

        def publish_setpoint(target, velocity=None):
            msg = JointTrajectory()
            msg.header.stamp = clock.now().to_msg()
            msg.joint_names = list(joint_names)
            pt = JointTrajectoryPoint()
            pt.positions = target
            pt.velocities = list(velocity) if velocity is not None else zero_vel
            pt.time_from_start = DurationMsg(
                sec=0, nanosec=int(SETPOINT_HORIZON_S * 1e9))
            msg.points = [pt]
            self._traj_pub.publish(msg)

        # Streaming phase
        while True:
            if cancel_event is not None and cancel_event.is_set():
                # Hold current position on cancel (zero velocity: come to rest)
                held = self._get_positions(joint_names)
                if all(p is not None for p in held):
                    publish_setpoint(held, zero_vel)
                return False, "Cancelled"
            elapsed = (clock.now() - t_start).nanoseconds / 1e9
            target, target_vel = interpolate(elapsed)
            # Command zero terminal velocity only once we've reached the end
            # of the path so the arm decelerates smoothly into the hold,
            # rather than every intermediate streamed point.
            if elapsed >= total_time:
                target_vel = zero_vel
            publish_setpoint(target, target_vel)
            if feedback_cb is not None:
                feedback_cb(joint_names, target, self._get_positions(joint_names))
            if elapsed >= total_time:
                break
            clock.sleep_for(period)

        # Settle phase: keep re-asserting the final target until within tolerance
        final = list(waypoints[-1][0])
        settle_deadline = clock.now() + Duration(seconds=SETTLE_TIMEOUT_S)
        while clock.now() < settle_deadline:
            if cancel_event is not None and cancel_event.is_set():
                return False, "Cancelled"
            actual = self._get_positions(joint_names)
            if all(abs(a - f) < GOAL_TOLERANCE_RAD for a, f in zip(actual, final)):
                return True, "Trajectory complete"
            publish_setpoint(final)
            clock.sleep_for(period)

        errors = [round(abs(a - f), 3) for a, f in zip(self._get_positions(joint_names), final)]
        return False, f"Goal tolerance violated after settle timeout; per-joint |error| = {errors}"

    def _move_to(self, joint_names, positions, duration_s=NAMED_POSE_DURATION_S):
        return self._execute_trajectory(joint_names, [(positions, duration_s)])

    # ------------------------------------------------------------------ #
    # Named-pose and gripper services
    # ------------------------------------------------------------------ #
    def _trivial_success(self, message):
        def cb(_, resp: Trigger.Response) -> Trigger.Response:
            resp.success = True
            resp.message = message
            return resp
        return cb

    def _stow_cb(self, _, resp):
        resp.success, resp.message = self._move_to(ARM_JOINT_ORDER, STOW_CONFIG)
        return resp

    def _unstow_cb(self, _, resp):
        resp.success, resp.message = self._move_to(ARM_JOINT_ORDER, UNSTOW_CONFIG)
        return resp

    def _mini_unstow_cb(self, _, resp):
        resp.success, resp.message = self._move_to(ARM_JOINT_ORDER, MINI_UNSTOW_CONFIG)
        return resp

    def _open_gripper_cb(self, _, resp):
        resp.success, resp.message = self._move_to([GRIPPER_JOINT], [GRIPPER_OPEN], 1.0)
        return resp

    def _close_gripper_cb(self, _, resp):
        resp.success, resp.message = self._move_to([GRIPPER_JOINT], [GRIPPER_CLOSED], 1.0)
        return resp

    def _gripper_angle_cb(self, req: GripperAngleMove.Request, resp: GripperAngleMove.Response):
        if req.gripper_angle > 90.0 or req.gripper_angle < 0.0:
            resp.success = False
            resp.message = "Could not set gripper angle to invalid angle"
            return resp
        target = req.gripper_angle / 90.0 * (GRIPPER_OPEN - GRIPPER_CLOSED) + GRIPPER_CLOSED
        resp.success, resp.message = self._move_to([GRIPPER_JOINT], [target], 1.0)
        return resp

    # ------------------------------------------------------------------ #
    # FollowJointTrajectory action servers
    # ------------------------------------------------------------------ #
    @staticmethod
    def _make_cancel_cb(event: threading.Event):
        def cb(_):
            event.set()
            return CancelResponse.ACCEPT
        return cb

    @staticmethod
    def _goal_to_waypoints(trajectory: JointTrajectory, joint_order):
        """Reorder incoming trajectory columns into joint_order; ignore other joints."""
        indices = []
        names = []
        for j in joint_order:
            if j in trajectory.joint_names:
                indices.append(trajectory.joint_names.index(j))
                names.append(j)
        waypoints = []
        for pt in trajectory.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            waypoints.append(([pt.positions[i] for i in indices], t))
        return names, waypoints

    def _run_fjt(self, goal_handle: ServerGoalHandle, joint_order, cancel_event):
        names, waypoints = self._goal_to_waypoints(goal_handle.request.trajectory, joint_order)
        if not names:
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=FollowJointTrajectory.Result.INVALID_JOINTS,
                error_string=f"No commandable joints in goal (expected from {joint_order})",
            )

        def feedback(joint_names, desired, actual):
            fb = FollowJointTrajectory.Feedback()
            fb.header.stamp = self.get_clock().now().to_msg()
            fb.joint_names = list(joint_names)
            fb.desired.positions = list(desired)
            if all(a is not None for a in actual):
                fb.actual.positions = list(actual)
                fb.error.positions = [d - a for d, a in zip(desired, actual)]
            goal_handle.publish_feedback(fb)

        cancel_event.clear()
        success, message = self._execute_trajectory(names, waypoints, cancel_event, feedback)

        result = FollowJointTrajectory.Result()
        result.error_string = message
        if cancel_event.is_set():
            goal_handle.canceled()
            cancel_event.clear()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        elif success:
            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        else:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
        self.get_logger().info(f"FollowJointTrajectory finished: {message}")
        return result

    def _arm_goal_cb(self, goal_handle):
        return self._run_fjt(goal_handle, ARM_JOINT_ORDER, self._arm_cancel)

    def _finger_goal_cb(self, goal_handle):
        return self._run_fjt(goal_handle, [GRIPPER_JOINT], self._finger_cancel)

    def _arm_and_finger_goal_cb(self, goal_handle):
        return self._run_fjt(goal_handle, ARM_JOINT_ORDER + [GRIPPER_JOINT], self._arm_and_finger_cancel)

    # ------------------------------------------------------------------ #
    # Stubs: Cartesian (v2, needs IK) and hardware-only interfaces
    # ------------------------------------------------------------------ #
    def _ee_vel_cb(self, msg: Twist):
        self.get_logger().warn(
            "Cartesian EE velocity (~/cmd_vel) not implemented in sim driver yet (needs IK layer)",
            throttle_duration_sec=5.0,
        )

    def _ap_ee_vel_cb(self, msg: TwistStamped):
        self._ee_vel_cb(msg.twist)

    def _solve_ik_stub(self, req, resp: InverseKinematics.Response):
        self.get_logger().warn("~/solve_ik not implemented in sim driver (needs IK layer)")
        resp.solution_found = False
        return resp

    def _arm_cartesian_stub(self, goal_handle) -> ArmCartesianCommand.Result:
        goal_handle.abort()
        return ArmCartesianCommand.Result(
            success=False, message="arm_cartesian_command not implemented in sim driver (needs IK layer)")

    def _image_to_grasp_stub(self, goal_handle) -> ImageToGrasp.Result:
        goal_handle.abort()
        result = ImageToGrasp.Result()
        result.success = False
        return result

    def _unsupported_fjt_cb(self, name):
        def cb(goal_handle):
            goal_handle.abort()
            return FollowJointTrajectory.Result(
                error_code=-1,
                error_string=f"{name} is not supported in simulation (CHAMP owns the body)",
            )
        return cb


def main():
    rclpy.init()
    node = SimSpotManipulationDriverROS()
    node.set_parameters([rclpy.parameter.Parameter(
        "use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()