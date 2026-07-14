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
  * Motion is PACED against the wall clock (time.monotonic/time.sleep), not the
    sim clock. A named-pose Trigger handler therefore blocks for a bounded
    amount of *real* time (<= NAMED_POSE_DURATION_S + SETTLE_TIMEOUT_S), so it
    can never outlast a behavior-tree client's wall-clock service timeout, no
    matter how low Gazebo's real-time factor is. (The previous version paced on
    the sim clock via clock.sleep_for(); at RTF < ~0.4 a 1 s move + 3 s settle
    exceeded the tree's 10 s TriggerService deadline and surfaced as
    "Service timeout. Failed to trigger service ...". Message stamps still use
    sim time.) This mirrors the wall-clock loop in sim_spot_driver.py.
  * The named-pose Trigger services (stow/unstow/mini_unstow + gripper) can
    optionally return BEFORE the motion finishes -- set the
    'named_pose_return_immediately' parameter True. The move is then dispatched
    on a background thread and the service replies instantly; the gz controller
    holds the streamed targets so the arm keeps moving. Default is False
    (block and report the real result), which is now safe under the client
    timeout thanks to wall-clock pacing.
  * ~/cmd_vel and pose-only ~/arm_cartesian_command goals are stubbed pending
    an IK layer (v2); ~/arm_cartesian_command goals carrying joint_waypoints
    (the stable_arm_motion_server path) are executed. ~/solve_ik forwards to
    MoveIt. Hardware-only interfaces (image_to_grasp, mobile/body
    manipulation) are stubbed permanently.
"""

import math
import threading
import time

import rclpy
import rclpy.callback_groups
from rclpy.action import ActionServer, CancelResponse
from rclpy.action.server import ServerGoalHandle
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

# For the real IK adapter (forwards to MoveIt's /spot_moveit/compute_ik)
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

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
NAMED_POSE_DURATION_S = 1.0  # duration for stow/unstow/gripper moves
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
        # When True, named-pose Trigger services reply as soon as the motion is
        # dispatched (background thread) instead of blocking until it completes.
        self.declare_parameter("named_pose_return_immediately", False)

        self._joint_positions = {}
        self._joint_lock = threading.Lock()
        self._traj_pub = self.create_publisher(JointTrajectory, "/spot/joint_trajectory", 10)
        self.create_subscription(JointState, "/spot_driver/joint_states", self._joint_state_cb, 10)

        # Cancel events, mirroring the real driver
        self._arm_cancel = threading.Event()
        self._arm_cartesian_cancel = threading.Event()
        self._finger_cancel = threading.Event()
        self._arm_and_finger_cancel = threading.Event()

        # Serialization + cancellation for async (background-thread) named poses.
        # Arm and gripper are independent channels so an async gripper move never
        # cancels an in-flight async arm move (they command disjoint joints).
        self._named_arm_lock = threading.Lock()
        self._named_arm_cancel = threading.Event()
        self._named_finger_lock = threading.Lock()
        self._named_finger_cancel = threading.Event()

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

        # solve_ik forwards to MoveIt's /spot_moveit/compute_ik (real IK) and
        # repacks the answer into spot_msgs/srv/InverseKinematics. Its own group
        # (reentrant) lets the handler block on the compute_ik client without
        # stalling the motion group.
        self.declare_parameter("ik_group_name", "arm")
        self.declare_parameter("ik_service", "/spot_moveit/compute_ik")
        self.declare_parameter("ik_avoid_collisions", True)
        ik_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self._ik_client = self.create_client(
            GetPositionIK, self.get_parameter("ik_service").value,
            callback_group=ik_group)
        self.create_service(
            InverseKinematics, "~/solve_ik", self._solve_ik_cb, callback_group=ik_group)

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
            self._arm_cartesian_cb, callback_group=motion_group,
            cancel_callback=self._make_cancel_cb(self._arm_cartesian_cancel),
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

        Timing note: pacing (elapsed time, sleeps, settle deadline) uses the
        WALL clock (time.monotonic/time.sleep) so the call always returns within
        a bounded amount of real time regardless of Gazebo's real-time factor.
        Only message stamps use the sim clock. See the module docstring.
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
        clock = self.get_clock()          # sim clock: used only for msg stamps
        t_start = time.monotonic()        # wall clock: used for all pacing
        period_s = 1.0 / STREAM_RATE_HZ

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
            msg.header.stamp = clock.now().to_msg()   # sim-time stamp preserved
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
            elapsed = time.monotonic() - t_start
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
            time.sleep(period_s)

        # Settle phase: keep re-asserting the final target until within tolerance
        final = list(waypoints[-1][0])
        settle_deadline = time.monotonic() + SETTLE_TIMEOUT_S
        while time.monotonic() < settle_deadline:
            if cancel_event is not None and cancel_event.is_set():
                return False, "Cancelled"
            actual = self._get_positions(joint_names)
            if all(abs(a - f) < GOAL_TOLERANCE_RAD for a, f in zip(actual, final)):
                return True, "Trajectory complete"
            publish_setpoint(final)
            time.sleep(period_s)

        errors = [round(abs(a - f), 3) for a, f in zip(self._get_positions(joint_names), final)]
        return False, f"Goal tolerance violated after settle timeout; per-joint |error| = {errors}"

    def _perform_named_move(self, joint_names, positions, duration_s, lock, cancel):
        """Run a named-pose / gripper move for a Trigger-style service.

        Blocking mode (default): execute the (wall-clock-bounded) move inline and
        return the real (success, message). Safe under the behavior tree's
        wall-clock service timeout now that _execute_trajectory paces in real
        time (<= duration_s + SETTLE_TIMEOUT_S seconds).

        Immediate mode ('named_pose_return_immediately' == True): dispatch the
        move on a background thread and return success right away. `lock`
        serializes moves on the same channel and `cancel` preempts an in-flight
        move on that channel, so a new request supersedes an old one instead of
        piling up. The gz controller holds the streamed targets, so the arm
        keeps moving after the service has already replied.
        """
        if not self.get_parameter("named_pose_return_immediately").value:
            return self._execute_trajectory(joint_names, [(positions, duration_s)])

        # Ask any in-flight move on this channel to stop, then launch a fresh one.
        cancel.set()

        def worker():
            with lock:                 # serialize; wait for the prior move to exit
                cancel.clear()
                _, message = self._execute_trajectory(
                    joint_names, [(positions, duration_s)], cancel_event=cancel)
                self.get_logger().info(f"Named pose finished (async): {message}")

        threading.Thread(target=worker, daemon=True).start()
        return True, "Motion dispatched (sim: returning before completion)"

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
        resp.success, resp.message = self._perform_named_move(
            ARM_JOINT_ORDER, STOW_CONFIG, NAMED_POSE_DURATION_S,
            self._named_arm_lock, self._named_arm_cancel)
        return resp

    def _unstow_cb(self, _, resp):
        resp.success, resp.message = self._perform_named_move(
            ARM_JOINT_ORDER, UNSTOW_CONFIG, NAMED_POSE_DURATION_S,
            self._named_arm_lock, self._named_arm_cancel)
        return resp

    def _mini_unstow_cb(self, _, resp):
        resp.success, resp.message = self._perform_named_move(
            ARM_JOINT_ORDER, MINI_UNSTOW_CONFIG, NAMED_POSE_DURATION_S,
            self._named_arm_lock, self._named_arm_cancel)
        return resp

    def _open_gripper_cb(self, _, resp):
        resp.success, resp.message = self._perform_named_move(
            [GRIPPER_JOINT], [GRIPPER_OPEN], 1.0,
            self._named_finger_lock, self._named_finger_cancel)
        return resp

    def _close_gripper_cb(self, _, resp):
        resp.success, resp.message = self._perform_named_move(
            [GRIPPER_JOINT], [GRIPPER_CLOSED], 1.0,
            self._named_finger_lock, self._named_finger_cancel)
        return resp

    def _gripper_angle_cb(self, req: GripperAngleMove.Request, resp: GripperAngleMove.Response):
        if req.gripper_angle > 90.0 or req.gripper_angle < 0.0:
            resp.success = False
            resp.message = "Could not set gripper angle to invalid angle"
            return resp
        target = req.gripper_angle / 90.0 * (GRIPPER_OPEN - GRIPPER_CLOSED) + GRIPPER_CLOSED
        resp.success, resp.message = self._perform_named_move(
            [GRIPPER_JOINT], [target], 1.0,
            self._named_finger_lock, self._named_finger_cancel)
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

    def _call_sync(self, client, request, timeout, what):
        """Blocking service call that is safe under a MultiThreadedExecutor.

        We never spin here (the executor spins in other threads); we wait on the
        future via a done-callback + Event so another thread delivers the reply.
        """
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(f"{what}: service '{client.srv_name}' not available")
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(timeout):
            self.get_logger().warn(f"{what}: timed out waiting for response")
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().warn(f"{what}: call raised {exc!r}")
            return None

    def _solve_ik_cb(self, req: InverseKinematics.Request, resp: InverseKinematics.Response):
        """Forward to MoveIt /spot_moveit/compute_ik and repack the result.

        Limitations vs. the real Boston Dynamics IK:
          * body_pose is returned as identity -- the MoveIt 'arm' group is
            fixed-base, so no whole-body redistribution is computed.
          * gaze_target requests are not supported; we solve the 6-DoF pose IK
            and warn. Wire a gaze constraint here if you need it later.
        """
        timeout = float(getattr(req, "timeout", 1.0)) if hasattr(req, "timeout") else 1.0
        if timeout <= 0.0:
            timeout = 1.0

        if getattr(req, "use_gaze_target", False):
            self.get_logger().warn(
                "solve_ik: gaze_target not supported in sim adapter; solving pose IK only")

        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = self.get_parameter("ik_group_name").value
        ik_req.ik_request.avoid_collisions = bool(
            self.get_parameter("ik_avoid_collisions").value)
        ik_req.ik_request.ik_link_name = req.tool_frame or "arm0_hand"
        ik_req.ik_request.pose_stamped = req.target_pose

        # Seed: use the provided nominal joints as a diff over the current state,
        # so partial seeds are valid and an empty seed means "use current state".
        ik_req.ik_request.robot_state.is_diff = True
        if req.joint_names:
            ik_req.ik_request.robot_state.joint_state.name = list(req.joint_names)
            ik_req.ik_request.robot_state.joint_state.position = list(
                req.joint_nominal_positions)

        ik_req.ik_request.timeout = DurationMsg(
            sec=int(timeout), nanosec=int((timeout % 1.0) * 1e9))

        result = self._call_sync(self._ik_client, ik_req, timeout + 1.0, "compute_ik")
        if result is None:
            resp.solution_found = False
            return resp

        ok = (result.error_code.val == MoveItErrorCodes.SUCCESS)
        resp.solution_found = bool(ok)
        if not ok:
            self.get_logger().warn(
                f"solve_ik: MoveIt returned no solution (error_code={result.error_code.val})")
            return resp

        # Extract just the arm joints (in canonical order) for arm_joint_state.
        sol = result.solution.joint_state
        name_to_pos = dict(zip(sol.name, sol.position))
        arm_state = JointState()
        arm_state.header.stamp = self.get_clock().now().to_msg()
        for j in ARM_JOINT_ORDER:
            if j in name_to_pos:
                arm_state.name.append(j)
                arm_state.position.append(name_to_pos[j])
        resp.arm_joint_state = arm_state

        # Fixed-base group: body does not move, report identity.
        resp.body_pose.orientation.w = 1.0
        return resp
    
    def _arm_cartesian_cb(self, goal_handle) -> ArmCartesianCommand.Result:
        """Honor ArmCartesianCommand when the caller supplies joint_waypoints.

        stable_arm_motion_server's populateKnownTrajectory() packs the already
        solved JointTrajectory into goal.joint_waypoints alongside the Cartesian
        poses (see ArmCartesianCommand.action:15-17), so for that path no IK is
        needed here -- we track the joint solution MoveIt already produced, and
        goal.waypoints/timestamps are advisory. Pose-only callers (e.g.
        move_hand_to_pose.cpp) still need an IK layer and are rejected loudly
        rather than silently approximated.
        """
        goal = goal_handle.request

        if not goal.joint_waypoints.points:
            goal_handle.abort()
            return ArmCartesianCommand.Result(
                success=False,
                message="pose-only arm_cartesian_command not implemented in sim "
                        "driver (needs IK layer)")

        names, waypoints = self._goal_to_waypoints(goal.joint_waypoints, ARM_JOINT_ORDER)
        if not names:
            goal_handle.abort()
            return ArmCartesianCommand.Result(
                success=False,
                message=f"No commandable arm joints in joint_waypoints "
                        f"(expected from {ARM_JOINT_ORDER})")

        def feedback(joint_names, desired, actual):
            fb = ArmCartesianCommand.Feedback()
            fb.status = ArmCartesianCommand.Feedback.STATUS_IN_PROGRESS
            goal_handle.publish_feedback(fb)

        self._arm_cartesian_cancel.clear()
        success, message = self._execute_trajectory(
            names, waypoints, self._arm_cartesian_cancel, feedback)

        if self._arm_cartesian_cancel.is_set():
            self._arm_cartesian_cancel.clear()
            goal_handle.canceled()
            return ArmCartesianCommand.Result(
                success=False, message="Trajectory cancelled")

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(f"ArmCartesianCommand finished: {message}")
        return ArmCartesianCommand.Result(success=success, message=message)

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