#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# sim_spot_driver.py
#
# A simulation stand-in ("mock" / shim) for the Boston Dynamics spot_driver
# node from the spot_ros stack.
#
# The real spot_driver (spot_driver/spot_driver/spot_ros.py) only exists to
# bridge ROS 2 to a PHYSICAL Spot over Boston Dynamics' gRPC API. It cannot
# talk to a Gazebo/CHAMP model, so in simulation none of its services or
# action servers exist -- and any node that calls e.g. /spot_driver/toggle_payload
# times out and aborts:
#
#   [ERROR] ... Unable to contact client [/spot_driver/toggle_payload]
#           within 5.00 seconds. Aborting
#
# This node impersonates the driver's ROS 2 interface: same node name
# ('spot_driver'), same service names/types, same action servers. Higher level
# code (spot_behaviors, teleop, etc.) can call /spot_driver/* exactly as it
# would against real hardware and get immediate, successful responses, while
# CHAMP/Gazebo handles the actual locomotion underneath.
#
# The body services carry the real robot's semantics, because code that drives
# them (e-stop, power, stand) must behave in sim as it will on the robot:
#
#   - Velocity goes through the driver, as on hardware: ~/cmd_vel is forwarded
#     to CHAMP (~/sim/cmd_vel) only while the robot is powered, standing, not
#     e-stopped and not frozen, and each command lapses after `cmd_duration`
#     s, as the real driver's velocity commands do (end_time).
#   - estop/gentle settles the robot (it crouches over `settle_s`) and then
#     cuts motor power;
#     estop/hard cuts at once; estop/release clears the e-stop and leaves the
#     motors off. power_on is refused while e-stopped; stand and sit are
#     refused without motor power, and stand raises the body over `stand_s` s
#     after it is accepted, reporting standing once it is up -- the real stand
#     command is non-blocking, so a client must wait for ~/status/feedback to
#     report standing. The body's height moves continuously both ways: a step
#     in CHAMP's body pose throws the body up and can topple it.
#   - ~/status/power_state, ~/status/estop and ~/status/feedback are published
#     with the real driver's message types and topic names.
#
#   Not modelled: the lease (claim is not required first), the estop
#   keepalive, and the physics of settling and standing beyond a crouch.
#
# The action servers (navigate_to, walk_to) succeed IMMEDIATELY: locomotion is
# owned by the navigation stack / CHAMP, so all this shim has to do is report a
# clean success back to the behavior tree. Every terminal path calls
# goal_handle.succeed() and every feedback field is guarded (see
# _safe_publish_feedback) so a spot_msgs schema mismatch can never raise inside
# an execute callback -- an unhandled exception there would leave the goal stuck
# in EXECUTING (rclpy does NOT abort on it), and a polling client such as
# spot_behaviors' WalkToPose would then hang forever on STATUS_EXECUTING.
#
# Usage:
#   ros2 run <your_pkg> sim_spot_driver.py
#   # or directly:
#   python3 sim_spot_driver.py
#   # optionally namespace it to match your setup:
#   ros2 run <your_pkg> sim_spot_driver.py --ros-args -r __ns:=/
# ---------------------------------------------------------------------------

import time

import rclpy
import rclpy.action
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Pose, Twist
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import Trigger, SetBool

from spot_msgs.srv import (
    Dock,
    ClearBehaviorFault,
    ListGraph,
    SetLocomotion,
    SetVelocity,
    GestureSequence,
    TogglePayload,
    RegisterPayload,
)
from spot_msgs.action import NavigateTo, WalkTo
from spot_msgs.msg import (BatteryStateArray, BatteryState, EStopState, EStopStateArray,
                           Feedback, PowerState)

ACTION_SETTLE_S = 1.0

class SimSpotDriver(Node):
    """Simulation stand-in for the real spot_driver node.

    Advertises the same services and action servers as spot_ros.py, returns
    success for everything, and tracks a minimal amount of self-consistent
    state so behaviors that read it back stay happy.
    """

    def __init__(self):
        super().__init__('spot_driver')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)

        cb = ReentrantCallbackGroup()

        # ---- Body state, as the real robot reports it ---------------------
        # Starts powered and standing so a plain sim launch walks at once (as
        # it always has); `start_standing: false` starts as a robot boots,
        # motors off and lying down.
        self.declare_parameter('start_standing', True)
        # How long a velocity command lasts (the real driver's cmd_duration).
        self.declare_parameter('cmd_duration', 0.125)
        # Sim seconds from an accepted stand to standing, and from a gentle
        # e-stop to motor cut (the robot settles first).
        self.declare_parameter('stand_s', 1.5)
        self.declare_parameter('settle_s', 1.5)
        # How far below nominal height CHAMP holds the body while sitting.
        self.declare_parameter('sit_drop', 0.25)
        up = bool(self.get_parameter('start_standing').value)
        self._claimed = False
        self._powered_on = up
        self._standing = up
        self._estopped = False
        self._frozen = False
        self._pending = None          # (deadline_s, action) for stand / motor cut
        self._last_cmd = None         # (stamp_s, Twist)
        self._body_pose = Pose()      # last ~/body_pose request
        self._drop = 0.0 if up else float(self.get_parameter('sit_drop').value)
        self._last_tick = None        # body height is ramped between ticks
        self._docked = False
        self._stair_mode = False
        self._locomotion_mode = 1        # HINT_AUTO
        self._payloads = {}              # guid/name -> attached (bool)

        # ---- Simulated battery ---------------------------------------------
        self.declare_parameter('battery_percentage', 95.0)

        # ================= std_srvs/Trigger services =======================
        self.create_service(Trigger, '~/claim',       self._h_claim,       callback_group=cb)
        self.create_service(Trigger, '~/release',     self._h_release,     callback_group=cb)
        self.create_service(Trigger, '~/force_claim', self._h_force_claim, callback_group=cb)
        self.create_service(Trigger, '~/stop',        self._h_stop,        callback_group=cb)
        self.create_service(Trigger, '~/self_right',  self._h_self_right,  callback_group=cb)
        self.create_service(Trigger, '~/sit',         self._h_sit,         callback_group=cb)
        self.create_service(Trigger, '~/stand',       self._h_stand,       callback_group=cb)
        self.create_service(Trigger, '~/power_on',    self._h_power_on,    callback_group=cb)
        self.create_service(Trigger, '~/power_off',   self._h_power_off,   callback_group=cb)

        # ---- E-stop (Trigger) ---------------------------------------------
        self.create_service(Trigger, '~/estop/freeze',   self._h_estop_freeze,   callback_group=cb)
        self.create_service(Trigger, '~/estop/unfreeze', self._h_estop_unfreeze, callback_group=cb)
        self.create_service(Trigger, '~/estop/hard',     self._h_estop_hard,     callback_group=cb)
        self.create_service(Trigger, '~/estop/gentle',   self._h_estop_gentle,   callback_group=cb)
        self.create_service(Trigger, '~/estop/release',  self._h_estop_release,  callback_group=cb)

        # ---- Docking (Trigger half) ---------------------------------------
        self.create_service(Trigger, '~/undock', self._h_undock, callback_group=cb)

        # ================= std_srvs/SetBool services =======================
        self.create_service(SetBool, '~/stair_mode', self._h_stair_mode, callback_group=cb)

        # ================= spot_msgs/srv services ==========================
        self.create_service(SetLocomotion,      '~/locomotion_mode',      self._h_locomotion_mode,      callback_group=cb)
        self.create_service(SetVelocity,        '~/max_velocity',         self._h_max_velocity,         callback_group=cb)
        self.create_service(ClearBehaviorFault, '~/clear_behavior_fault', self._h_clear_behavior_fault, callback_group=cb)
        self.create_service(TogglePayload,      '~/toggle_payload',       self._h_toggle_payload,       callback_group=cb)
        self.create_service(RegisterPayload,    '~/register_payload',     self._h_register_payload,     callback_group=cb)
        self.create_service(ListGraph,          '~/list_graph',           self._h_list_graph,           callback_group=cb)
        self.create_service(Dock,               '~/dock',                 self._h_dock,                 callback_group=cb)
        self.create_service(GestureSequence,    '~/gesture_sequence',     self._h_gesture_sequence,     callback_group=cb)

        # ================= Action servers ==================================
        self._navigate_to_server = ActionServer(
            self, NavigateTo, '~/navigate_to',
            execute_callback=self._h_navigate_to,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=cb,
        )
        self._walk_to_server = ActionServer(
            self, WalkTo, '~/walk_to',
            execute_callback=self._h_walk_to,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=cb,
        )

        # ================= Status publishers ===============================
        self._battery_pub = self.create_publisher(
            BatteryStateArray, '~/status/battery_states', qos_profile_sensor_data)
        self.create_timer(0.5, self._publish_battery, callback_group=cb)
        # Same names, types and depths as spot_ros.py.
        self._power_pub = self.create_publisher(PowerState, '~/status/power_state', 1)
        self._estop_pub = self.create_publisher(EStopStateArray, '~/status/estop', 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._feedback_pub = self.create_publisher(Feedback, '~/status/feedback', latched)

        # ================= Locomotion ======================================
        # Commands in on the real driver's topics; out to CHAMP only as the
        # robot's state allows.
        self.create_subscription(Twist, '~/cmd_vel', self._on_cmd_vel,
                                 qos_profile_sensor_data, callback_group=cb)
        self.create_subscription(Pose, '~/body_pose', self._on_body_pose,
                                 qos_profile_sensor_data, callback_group=cb)
        self._gait_vel_pub = self.create_publisher(Twist, '~/sim/cmd_vel', 10)
        self._gait_pose_pub = self.create_publisher(Pose, '~/sim/body_pose', 1)
        self.create_timer(0.05, self._tick, callback_group=cb)
        self._publish_state()

        self.get_logger().info(
            'sim_spot_driver up: emulating all /spot_driver services + action servers '
            '(NOT connected to real hardware).'
        )

    # ----------------------------------------------------------------------
    # Simulated battery state (matches spot_driver/status/battery_states)
    # ----------------------------------------------------------------------
    def _publish_battery(self):
        # The real driver refreshes power and e-stop status continually; so do we.
        self._publish_state()
        pct = float(self.get_parameter('battery_percentage').value)
        stamp = self.get_clock().now().to_msg()
        battery = BatteryState()
        battery.charge_percentage = pct

        if hasattr(battery, 'identifier'):
            battery.identifier = 'sim_battery_0'
        if hasattr(battery, 'timestamp'):
            battery.timestamp = stamp
        if hasattr(battery, 'voltage'):
            battery.voltage = 58.0
        if hasattr(battery, 'current'):
            battery.current = -5.0
        if hasattr(battery, 'temperatures'):
            battery.temperatures = [30.0]
        if hasattr(battery, 'status'):
            battery.status = getattr(BatteryState, 'STATUS_DISCHARGING', 0)

        msg = BatteryStateArray()
        # Header is on the array in this schema; guard it too.
        if hasattr(msg, 'header'):
            msg.header.stamp = stamp
        msg.battery_states = [battery]
        self._battery_pub.publish(msg)

    # ----------------------------------------------------------------------
    # Body state: what the robot will do, and what it reports
    # ----------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mobile(self) -> bool:
        return self._powered_on and self._standing and not self._estopped and not self._frozen

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = (self._now(), msg)

    def _on_body_pose(self, msg: Pose):
        self._body_pose = msg

    def _tick(self):
        """Advance pending transitions and drive CHAMP from the body state."""
        now = self._now()
        # A stand completes once the body is up, not merely when its time is.
        due = self._pending is not None and now >= self._pending[0]
        if due and self._pending[1] == 'stand' and self._drop > 0.0:
            due = False
        if due:
            _, action = self._pending
            self._pending = None
            if action == 'stand' and self._powered_on and not self._estopped:
                self._standing = True
            elif action == 'cut':
                self._powered_on = False
            self._publish_state()

        cmd = Twist()
        lapse = float(self.get_parameter('cmd_duration').value)
        if self._mobile() and self._last_cmd is not None and now - self._last_cmd[0] <= lapse:
            cmd = self._last_cmd[1]
        self._gait_vel_pub.publish(cmd)

        # Lowered while not standing, raised during a stand, at the rate that
        # covers the full drop in settle_s / stand_s.
        sit_drop = float(self.get_parameter('sit_drop').value)
        dt = 0.0 if self._last_tick is None else max(0.0, now - self._last_tick)
        self._last_tick = now
        rising = self._standing or (self._pending is not None and self._pending[1] == 'stand')
        if rising:
            step = sit_drop / max(float(self.get_parameter('stand_s').value), 1e-3) * dt
            self._drop = max(0.0, self._drop - step)
        else:
            step = sit_drop / max(float(self.get_parameter('settle_s').value), 1e-3) * dt
            self._drop = min(sit_drop, self._drop + step)
        pose = Pose()
        if self._standing and self._drop == 0.0:
            pose = self._body_pose
        else:
            pose.position.z = self._body_pose.position.z - self._drop
        pose.orientation.w = pose.orientation.w or 1.0
        self._gait_pose_pub.publish(pose)

    def _publish_state(self):
        stamp = self.get_clock().now().to_msg()
        power = PowerState()
        power.stamp = stamp
        power.motor_power_state = PowerState.STATE_ON if self._powered_on else PowerState.STATE_OFF
        power.shore_power_state = PowerState.STATE_OFF_SHORE_POWER
        power.locomotion_charge_percentage = float(self.get_parameter('battery_percentage').value)
        self._power_pub.publish(power)

        estop = EStopState()
        estop.header.stamp = stamp
        estop.name = 'software_estop'
        estop.type = EStopState.TYPE_SOFTWARE
        estop.state = EStopState.STATE_ESTOPPED if self._estopped else EStopState.STATE_NOT_ESTOPPED
        self._estop_pub.publish(EStopStateArray(estop_states=[estop]))

        fb = Feedback()
        fb.standing = self._standing
        fb.sitting = not self._standing
        fb.moving = False
        fb.species = 'spot'
        fb.nickname = 'sim'
        self._feedback_pub.publish(fb)

    def _fail(self, response, msg):
        self.get_logger().warn(f'[sim] {msg}')
        response.success = False
        response.message = msg
        return response

    # ----------------------------------------------------------------------
    # Small helper to fill and log a Trigger-style (success, message) response
    # ----------------------------------------------------------------------
    def _ok(self, response, msg):
        self.get_logger().info(f'[sim] {msg}')
        response.success = True
        response.message = msg
        return response

    # ===================== Trigger handlers ===============================
    def _h_claim(self, req, resp):
        self._claimed = True
        return self._ok(resp, 'claim: lease acquired (simulated)')

    def _h_release(self, req, resp):
        self._claimed = False
        return self._ok(resp, 'release: lease released (simulated)')

    def _h_force_claim(self, req, resp):
        self._claimed = True
        return self._ok(resp, 'force_claim: lease force-acquired (simulated)')

    def _command_refused(self):
        """Why the robot would refuse a body command now, as robot_command does."""
        if self._frozen:
            return 'Cannot issue a command to the robot while frozen'
        if self._estopped:
            return 'Robot is estopped. Please use an external E-Stop client.'
        if not self._powered_on:
            return 'Motor power is off'
        return None

    def _h_stop(self, req, resp):
        if (why := self._command_refused()):
            return self._fail(resp, f'stop: {why}')
        self._last_cmd = None
        return self._ok(resp, 'stop: motion halted (simulated)')

    def _h_self_right(self, req, resp):
        if (why := self._command_refused()):
            return self._fail(resp, f'self_right: {why}')
        return self._ok(resp, 'self_right: robot self-righted (simulated)')

    def _h_sit(self, req, resp):
        if (why := self._command_refused()):
            return self._fail(resp, f'sit: {why}')
        self._standing = False
        self._pending = None
        self._publish_state()
        return self._ok(resp, 'sit: robot sitting (simulated)')

    def _h_stand(self, req, resp):
        # Non-blocking on the robot: accepted now, standing once the body is up.
        if (why := self._command_refused()):
            return self._fail(resp, f'stand: {why}')
        if not self._standing:
            self._pending = (self._now() + float(self.get_parameter('stand_s').value), 'stand')
        return self._ok(resp, 'stand: command accepted (simulated)')

    def _h_power_on(self, req, resp):
        if self._estopped:
            return self._fail(resp, 'power_on: Robot is estopped. Please use an external E-Stop client.')
        self._powered_on = True
        self._publish_state()
        return self._ok(resp, 'power_on: motors powered on (simulated)')

    def _h_power_off(self, req, resp):
        # Safe power off: sit, then cut.
        self._standing = False
        self._powered_on = False
        self._pending = None
        self._publish_state()
        return self._ok(resp, 'power_off: motors safely powered off (simulated)')

    # ---- e-stop ----
    def _h_estop_freeze(self, req, resp):
        self._frozen = True
        return self._ok(resp, 'estop/freeze: robot frozen in place (simulated)')

    def _h_estop_unfreeze(self, req, resp):
        self._frozen = False
        return self._ok(resp, 'Robot can now accept new commands')

    def _h_estop_hard(self, req, resp):
        self._estopped = True
        self._standing = False
        self._powered_on = False
        self._pending = None
        self._publish_state()
        return self._ok(resp, 'estop/hard: hard e-stop, motors cut (simulated)')

    def _h_estop_gentle(self, req, resp):
        # ESTOP_LEVEL_SETTLE_THEN_CUT: the body settles, then the motors cut.
        self._estopped = True
        self._standing = False
        if self._powered_on:
            self._pending = (self._now() + float(self.get_parameter('settle_s').value), 'cut')
        self._publish_state()
        return self._ok(resp, 'estop/gentle: settling, then cutting motor power (simulated)')

    def _h_estop_release(self, req, resp):
        # Clears the e-stop only; the motors stay off until power_on.
        self._estopped = False
        self._publish_state()
        return self._ok(resp, 'EStop disengaged; robot can be powered on again')

    # ---- docking ----
    def _h_undock(self, req, resp):
        self._docked = False
        return self._ok(resp, 'undock: robot undocked (simulated)')

    # ===================== SetBool handlers ===============================
    def _h_stair_mode(self, req, resp):
        self._stair_mode = req.data
        return self._ok(resp, f'stair_mode set to {req.data} (simulated)')

    # ===================== spot_msgs/srv handlers =========================
    def _h_locomotion_mode(self, req, resp):
        self._locomotion_mode = req.locomotion_mode
        return self._ok(resp, f'locomotion_mode set to {req.locomotion_mode} (simulated)')

    def _h_max_velocity(self, req, resp):
        v = req.velocity_limit.linear
        w = req.velocity_limit.angular
        return self._ok(resp, f'max_velocity set (lin x={v.x:.2f} y={v.y:.2f}, ang z={w.z:.2f}) (simulated)')

    def _h_clear_behavior_fault(self, req, resp):
        return self._ok(resp, f'clear_behavior_fault: cleared fault id={req.id} (simulated)')

    def _h_toggle_payload(self, req, resp):
        key = req.guid if req.guid else req.name
        self._payloads[key] = req.attached
        state = 'attached' if req.attached else 'detached'
        return self._ok(resp, f"toggle_payload: payload '{key}' {state} (simulated)")

    def _h_register_payload(self, req, resp):
        self._payloads.setdefault(req.guid or req.name, False)
        return self._ok(resp, f"register_payload: registered '{req.name}' (guid={req.guid}) (simulated)")

    def _h_list_graph(self, req, resp):
        # No graph_nav map in sim -- return an empty waypoint list.
        resp.waypoint_ids = []
        self.get_logger().info('[sim] list_graph: returning empty waypoint list (simulated)')
        return resp

    def _h_dock(self, req, resp):
        self._docked = True
        self._standing = False
        return self._ok(resp, f'dock: docked at fiducial id={req.dock_id} (simulated)')

    def _h_gesture_sequence(self, req, resp):
        n = len(req.gesture_sequence)
        return self._ok(resp, f"gesture_sequence: ran '{req.gesture_mode}' with {n} gesture(s) (simulated)")

    # ===================== Action helpers =================================
    def _safe_publish_feedback(self, goal_handle, feedback, fields):
        """Populate `feedback` from `fields` (name -> value), skipping any field
        that doesn't exist on this spot_msgs version, then publish it.

        Feedback field/constant names drift between spot_msgs releases. If a
        handler blindly assigns a missing attribute it raises inside the
        execute callback, and rclpy leaves the goal WEDGED in EXECUTING instead
        of aborting it (ros2/rclpy#296). A polling client such as
        spot_behaviors' WalkToPose then sees STATUS_EXECUTING forever and hangs.
        Guarding every field the way _publish_battery() does keeps that from
        ever happening.
        """
        try:
            for name, value in fields.items():
                if hasattr(feedback, name):
                    setattr(feedback, name, value)
            goal_handle.publish_feedback(feedback)
        except Exception as exc:  # never let feedback wedge the goal
            self.get_logger().warn(f'[sim] feedback skipped: {exc!r}')

    # ===================== Action handlers ================================
    # Locomotion is handled by the navigation stack / CHAMP, so these just
    # report a clean success to the behavior tree -- but only after a brief
    # EXECUTING hold (see _hold_executing / ACTION_SETTLE_S) so a polling client
    # can't miss the terminal status.
    def _hold_executing(self, goal_handle, feedback_type, feedback_fields):
        """Keep the goal in EXECUTING for ACTION_SETTLE_S, streaming feedback.

        Uses a wall-clock loop so it always terminates even if sim time is
        paused. Returns False if the goal was canceled during the hold.
        """
        deadline = time.monotonic() + ACTION_SETTLE_S
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return False
            self._safe_publish_feedback(goal_handle, feedback_type(), feedback_fields)
            time.sleep(0.1)
        return True

    def _h_navigate_to(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(
            f"[sim] navigate_to: goal waypoint '{req.navigate_to}' (simulated)"
        )
        if not self._hold_executing(goal_handle, NavigateTo.Feedback,
                                     {'waypoint_id': req.navigate_to}):
            goal_handle.canceled()
            result = NavigateTo.Result()
            result.success = False
            result.message = 'navigate_to canceled (simulated)'
            return result

        goal_handle.succeed()
        result = NavigateTo.Result()
        result.success = True
        result.message = 'navigate_to complete (simulated)'
        self.get_logger().info('[sim] navigate_to: succeeded')
        return result

    def _h_walk_to(self, goal_handle):
        req = goal_handle.request
        p = req.target_pose.pose.position
        self.get_logger().info(
            f'[sim] walk_to: target ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) (simulated)'
        )
        walk_feedback = {
            'status_enum': getattr(WalkTo.Feedback, 'STATUS_IN_PROGRESS', 0),
            'status_string': 'in_progress',
            'body_status_enum': getattr(WalkTo.Feedback, 'BODY_STATUS_MOVING', 0),
            'body_status_string': 'moving',
            'final_goal_status_enum': getattr(WalkTo.Feedback, 'FINAL_GOAL_STATUS_IN_PROGRESS', 0),
            'final_goal_status_string': 'in_progress',
        }
        if not self._hold_executing(goal_handle, WalkTo.Feedback, walk_feedback):
            goal_handle.canceled()
            result = WalkTo.Result()
            result.success = False
            result.message = 'walk_to canceled (simulated)'
            self.get_logger().info('[sim] walk_to: canceled')
            return result

        goal_handle.succeed()
        result = WalkTo.Result()
        result.success = True
        result.message = 'walk_to complete (simulated)'
        self.get_logger().info('[sim] walk_to: succeeded')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimSpotDriver()
    # MultiThreadedExecutor + ReentrantCallbackGroup so concurrent / nested
    # service calls (common from behavior trees) don't deadlock.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()