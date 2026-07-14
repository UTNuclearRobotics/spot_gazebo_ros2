#!/usr/bin/env python3
# ---------------------------------------------------------------------------
# sim_simulation_pose.py
#
# Simulation stand-in for the "/spot_simulation/set_robot_pose" service that the
# SetSimulatedPose behavior (spot_behaviors/set_simulated_pose.cpp) calls.
#
# On the real robot there is no such service -- SetSimulatedPose only exists to
# teleport the *simulated* Spot to a known pose (e.g. to reset the world between
# behavior-tree runs). Nothing in your current graph advertises it, so every
# tick of set_simulated_pose_* hits wait_for_service() and returns FAILURE.
#
# This node provides:
#     /spot_simulation/set_robot_pose   (spot_msgs/srv/SetSimulatedPose)
#
# and fulfils each request by, in order:
#   1. Teleporting the Gazebo model via the gz set_pose service, bridged to ROS
#      as ros_gz_interfaces/srv/SetEntityPose (default name
#      "/world/<world_name>/set_pose").
#   2. Resetting the robot_localization EKF seed via /set_pose
#      (robot_localization/srv/SetPose) so the filter doesn't fight the jump.
#   3. Resetting the Nav2 estimate via /spot_nav/set_initial_pose
#      (nav2_msgs/srv/SetInitialPose) so AMCL/costmaps re-center.
#
# Steps 2 and 3 are best-effort: if those services aren't up the teleport still
# happens and we log a warning. Overall success is driven by the teleport.
#
# Parameters (all overridable):
#   world_name            (string, default "")  -- REQUIRED for teleport. The gz
#                          world name. If left empty the node logs an error and
#                          returns failure for the teleport step. Find it with:
#                              gz service -l | grep set_pose
#                          e.g. "/world/spot_world/set_pose" -> world_name:=spot_world
#   gz_set_pose_service   (string, default "/world/<world_name>/set_pose")
#                          -- override if your bridge remapped it.
#   model_name            (string, default "") -- gz model/entity name to move.
#                          If empty, the request's robot_name field is used.
#   update_ekf            (bool, default True)
#   update_nav            (bool, default True)
#   ekf_set_pose_service  (string, default "/set_pose")
#   nav_set_pose_service  (string, default "/spot_nav/set_initial_pose")
#   service_timeout       (float, default 5.0) -- per downstream call, seconds.
#
# Usage:
#   ros2 run <your_pkg> sim_simulation_pose.py --ros-args -p world_name:=spot_world
# ---------------------------------------------------------------------------

import threading

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

from spot_msgs.srv import SetSimulatedPose

# Downstream service types. Imported lazily-tolerant so a missing optional
# dependency (e.g. ros_gz_interfaces not installed) degrades gracefully.
try:
    from ros_gz_interfaces.srv import SetEntityPose
    from ros_gz_interfaces.msg import Entity
    _HAVE_ROS_GZ = True
except Exception:  # pragma: no cover - depends on host install
    SetEntityPose = None
    Entity = None
    _HAVE_ROS_GZ = False

try:
    from robot_localization.srv import SetPose as EkfSetPose
    _HAVE_EKF = True
except Exception:  # pragma: no cover
    EkfSetPose = None
    _HAVE_EKF = False

try:
    from nav2_msgs.srv import SetInitialPose as NavSetInitialPose
    _HAVE_NAV = True
except Exception:  # pragma: no cover
    NavSetInitialPose = None
    _HAVE_NAV = False


class SimSimulationPose(Node):
    def __init__(self):
        super().__init__('spot_simulation')

        # ---- Parameters ----
        self.declare_parameter('world_name', '')
        self.declare_parameter('gz_set_pose_service', '')
        self.declare_parameter('model_name', '')
        self.declare_parameter('update_ekf', True)
        self.declare_parameter('update_nav', True)
        self.declare_parameter('ekf_set_pose_service', '/set_pose')
        self.declare_parameter('nav_set_pose_service', '/spot_nav/set_initial_pose')
        self.declare_parameter('service_timeout', 5.0)

        world = self.get_parameter('world_name').value
        gz_srv = self.get_parameter('gz_set_pose_service').value
        if not gz_srv:
            gz_srv = f'/world/{world}/set_pose' if world else ''
        self._gz_service_name = gz_srv
        self._model_name = self.get_parameter('model_name').value
        self._update_ekf = bool(self.get_parameter('update_ekf').value)
        self._update_nav = bool(self.get_parameter('update_nav').value)
        self._ekf_service_name = self.get_parameter('ekf_set_pose_service').value
        self._nav_service_name = self.get_parameter('nav_set_pose_service').value
        self._timeout = float(self.get_parameter('service_timeout').value)

        # Separate groups: server is mutually-exclusive (one teleport at a time),
        # clients are reentrant so their responses can be processed by another
        # executor thread while the server handler blocks waiting on them.
        self._srv_group = MutuallyExclusiveCallbackGroup()
        self._client_group = ReentrantCallbackGroup()

        # ---- Downstream clients ----
        self._gz_client = None
        if _HAVE_ROS_GZ and self._gz_service_name:
            self._gz_client = self.create_client(
                SetEntityPose, self._gz_service_name, callback_group=self._client_group)
        elif not _HAVE_ROS_GZ:
            self.get_logger().error(
                'ros_gz_interfaces not importable; teleport disabled. '
                'Install ros-<distro>-ros-gz-interfaces or override gz_set_pose_service.')
        elif not self._gz_service_name:
            self.get_logger().error(
                'No world_name / gz_set_pose_service set; teleport disabled. '
                "Pass -p world_name:=<your_gz_world>.")

        self._ekf_client = None
        if self._update_ekf and _HAVE_EKF:
            self._ekf_client = self.create_client(
                EkfSetPose, self._ekf_service_name, callback_group=self._client_group)

        self._nav_client = None
        if self._update_nav and _HAVE_NAV:
            self._nav_client = self.create_client(
                NavSetInitialPose, self._nav_service_name, callback_group=self._client_group)

        # ---- The service the behavior actually calls ----
        self.create_service(
            SetSimulatedPose, '~/set_robot_pose', self._on_set_pose,
            callback_group=self._srv_group)

        self.get_logger().info(
            f"sim_simulation_pose up: /spot_simulation/set_robot_pose ready "
            f"(teleport via '{self._gz_service_name or '<disabled>'}').")

    # ------------------------------------------------------------------ #
    # Synchronous service call helper, safe under a MultiThreadedExecutor.
    # We do NOT spin here (the executor is already spinning in other threads);
    # we just wait on the future via a done-callback + Event.
    # ------------------------------------------------------------------ #
    def _call_sync(self, client, request, what):
        if client is None:
            return None
        if not client.wait_for_service(timeout_sec=self._timeout):
            self.get_logger().warning(
                f"{what}: service '{client.srv_name}' not available within "
                f"{self._timeout:.1f}s")
            return None
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(self._timeout):
            self.get_logger().warning(f"{what}: timed out waiting for response")
            return None
        try:
            return future.result()
        except Exception as exc:  # pragma: no cover
            self.get_logger().warning(f"{what}: call raised {exc!r}")
            return None

    # ------------------------------------------------------------------ #
    # Service handler
    # ------------------------------------------------------------------ #
    def _on_set_pose(self, req: SetSimulatedPose.Request, resp: SetSimulatedPose.Response):
        pose_stamped = req.pose
        model = self._model_name or req.robot_name or 'spot'

        # 1) Teleport the gz model -------------------------------------------------
        teleport_ok = False
        if self._gz_client is not None:
            gz_req = SetEntityPose.Request()
            gz_req.entity = Entity()
            gz_req.entity.name = model
            # Entity.MODEL == 2 across ros_gz_interfaces versions; fall back to 2.
            gz_req.entity.type = getattr(Entity, 'MODEL', 2)
            gz_req.pose = pose_stamped.pose
            result = self._call_sync(self._gz_client, gz_req, 'gz set_pose')
            teleport_ok = bool(result is not None and getattr(result, 'success', False))
            if not teleport_ok:
                self.get_logger().warning(
                    f"Teleport of model '{model}' did not report success")
            else:
                p = pose_stamped.pose.position
                self.get_logger().info(
                    f"Teleported '{model}' to ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) "
                    f"in frame '{pose_stamped.header.frame_id}'")
        else:
            self.get_logger().error("Teleport skipped: no gz set_pose client configured")

        # Build a PoseWithCovarianceStamped once for the estimators.
        pwcs = PoseWithCovarianceStamped()
        pwcs.header = pose_stamped.header
        pwcs.pose.pose = pose_stamped.pose
        # Small, non-zero diagonal so the filters trust the reset but stay stable.
        cov = [0.0] * 36
        for i, v in ((0, 0.05), (7, 0.05), (14, 0.05), (21, 0.02), (28, 0.02), (35, 0.05)):
            cov[i] = v
        pwcs.pose.covariance = cov

        # 2) Reset the EKF ---------------------------------------------------------
        if self._ekf_client is not None:
            ekf_req = EkfSetPose.Request()
            ekf_req.pose = pwcs
            self._call_sync(self._ekf_client, ekf_req, 'EKF /set_pose')

        # 3) Reset Nav2 ------------------------------------------------------------
        if self._nav_client is not None:
            nav_req = NavSetInitialPose.Request()
            nav_req.pose = pwcs
            self._call_sync(self._nav_client, nav_req, 'Nav2 set_initial_pose')

        resp.success = teleport_ok
        # 'message' is optional across spot_msgs versions; set only if present.
        if hasattr(resp, 'message'):
            resp.message = (
                'teleport complete (simulated)' if teleport_ok
                else 'teleport failed: see sim_simulation_pose log')
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = SimSimulationPose()
    # use_sim_time so timeouts/logging line up with /clock like the rest of the sim.
    node.set_parameters([rclpy.parameter.Parameter(
        'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    executor = MultiThreadedExecutor(num_threads=4)
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
