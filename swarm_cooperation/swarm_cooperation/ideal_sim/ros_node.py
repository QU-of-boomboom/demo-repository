"""ROS 2 Humble 入口：使用 Gazebo /clock；纯算法核心仍可独立运行。"""
import json
from pathlib import Path

from .artifacts import write_artifacts
from .config import ExperimentConfig
from .gazebo_transport import GazeboView
from .simulation import IdealSimulation


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Path as PathMessage

    class ExperimentNode(Node):
        def __init__(self):
            super().__init__("ideal_relay_scheduler")
            for name, default in (("mode", "random"), ("seed", 42), ("rounds", 5),
                                  ("window_start_s", 2.), ("window_end_s", 12.),
                                  ("model_plugin", ""), ("gazebo_view", True),
                                  ("predictor_path", ""), ("strategy", "predeploy"),
                                  ("map_observation", "ideal"), ("output_dir", "outputs_v2/ros")):
                self.declare_parameter(name, default)
            value = lambda name: self.get_parameter(name).value
            cfg = ExperimentConfig(mode=value("mode"), seed=value("seed"), rounds=value("rounds"),
                                   window_start_s=value("window_start_s"), window_end_s=value("window_end_s"),
                                   model_plugin=value("model_plugin"), predictor_path=value("predictor_path"),
                                   strategy=value("strategy"), map_observation=value("map_observation"))
            self.event_pub = self.create_publisher(String, "/swarm/events", 100)
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.state_pub = self.create_publisher(String, "/swarm/state", latched)
            self.path_pub = self.create_publisher(PathMessage, "/swarm/main/path", latched)
            self.command_sub = self.create_subscription(String, "/swarm/remove_cells", self.on_command, 10)
            self.sim = IdealSimulation(cfg, output=self.on_event)
            self.sim.run_context = {"entrypoint": "ros2", "window_input": "topic"}
            self.view = GazeboView() if value("gazebo_view") else None
            self.report_dir = Path(value("output_dir")).expanduser()
            self.start_time = None
            self.last_view_time = -1.
            self.last_error = None
            self.saved = False
            self.path = PathMessage()
            self.path.header.frame_id = "map"
            self.timer = self.create_timer(cfg.dt_s, self.tick)
            self.get_logger().info("等待仿真时钟；窗口命令发至 /swarm/remove_cells，坐标为1基货箱编号")

        def on_event(self, event):
            payload = json.dumps(event, ensure_ascii=False)
            self.event_pub.publish(String(data=payload))
            self.get_logger().info(payload)

        def on_command(self, message):
            try:
                command = json.loads(message.data)
                if not isinstance(command, dict):
                    raise ValueError("命令须为JSON对象")
                command_id = command.get("id", "manual-1")
                result = self.sim.submit_removal(command.get("cells", []), command_id)
                self.on_event({"kind": "command_result", "command_id": command_id, **result})
            except (ValueError, TypeError) as exc:
                self.on_event({"kind": "invalid_command", "accepted": False, "reason": str(exc)})

        def tick(self):
            now = self.get_clock().now().nanoseconds / 1e9
            if self.start_time is None:
                self.start_time = now
            elapsed = now - self.start_time
            if elapsed + 1e-9 < self.sim.now:
                self.get_logger().error("仿真时钟回退，请重启本实验节点以重置五轮任务")
                self.timer.cancel()
                return
            # 回调延后时补步，单次最多20步；不会读取将来的地图事件。
            for _ in range(20):
                if self.sim.finished or self.sim.now + self.sim.cfg.dt_s > elapsed + 1e-9:
                    break
                self.sim.step()
            snapshot = self.sim.snapshot()
            self.state_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
            pose = PoseStamped()
            pose.header.frame_id, pose.header.stamp = "map", self.get_clock().now().to_msg()
            pose.pose.position.x = self.sim.position[0] * self.sim.cfg.cell_m
            pose.pose.position.y = self.sim.position[1] * self.sim.cfg.cell_m
            pose.pose.position.z, pose.pose.orientation.w = 1.5, 1.
            if not self.path.poses or self.path.poses[-1].pose.position != pose.pose.position:
                self.path.poses.append(pose)
            self.path.header.stamp = pose.header.stamp
            self.path_pub.publish(self.path)
            if self.view and (self.sim.now - self.last_view_time >= .2 - 1e-9 or self.sim.finished):
                self.view.submit(snapshot, self.sim.cfg.cell_m)
                self.last_view_time = self.sim.now
                if self.view.last_error and self.view.last_error != self.last_error:
                    self.get_logger().warning("Gazebo显示未同步：" + self.view.last_error)
                self.last_error = self.view.last_error
            if self.sim.finished and not self.saved:
                directory = write_artifacts(self.sim, self.report_dir)
                self.saved = True
                self.get_logger().info(f"实验结束，success={self.sim.report()['success']}，产物：{directory}")
                if self.sim.cfg.mode == "window" and self.sim.removal is None:
                    self.get_logger().warning("未接受移箱命令，本次不算窗口变更实验验收通过")
                self.timer.cancel()

        def destroy_node(self):
            if self.sim.predictor is not None:
                self.sim.predictor.close()
            if self.view:
                self.view.close()
            if not self.saved:
                write_artifacts(self.sim, self.report_dir)
            super().destroy_node()

    rclpy.init(args=args)
    node = ExperimentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
