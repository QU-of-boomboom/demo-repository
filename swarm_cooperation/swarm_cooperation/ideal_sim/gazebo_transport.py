"""Fortress 位姿服务适配；独立线程最多保留一个最新帧，不阻塞调度。

使用 ign CLI 的参数列表调用，不经过 shell；服务接受只说明可视化命令入队。
"""
import queue
import subprocess
import threading


def pose_request(snapshot, cell_m):
    poses = [("main", *snapshot["main"], 1.5),
             ("relay1", *snapshot["relays"][0], 1.8),
             ("relay2", *snapshot["relays"][1], 2.1)]
    removed = set(map(tuple, snapshot["removed"]))
    poses.extend((f"box_{x}_{y}", x - .5, y - .5, -10. if (x, y) in removed else .4)
                 for x in range(1, 4) for y in range(1, 5))
    return " ".join(f'pose {{ name: "{name}" position {{ x: {x*cell_m:.6f} y: {y*cell_m:.6f} z: {z} }} '
                    'orientation { w: 1 } }' for name, x, y, z in poses)


class GazeboView:
    def __init__(self):
        self.pending = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.last_error = None
        self.worker = threading.Thread(target=self._run, daemon=True)
        self.worker.start()

    def submit(self, snapshot, cell_m):
        request = pose_request(snapshot, cell_m)
        try:
            self.pending.get_nowait()
        except queue.Empty:
            pass
        self.pending.put_nowait(request)

    def _run(self):
        while not self.stop.is_set():
            try:
                request = self.pending.get(timeout=.2)
            except queue.Empty:
                continue
            try:
                result = subprocess.run(
                    ["ign", "service", "-s", "/world/warehouse/set_pose_vector",
                     "--reqtype", "ignition.msgs.Pose_V", "--reptype", "ignition.msgs.Boolean",
                     "--timeout", "1000", "--req", request],
                    capture_output=True, text=True, timeout=2, check=False)
                if result.returncode or "data: true" not in result.stdout:
                    self.last_error = (result.stderr or result.stdout or "Gazebo未确认位姿请求").strip()
                else:
                    self.last_error = None
            except (OSError, subprocess.TimeoutExpired) as exc:
                self.last_error = str(exc)

    def close(self):
        self.stop.set()
        self.worker.join(timeout=2.5)
