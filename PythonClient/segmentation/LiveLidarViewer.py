"""Live GPU lidar point cloud viewer.

Connects to a running AirSim/Cosys-AirSim sim over the API, polls a GPU lidar for
fresh scans (by timestamp) as fast as it can, and renders them in a smooth,
mouse-orbitable 3D view that keeps responding even while no new scan has arrived.
"""
import argparse
import sys
import threading
import time
from collections import deque
from pathlib import Path

import matplotlib
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
import cosysairsim as airsim
from cadslib.lidar import gpu_lidar_point_cloud_to_array, gpu_lidar_rgb_from_points

COLOR_MODES = ["rgb", "intensity", "height"]
WINDOW_NAME = "Live Lidar Viewer"
POINTCLOUD_NAME = "lidar_points"


def points_up_z(points):
    # The point cloud is in NED, where Z increases downward - flip it so "up" renders as up.
    return -points[:, 2]


def points_left_y(points):
    # AirSim's body frame has Y pointing right - flip it so the display convention is
    # X-forward, Y-left, Z-up (matching the origin axis triad), a right-handed frame.
    return -points[:, 1]


def filter_null_points(points):
    # A point that never hit anything (out of range / dropped) comes back as the exact
    # sentinel (0, 0, 0, 0, 0) - drop it so it doesn't show up as a dense fake cluster
    # at the sensor origin.
    if points.shape[0] == 0:
        return points
    nonzero = np.any(points[:, :3] != 0, axis=1)
    return points[nonzero]


def colors_for_mode(points, mode):
    if points.shape[0] == 0:
        return np.zeros((0, 3))
    if mode == "rgb":
        return gpu_lidar_rgb_from_points(points)
    if mode == "intensity":
        values = points[:, 4]
    else:
        values = points_up_z(points)
    vmin, vmax = float(np.min(values)), float(np.max(values))
    norm = (values - vmin) / (vmax - vmin) if vmax > vmin else np.zeros_like(values)
    cmap_name = "viridis" if mode == "intensity" else "turbo"
    return matplotlib.colormaps[cmap_name](norm)[:, :3]


def make_ground_grid(extent, spacing, z=0.0):
    grid_points = []
    lines = []
    n = int(round(extent / spacing))
    for i in range(-n, n + 1):
        offset = i * spacing
        idx = len(grid_points)
        grid_points += [[offset, -extent, z], [offset, extent, z]]
        lines.append([idx, idx + 1])
        idx = len(grid_points)
        grid_points += [[-extent, offset, z], [extent, offset, z]]
        lines.append([idx, idx + 1])

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(grid_points, dtype=np.float64))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines, dtype=np.int32))
    is_axis = [1.0 if i % (2 * n + 1) == n else 0.35 for i in range(len(lines))]
    colors = [[c, c, c] for c in is_axis]
    line_set.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=np.float64))
    return line_set


def make_origin_axes(length):
    # X-forward (red), Y-left (green), Z-up (blue) - matches the display convention
    # applied to the point cloud (points_left_y/points_up_z).
    points = [[0, 0, 0], [length, 0, 0], [0, 0, 0], [0, length, 0], [0, 0, 0], [0, 0, length]]
    lines = [[0, 1], [2, 3], [4, 5]]
    colors = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(points, dtype=np.float64))
    line_set.lines = o3d.utility.Vector2iVector(np.array(lines, dtype=np.int32))
    line_set.colors = o3d.utility.Vector3dVector(np.array(colors, dtype=np.float64))
    return line_set


class LidarPoller:
    """Runs on its own thread with its own connection (msgpack-rpc needs a connection
    created on the thread that uses it) and pushes fresh scans to the GUI thread."""

    def __init__(self, ip, port, lidar_name, on_new_scan, on_status):
        self.ip = ip
        self.port = port
        self.lidar_name = lidar_name
        self.on_new_scan = on_new_scan
        self.on_status = on_status
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=5)

    def _run(self):
        try:
            client = airsim.VehicleClient(ip=self.ip, port=self.port)
            client.confirmConnection()
        except Exception as e:
            self.on_status(connected=False, error=str(e))
            return
        self.on_status(connected=True, error=None)

        last_ts = None
        recent_ts_deltas = deque(maxlen=20)
        while not self._stop.is_set():
            try:
                data = client.getGPULidarData(self.lidar_name)
            except Exception as e:
                self.on_status(connected=False, error=str(e))
                time.sleep(0.5)
                continue

            if data.time_stamp != last_ts:
                if last_ts is not None:
                    recent_ts_deltas.append((data.time_stamp - last_ts) / 1e9)
                last_ts = data.time_stamp

                avg_fps = 0.0
                if recent_ts_deltas:
                    avg_dt = sum(recent_ts_deltas) / len(recent_ts_deltas)
                    if avg_dt > 0:
                        avg_fps = 1.0 / avg_dt

                points = gpu_lidar_point_cloud_to_array(data.point_cloud)
                points = filter_null_points(points)
                self.on_new_scan(points, avg_fps)
            else:
                time.sleep(0.001)


class LiveLidarViewerApp:
    def __init__(self, args):
        self.args = args
        self.color_mode_index = 0
        self.latest_points = np.zeros((0, 5), dtype=np.float32)
        self.avg_fps = 0.0
        self.connected = False
        self.last_error = None
        self.data_lock = threading.Lock()
        self.pending_update = False
        self.ground_z = 0.0
        self.ground_locked = False

        gui.Application.instance.initialize()
        self.window = gui.Application.instance.create_window(WINDOW_NAME, 1280, 900)
        self.vis = o3d.visualization.O3DVisualizer(WINDOW_NAME, 1280, 900)
        self.vis.show_axes = False  # replaced with a bigger, thicker custom origin triad below
        self.vis.show_skybox(False)  # was drawing a sky gradient over the flat background color
        self.vis.point_size = args.point_size
        self.vis.show_settings = False
        self.vis.add_action("Cycle color mode (RGB / Intensity / Height)", self._on_cycle_mode)
        self.vis.set_background([0.45, 0.45, 0.45, 1.0], None)

        self.pcd = o3d.geometry.PointCloud()
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultUnlit"
        material.base_color = [1.0, 1.0, 1.0, 1.0]
        material.point_size = args.point_size
        self.vis.add_geometry(POINTCLOUD_NAME, self.pcd, material)

        self.vis.add_geometry("ground_grid", make_ground_grid(args.grid_extent, args.grid_spacing, 0.0))

        axis_length = args.grid_spacing
        axis_material = o3d.visualization.rendering.MaterialRecord()
        axis_material.shader = "unlitLine"
        axis_material.line_width = 12.0
        self.vis.add_geometry("origin_axes", make_origin_axes(axis_length), axis_material)

        self._redraw_labels()
        self.vis.reset_camera_to_default()
        self.vis.setup_camera(60, [0, 0, 0], [-args.grid_extent, -args.grid_extent, args.grid_extent], [0, 0, 1])

        gui.Application.instance.add_window(self.vis)

        self.poller = LidarPoller(args.ip, args.port, args.lidar_name, self._on_new_scan, self._on_status)
        self.poller.start()

        self._last_label_refresh = 0.0

    def _redraw_labels(self):
        self.vis.clear_3d_labels()
        extent, spacing = self.args.grid_extent, self.args.grid_spacing
        n = int(round(extent / spacing))
        z = self.ground_z
        for i in range(-n, n + 1):
            if i == 0:
                continue
            d = i * spacing
            self.vis.add_3d_label([d, 0.0, z], f"{d:.0f}m")
            self.vis.add_3d_label([0.0, d, z], f"{d:.0f}m")

        self._update_title()

    def _update_title(self):
        mode = COLOR_MODES[self.color_mode_index]
        if self.connected:
            status = f"lidar avg: {self.avg_fps:.1f} Hz  |  color: {mode}  |  points: {self.latest_points.shape[0]}"
        elif self.last_error:
            status = f"DISCONNECTED ({self.last_error})"
        else:
            status = "connecting..."
        self.vis.title = f"{WINDOW_NAME} - {status}"

    def _on_cycle_mode(self, vis):
        self.color_mode_index = (self.color_mode_index + 1) % len(COLOR_MODES)
        self._apply_points_to_geometry()
        self._redraw_labels()
        self.vis.post_redraw()

    def _on_status(self, connected, error):
        self.connected = connected
        self.last_error = error
        gui.Application.instance.post_to_main_thread(self.vis, self._on_status_main)

    def _on_status_main(self):
        self._redraw_labels()
        self.vis.post_redraw()

    def _on_new_scan(self, points, avg_fps):
        with self.data_lock:
            self.latest_points = points
            self.avg_fps = avg_fps
        gui.Application.instance.post_to_main_thread(self.vis, self._on_new_scan_main)

    def _on_new_scan_main(self):
        self._apply_points_to_geometry()
        now = time.time()
        if now - self._last_label_refresh > 0.2:
            self._redraw_labels()
            self._last_label_refresh = now
        self.vis.post_redraw()

    def _apply_points_to_geometry(self):
        with self.data_lock:
            points = self.latest_points
        mode = COLOR_MODES[self.color_mode_index]
        if points.shape[0]:
            up_z = points_up_z(points)
            # AirSim's GPU lidar returns points in its NED body frame (X-forward, Y-right,
            # Z-down), which is left-handed as used here. Converting to X-forward, Y-left,
            # Z-up (negate Y and Z) gives a proper right-handed display frame matching the
            # origin axis triad.
            xyz = np.column_stack((points[:, 0], points_left_y(points), up_z))

            # The sensor sits above whatever it's actually scanning, so a grid fixed at the
            # sensor's own Z=0 visibly hovers above the real ground. Lock the grid/labels to
            # the ground level seen in the first substantial scan instead - once, not every
            # frame, so it doesn't jitter around as new data comes in.
            if not self.ground_locked and points.shape[0] > 50:
                self.ground_z = float(np.percentile(up_z, 2))
                self.ground_locked = True
                self.vis.remove_geometry("ground_grid")
                self.vis.add_geometry("ground_grid",
                    make_ground_grid(self.args.grid_extent, self.args.grid_spacing, self.ground_z))
                self._redraw_labels()
        else:
            xyz = np.zeros((0, 3))
        colors = colors_for_mode(points, mode)

        self.vis.remove_geometry(POINTCLOUD_NAME)
        self.pcd.points = o3d.utility.Vector3dVector(xyz)
        self.pcd.colors = o3d.utility.Vector3dVector(colors)
        material = o3d.visualization.rendering.MaterialRecord()
        material.shader = "defaultUnlit"
        material.base_color = [1.0, 1.0, 1.0, 1.0]
        material.point_size = self.args.point_size
        self.vis.add_geometry(POINTCLOUD_NAME, self.pcd, material)

    def run(self):
        gui.Application.instance.run()
        self.poller.stop()


def parse_args():
    parser = argparse.ArgumentParser(description="Live GPU lidar point cloud viewer")
    parser.add_argument("--ip", default="127.0.0.1", help="AirSim API host")
    parser.add_argument("--port", type=int, default=41451, help="AirSim API port")
    parser.add_argument("--lidar_name", default="lidar", help="GPU lidar sensor name, as in settings.json")
    parser.add_argument("--grid_extent", type=float, default=25.0, help="ground grid half-extent in meters")
    parser.add_argument("--grid_spacing", type=float, default=5.0, help="ground grid line spacing in meters")
    parser.add_argument("--point_size", type=int, default=6, help="rendered point size in pixels")
    return parser.parse_args()


if __name__ == "__main__":
    app = LiveLidarViewerApp(parse_args())
    app.run()
