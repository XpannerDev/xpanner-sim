"""
isaac_sensors.py -- the perception the demo carries on the house: a roof LiDAR mounted with its spin axis
horizontal (a vertical fan that the swing sweeps across the site -- the Bedrock-style terrain survey), a
wide semantic camera for people, and a spectator camera for the recording.

All three are prims under the house link (or the world), updated on render, read with Replicator
annotators. Points are returned in WORLD frame.
"""
import math
from collections import deque

import numpy as np


def _np(a):
    return np.asarray(a.numpy() if hasattr(a, "numpy") else a)


def _dist_to_segment(pts, a, b):
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-9:
        return np.linalg.norm(pts - a, axis=1)
    s = np.clip(((pts - a) @ ab) / L2, 0.0, 1.0)
    return np.linalg.norm(pts - (a + s[:, None] * ab), axis=1)


def _xform_world(prim):
    from pxr import Usd, UsdGeom
    M = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array([[M[c][r] for c in range(4)] for r in range(4)])          # Gf is row-vector; transpose


class RoofLidar:
    """RTX LiDAR (isaacsim.sensors.experimental.rtx, the 6.0.1 API) as a child of `parent_path`, at `translation`
    in the parent frame, rotated by `rotation_xyz_deg`. A spinning LiDAR's native frame spins about +Z;
    (90, 0, 0) lays that axis along the house y, so the beams sweep a VERTICAL plane (forward - up - back -
    down) and the house swing paints the surroundings. Points are returned in WORLD frame."""

    def __init__(self, parent_path, name="roof_lidar", translation=(0.3, 0.0, 1.0),
                 rotation_xyz_deg=(90.0, 0.0, 0.0), config="Example_Rotary", keep_s=10.0):
        from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor
        from scipy.spatial.transform import Rotation as R
        import omni.usd

        q = R.from_euler("xyz", rotation_xyz_deg, degrees=True).as_quat()        # x, y, z, w
        self.local_T = np.eye(4)
        self.local_T[:3, :3] = R.from_euler("xyz", rotation_xyz_deg, degrees=True).as_matrix()
        self.local_T[:3, 3] = translation
        self.path = f"{parent_path}/{name}"
        self.lidar = Lidar.create(self.path, config=config, translations=np.array(translation, float),
                                  orientations=np.array([q[3], q[0], q[1], q[2]]), aux_output_level="BASIC")
        self.sensor = LidarSensor(self.lidar, annotators=["generic-model-output"])
        self.prim = omni.usd.get_context().get_stage().GetPrimAtPath(self.path)
        self.buffer = deque()
        self.keep_s = keep_s
        self.frames = 0
        self.points_total = 0
        self.last_frame_of_reference = None
        self.frame_decision = None
        self.last_pose = None
        self.coords_type = None
        self.angles_in_degrees = None
        self.spherical_override = True        # see read(): the enum said CARTESIAN while the data was spherical
        self.az_sign = 1.0                    # flipped by calibrate_azimuth() if the world lands mirrored
        self.below_grade = 0

    def read(self, t, parent_T=None, exclude=()):
        """Pull this frame's returns, move them to world with the sensor pose, keep the last keep_s seconds.
        parent_T: world 4x4 of the parent link from the physics view (the USD xform can lag or, with Fabric,
        never update); the sensor's own offset is composed on it. None falls back to the USD xform.
        exclude: [(p0, p1, radius), ...] world segments (the machine's own boom, arm, bucket) whose returns are
        dropped, so the arm hanging over the pile is not surveyed as terrain (run 5: 13 m3 for a 4 m3 pile)."""
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
        raw, _info = self.sensor.get_data("generic-model-output")
        gmo = parse_generic_model_output_data(raw)
        pts = None
        if gmo is not None and getattr(gmo, "numElements", 0):
            raw = np.stack([np.asarray(gmo.x, float), np.asarray(gmo.y, float), np.asarray(gmo.z, float)], axis=1)
            raw = raw[np.isfinite(raw).all(axis=1)]
            self.last_frame_of_reference = int(getattr(gmo, "frameOfReference", -1))
            self.coords_type = int(getattr(gmo, "elementsCoordsType", 0))
            if self.coords_type == 1 or self.spherical_override:
                # SPHERICAL: (azimuth, elevation, range). Run 4 with Example_Rotary/BASIC delivered exactly this,
                # with the angles in DEGREES (azimuth +-180, elevation -15..+10), while elementsCoordsType read 0.
                az, el, rng = raw[:, 0], raw[:, 1], raw[:, 2]
                if np.nanmax(np.abs(az)) > 2 * math.pi * 1.05:
                    az, el = np.radians(az), np.radians(el)
                self.angles_in_degrees = bool(np.nanmax(np.abs(raw[:, 0])) > 2 * math.pi * 1.05)
                keep = rng > 0.05
                az, el, rng = az[keep], el[keep], rng[keep]
                raw = np.column_stack([rng * np.cos(el) * np.cos(az * self.az_sign), rng * np.cos(el) * np.sin(az * self.az_sign),
                                       rng * np.sin(el)])
            M = _xform_world(self.prim) if parent_T is None else np.asarray(parent_T, float) @ self.local_T
            self.last_pose = M
            moved = (M[:3, :3] @ raw.T).T + M[:3, 3]
            # Which frame the GMO is in is decided by the data, not by the enum (recorded for the report): the
            # ground plane is z = 0 in world, so the candidate whose median z is nearest 0 is the world cloud.
            if len(raw) > 200:
                z_raw, z_mv = abs(float(np.median(raw[:, 2]))), abs(float(np.median(moved[:, 2])))
                self.frame_decision = "world_already" if z_raw < z_mv else "sensor_frame"
            else:
                self.frame_decision = getattr(self, "frame_decision", "sensor_frame")
            pts = raw if self.frame_decision == "world_already" else moved
            pts = pts[np.linalg.norm(pts - M[:3, 3], axis=1) > 0.6]        # drop self-hits on the house
            # Returns below grade are impossible here (flat ground plane) yet ~3 % of the pile-face returns in run 6
            # came back 10-25 % longer than the ground range (z -0.3..-0.8). Unexplained; see docs/LOADING_DEMO.md.
            self.below_grade += int(np.sum(pts[:, 2] < -0.05))
            pts = pts[pts[:, 2] >= -0.05]
            for p0, p1, rad in exclude:
                pts = pts[_dist_to_segment(pts, np.asarray(p0, float), np.asarray(p1, float)) > rad]
            self.buffer.append((float(t), pts))
            self.frames += 1
            self.points_total += len(pts)
        while self.buffer and self.buffer[0][0] < t - self.keep_s:
            self.buffer.popleft()
        return pts

    def cloud(self, since=None):
        parts = [p for tt, p in self.buffer if since is None or tt >= since]
        return np.vstack(parts) if parts else np.zeros((0, 3))

    def calibrate_azimuth(self, box_centre_xy, half, z_lo=0.1, z_hi=2.0):
        """A vertical fan cannot tell forward from backward if the azimuth sign is wrong: the stockpile at +x
        would land at -x. Count elevated points inside the known pile box for the current sign and for the
        mirror (x -> reflected about the sensor), keep the better sign for future frames. Returns the counts."""
        cl = self.cloud()
        if not len(cl) or self.last_pose is None:
            return None
        c = np.asarray(box_centre_xy, float)

        def count(pts):
            return int(np.sum((np.abs(pts[:, 0] - c[0]) < half) & (np.abs(pts[:, 1] - c[1]) < half) & (pts[:, 2] > z_lo) & (pts[:, 2] < z_hi)))

        n_now = count(cl)
        # mirror: reflect the azimuth = reflect the point about the sensor's local x axis in the HOUSE frame; with the
        # house at ~0 yaw during the reference sweep this is x -> 2*sx - x
        sx = self.last_pose[0, 3]
        mirrored = cl.copy()
        mirrored[:, 0] = 2 * sx - mirrored[:, 0]
        n_mir = count(mirrored)
        if n_mir > 3 * max(n_now, 1):
            self.az_sign *= -1.0
            self.buffer = deque((t, np.column_stack([2 * sx - p[:, 0], p[:, 1], p[:, 2]])) for t, p in self.buffer)
        return dict(points_in_pile_box=n_now, mirrored=n_mir, az_sign=self.az_sign)

    def stats(self):
        cl = self.cloud()
        if not len(cl):
            return dict(points=0)
        return dict(points=int(len(cl)), frames=self.frames, frame_of_reference=self.last_frame_of_reference,
                    frame_decision=self.frame_decision, coords_type=self.coords_type, angles_in_degrees=self.angles_in_degrees,
                    az_sign=self.az_sign, below_grade_dropped=self.below_grade,
                    bbox=[cl.min(axis=0).round(2).tolist(), cl.max(axis=0).round(2).tolist()],
                    z_percentiles=np.percentile(cl[:, 2], [5, 50, 95]).round(2).tolist(),
                    sensor_world_pos=None if self.last_pose is None else self.last_pose[:3, 3].round(2).tolist())


class RoofCamera:
    """Wide RGB-D camera with semantic segmentation, for people. Returns the nearest 'person' depth."""

    def __init__(self, parent_path, name="roof_cam", translation=(0.6, 0.3, 1.0), rpy_deg=(0.0, 15.0, 45.0),
                 resolution=(480, 300), hfov_deg=120.0):
        from isaacsim.sensors.camera import Camera
        from scipy.spatial.transform import Rotation as R

        # The constructor hands `orientation` to set_local_pose(camera_axes="world"): +X forward, +Z up, in the
        # parent frame (camera.py:303-310, 842-885). So plain roll/pitch/yaw about the parent axes is right
        # (pitch +12 = nose down, yaw +40 = left). An extra USD-axes rotation here rolled the images 90 deg
        # (visuals check, 10-02); run 3's "looking at the ground behind" was the camera sitting inside the cab box.
        q = R.from_euler("xyz", rpy_deg, degrees=True).as_quat()               # x, y, z, w
        self.path = f"{parent_path}/{name}"
        self.cam = Camera(prim_path=self.path, translation=np.array(translation, float),
                          orientation=np.array([q[3], q[0], q[1], q[2]]), resolution=resolution)
        self.hfov = hfov_deg
        self.resolution = resolution
        self.world_xyz = None
        self.debug = {}

    def initialize(self):
        self.cam.initialize()
        # horizontal FOV from aperture / focal length: tan(h/2) = (aperture/2) / f
        f = self.cam.get_focal_length()
        self.cam.set_horizontal_aperture(2 * f * math.tan(math.radians(self.hfov / 2)))
        self.cam.set_clipping_range(0.1, 200.0)
        self.cam.add_distance_to_image_plane_to_frame()
        self.cam.add_semantic_segmentation_to_frame()
        self.cam.add_bounding_box_2d_tight_to_frame()

    def person(self):
        """(distance m or None, pixel count) of the nearest 'person' pixels this frame. self.debug keeps what
        the last call saw (label table, id histogram, depth stats) so a false detection can be explained."""
        fr = self.cam.get_current_frame()
        seg = fr.get("semantic_segmentation")
        depth = fr.get("distance_to_image_plane")
        self.debug = dict(seg_type=str(type(seg)), depth_shape=None)
        if not seg or depth is None:
            return None, 0
        ids = seg.get("data")
        info = seg.get("info", {})
        labels = info.get("idToLabels", {})
        self.debug["labels"] = {str(k): str(v) for k, v in labels.items()}
        person_ids = [int(k) for k, v in labels.items() if "person" in str(v).lower()]
        if ids is None or not person_ids:
            return None, 0
        ids = _np(ids)
        depth = _np(depth)
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        self.debug.update(ids_shape=list(ids.shape), ids_dtype=str(ids.dtype), depth_shape=list(depth.shape),
                          person_ids=person_ids)
        if ids.ndim == 3:                                   # colorised output: cannot be matched by id
            self.debug["note"] = "semantic_segmentation is colorised (HxWx4); no id match possible"
            return None, 0
        mask = np.isin(ids, person_ids)
        n = int(mask.sum())
        if n < 12:                                          # fewer pixels than a person at 40 m: noise
            return None, n
        d = depth[mask]
        ok = np.isfinite(d) & (d > 0.3)
        d = d[ok]
        self.debug.update(n=n, d_min=float(d.min()) if len(d) else None, d_p10=float(np.percentile(d, 10)) if len(d) else None,
                          d_med=float(np.median(d)) if len(d) else None,
                          hist={int(u): int(c) for u, c in zip(*np.unique(ids, return_counts=True))})
        if not len(d):
            return None, n
        # where in the world: the centroid pixel of the person at the median depth, through the camera model
        vv, uu = np.nonzero(mask)
        u_c, v_c = float(np.mean(uu[ok])), float(np.mean(vv[ok]))
        self.world_xyz = None
        try:
            w = self.cam.get_world_points_from_image_coords(np.array([[u_c, v_c]]), np.array([float(np.median(d))]))
            w = _np(w).reshape(-1, 3)
            self.world_xyz = w[0].astype(float)
        except Exception as exc:                                 # pinhole-only helper; keep the distance anyway
            self.debug["world_err"] = str(exc)
        return float(np.percentile(d, 10)), n

    def rgb(self):
        return self.cam.get_rgba()[:, :, :3]

    def pointcloud_world(self):
        """Depth image -> world points (the deprecated Camera does the unprojection with its own pose)."""
        try:
            pc = self.cam.get_pointcloud(world_frame=True)
            pc = _np(pc).reshape(-1, 3).astype(float)
            return pc[np.isfinite(pc).all(axis=1)]
        except Exception:
            return np.zeros((0, 3))


class Spectator:
    """A fixed camera for the recording / the viewer's default view."""

    def __init__(self, eye, target, resolution=(960, 540), path="/World/spectator_cam"):
        from isaacsim.sensors.camera import Camera
        self.cam = Camera(prim_path=path, position=np.array(eye, float), resolution=resolution)
        self.eye, self.target = np.array(eye, float), np.array(target, float)

    def initialize(self):
        self.cam.initialize()
        self.cam.set_clipping_range(0.1, 300.0)
        self.aim()

    def aim(self):
        """Point the camera at target. The deprecated Camera class takes poses in its "world" camera axes:
        +X forward, +Y left, +Z up (not the USD -Z/+Y convention), so build the frame that way."""
        from scipy.spatial.transform import Rotation as R
        f = self.target - self.eye
        f /= np.linalg.norm(f)
        up = np.array([0.0, 0.0, 1.0])
        left = np.cross(up, f); left /= np.linalg.norm(left)
        u = np.cross(f, left)
        M = np.column_stack([f, left, u])
        q = R.from_matrix(M).as_quat()          # x, y, z, w
        self.cam.set_world_pose(position=self.eye, orientation=np.array([q[3], q[0], q[1], q[2]]), camera_axes="world")

    def rgb(self):
        return self.cam.get_rgba()[:, :, :3]
