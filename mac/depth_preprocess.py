"""Convert Starling TOF data into DiffAero's 9x16 metric range grid."""

import math

import numpy as np

# DiffAero camera grid (matches run_px4_sim.py / diffaero/cfg/sensor/camera.yaml).
OUT_W, OUT_H = 16, 9
POOL = 4
RENDER_W, RENDER_H = OUT_W * POOL, OUT_H * POOL  # 64 x 36
FOV_X_DEG = 86.0
FOV_Y_DEG = FOV_X_DEG * OUT_H / OUT_W  # 48.375 deg
MAX_RANGE = 6.0  # sensor far clip for invalid pixels
POLICY_MAX_DIST = 5.0  # DiffAero perception encoding clip [m]


def _replace_nonfinite(arr, value):
    """Replace nan/inf with value (VOXL numpy lacks nan_to_num nan= kwargs)."""
    out = np.asarray(arr, dtype=np.float32)
    bad = ~np.isfinite(out)
    if bad.any():
        out = out.copy()
        out[bad] = value
    return out


def _euclid_scale_map(width: int, height: int, fov_x_deg: float, fov_y_deg: float) -> np.ndarray:
    """Per-pixel scale: planar Z-depth -> Euclidean range."""
    fx = 0.5 * width / math.tan(0.5 * math.radians(fov_x_deg))
    fy = 0.5 * height / math.tan(0.5 * math.radians(fov_y_deg))
    cx, cy = 0.5 * width, 0.5 * height
    u = np.arange(width, dtype=np.float32)
    v = np.arange(height, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    xn = (uu - cx) / fx
    yn = (vv - cy) / fy
    return np.sqrt(1.0 + xn * xn + yn * yn).astype(np.float32)


_EUCLID_SCALE = _euclid_scale_map(RENDER_W, RENDER_H, FOV_X_DEG, FOV_Y_DEG)
_FX = 0.5 * RENDER_W / math.tan(0.5 * math.radians(FOV_X_DEG))
_FY = 0.5 * RENDER_H / math.tan(0.5 * math.radians(FOV_Y_DEG))
_CX, _CY = 0.5 * RENDER_W, 0.5 * RENDER_H


class DepthConfig(object):
    def __init__(self, depth_min=0.1, depth_max=6.0, flip_h=False, flip_v=False):
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.flip_h = flip_h
        self.flip_v = flip_v


def _apply_flips(arr: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    if cfg.flip_v:
        arr = arr[::-1]
    if cfg.flip_h:
        arr = arr[:, ::-1]
    return arr


def orient_raw_tof(arr: np.ndarray) -> np.ndarray:
    """Starling TOF is portrait (tall); ROS /tof_depth is often stored wide — rotate 90° CCW."""
    h, w = arr.shape[:2]
    if w > h:
        return np.rot90(arr, k=1)
    return arr


def prepare_raw_tof(arr: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """Orient + optional CLI flips for display and mono8 decode."""
    return _apply_flips(orient_raw_tof(arr), cfg)


def depth_u8_to_hotcold_bgr(gray: np.ndarray) -> np.ndarray:
    """Grayscale depth -> BGR with hot (close/high) / cold (far/low) colormap."""
    import cv2

    if gray.dtype != np.uint8:
        gray = np.clip(gray, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)


def gray_u8_to_bgr(gray: np.ndarray) -> np.ndarray:
    """Grayscale uint8 or float -> BGR for OpenCV display (no colormap)."""
    import cv2

    if gray.dtype != np.uint8:
        gray = np.clip(gray, 0, 255).astype(np.uint8)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def _resize_nearest(planar_z: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
    if planar_z.shape == (out_h, out_w):
        return planar_z
    yi = np.linspace(0, planar_z.shape[0] - 1, out_h).astype(np.int64)
    xi = np.linspace(0, planar_z.shape[1] - 1, out_w).astype(np.int64)
    return planar_z[yi][:, xi]


def _planar_to_euclid(planar_z: np.ndarray) -> np.ndarray:
    """Convert (RENDER_H, RENDER_W) planar depth to Euclidean range."""
    if planar_z.shape != (RENDER_H, RENDER_W):
        planar_z = _resize_nearest(planar_z, RENDER_H, RENDER_W)
    far = MAX_RANGE
    planar_z = _replace_nonfinite(planar_z, far)
    return planar_z * _EUCLID_SCALE


def _min_pool_to_policy(euclid: np.ndarray) -> np.ndarray:
    """Min-pool RENDER grid to OUT grid (nearest surface per cell)."""
    if euclid.shape != (RENDER_H, RENDER_W):
        euclid = _resize_nearest(euclid, RENDER_H, RENDER_W)
    pooled = euclid.reshape(OUT_H, POOL, OUT_W, POOL).min(axis=(1, 3))
    return np.ascontiguousarray(pooled, dtype=np.float32)


def decode_mono8_depth_u8(depth_u8: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """Decode mono8 debug image to metric planar depth (H, W)."""
    depth_u8 = prepare_raw_tof(depth_u8, cfg)
    span = cfg.depth_max - cfg.depth_min
    return cfg.depth_min + (depth_u8.astype(np.float32) / 255.0) * span


def policy_depth_config(flip_h: bool = False, flip_v: bool = False) -> DepthConfig:
    """DepthConfig aligned with bag replay / on-drone policy preprocessing."""
    return DepthConfig(depth_min=0.0, depth_max=POLICY_MAX_DIST, flip_h=flip_h, flip_v=flip_v)


def decode_mono8_planar_policy(depth_u8: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """mono8 -> metric planar depth; invalid (0) pixels -> cfg.depth_max."""
    depth_u8 = prepare_raw_tof(depth_u8, cfg)
    valid = depth_u8 > 0
    span = cfg.depth_max - cfg.depth_min
    planar = cfg.depth_min + (depth_u8.astype(np.float32) / 255.0) * span
    planar[~valid] = cfg.depth_max
    return planar


def encode_policy_perception(
    range_m: np.ndarray, max_dist: float = POLICY_MAX_DIST
) -> np.ndarray:
    """Euclidean range grid -> policy perception (1 = close)."""
    return (1.0 - np.clip(range_m, 0.0, max_dist) / max_dist).astype(np.float32)


def planar_to_range_grid(planar: np.ndarray, far_clip: float = POLICY_MAX_DIST) -> np.ndarray:
    """Resize planar depth to render grid, convert to Euclidean, min-pool to 9x16."""
    planar = _resize_nearest(planar, RENDER_H, RENDER_W)
    planar = _replace_nonfinite(planar, far_clip)
    planar = np.where(planar <= 1e-3, far_clip, planar).astype(np.float32)
    euclid = planar * _EUCLID_SCALE
    return _min_pool_to_policy(euclid)


def depth_u8_to_perception(depth_u8, cfg=None):
    """mono8 depth image -> (perception 9x16, range_9x16 Euclidean)."""
    cfg = cfg or policy_depth_config()
    planar = decode_mono8_planar_policy(depth_u8, cfg)
    range_m = planar_to_range_grid(planar, far_clip=cfg.depth_max)
    return encode_policy_perception(range_m, cfg.depth_max), range_m


def depth_image_to_range(depth_u8: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """mono8 / float depth image -> (9, 16) Euclidean range."""
    planar = decode_mono8_depth_u8(depth_u8, cfg)
    return planar_to_range_grid(planar, far_clip=cfg.depth_max)


def depth_image_to_vis(depth_u8: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """Native-resolution hot/cold depth BGR (no resize)."""
    return depth_u8_to_hotcold_bgr(prepare_raw_tof(depth_u8, cfg))


def parse_image_msg(msg):
    """Parse sensor_msgs/Image to uint8 depth or None if unsupported."""
    if msg.encoding == "mono8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
    if msg.encoding == "32FC1":
        depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        depth_u8 = np.clip(
            (depth_m - 0.1) / (6.0 - 0.1) * 255.0, 0, 255
        ).astype(np.uint8)
        return depth_u8
    return None


def _pointcloud2_xyz(msg):
    """Extract Nx3 float32 XYZ from PointCloud2."""
    field_map = {f.name: f for f in msg.fields}
    for name in ("x", "y", "z"):
        if name not in field_map:
            raise ValueError(f"PointCloud2 missing '{name}' field")

    little_endian = not msg.is_bigendian
    endian = "<" if little_endian else ">"
    point_step = msg.point_step
    n_points = len(msg.data) // point_step

    offsets = [field_map[n].offset for n in ("x", "y", "z")]
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [f"{endian}f4", f"{endian}f4", f"{endian}f4"],
            "offsets": offsets,
            "itemsize": point_step,
        }
    )
    cloud = np.frombuffer(msg.data, dtype=dtype, count=n_points)
    xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=-1)
    valid = np.isfinite(xyz).all(axis=1)
    return xyz[valid]


def pointcloud_to_range(msg, cfg):
    """Project PointCloud2 -> (9,16) Euclidean range + (RENDER_H,RENDER_W) planar vis grid."""
    xyz = _pointcloud2_xyz(msg)
    planar = np.full((RENDER_H, RENDER_W), MAX_RANGE, dtype=np.float32)

    # Camera frame: x right, y down, z forward (ModalAI TOF convention).
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    in_front = z > cfg.depth_min
    x, y, z = x[in_front], y[in_front], z[in_front]

    u = (_FX * x / z + _CX).astype(np.int32)
    v = (_FY * y / z + _CY).astype(np.int32)
    in_bounds = (u >= 0) & (u < RENDER_W) & (v >= 0) & (v < RENDER_H)
    u, v, z = u[in_bounds], v[in_bounds], z[in_bounds]

    for ui, vi, zi in zip(u, v, z):
        if zi < planar[vi, ui]:
            planar[vi, ui] = zi

    planar = _apply_flips(planar, cfg)
    euclid = _planar_to_euclid(planar)
    return _min_pool_to_policy(euclid), planar


def pointcloud_to_vis(planar: np.ndarray, cfg: DepthConfig) -> np.ndarray:
    """Projected depth grid as hot/cold BGR."""
    planar = _apply_flips(orient_raw_tof(planar), cfg)
    vis_u8 = (np.clip(planar / cfg.depth_max, 0.0, 1.0) * 255.0).astype(np.uint8)
    # Invert so close = hot (brighter range -> higher u8 after 1 - norm)
    vis_u8 = (255 - vis_u8).astype(np.uint8)
    return depth_u8_to_hotcold_bgr(vis_u8)


def perception_to_vis(perception: np.ndarray, scale: int = 20) -> np.ndarray:
    """Upscale 9x16 policy perception [0,1] to hot/cold BGR (1=close=hot)."""
    import cv2

    p = np.clip(perception, 0.0, 1.0)
    p_u8 = (p * 255.0).astype(np.uint8)
    big = cv2.resize(
        p_u8,
        (OUT_W * scale, OUT_H * scale),
        interpolation=cv2.INTER_NEAREST,
    )
    return depth_u8_to_hotcold_bgr(big)
