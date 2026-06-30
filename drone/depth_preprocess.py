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
POLICY_FOV_DEG = 86.0  # DiffAero training crop FOV
# M0178 / pmd-tof-liow2 (ModalAI datasheet: 106 deg vertical x 86 deg horizontal)
SENSOR_HFOV_DEG = 86.0
SENSOR_VFOV_DEG = 106.0


class CropConfig:
    """Vertical placement of the 86 deg training FOV window on the native grid.

    Image rows increase downward (top = ceiling on a forward-facing ToF).
    v_anchor='bottom' discards the top of the frame (ceiling) first.
    v_shift_px > 0 moves the window down (more ceiling discarded).
    """

    __slots__ = ('v_anchor', 'v_shift_px')

    def __init__(self, v_anchor: str = 'center', v_shift_px: int = 0):
        anchor = str(v_anchor).lower()
        if anchor not in ('center', 'bottom', 'top'):
            raise ValueError(f'v_anchor must be center|bottom|top, got {v_anchor!r}')
        self.v_anchor = anchor
        self.v_shift_px = int(v_shift_px)


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
    """Legacy: resize full frame to 64x36 render grid, euclid, 4x4 min-pool."""
    planar = _resize_nearest(planar, RENDER_H, RENDER_W)
    planar = _replace_nonfinite(planar, far_clip)
    planar = np.where(planar <= 1e-3, far_clip, planar).astype(np.float32)
    euclid = planar * _EUCLID_SCALE
    return _min_pool_to_policy(euclid)


def _intrinsics_oriented(h: int, w: int, landscape_rotated: bool) -> tuple[float, float, float, float]:
    """Pinhole intrinsics for oriented Starling ToF (matches tof_udp_stream.cpp)."""
    cx, cy = 0.5 * w, 0.5 * h
    if landscape_rotated:
        fx = 0.5 * w / math.tan(0.5 * math.radians(SENSOR_VFOV_DEG))
        fy = 0.5 * h / math.tan(0.5 * math.radians(SENSOR_HFOV_DEG))
    else:
        fx = 0.5 * w / math.tan(0.5 * math.radians(SENSOR_HFOV_DEG))
        fy = 0.5 * h / math.tan(0.5 * math.radians(SENSOR_VFOV_DEG))
    return fx, fy, cx, cy


def _compute_crop(fx: float, fy: float, cx: float, cy: float, h: int, w: int,
                  fov_deg: float, crop: CropConfig | None = None) -> tuple[int, int, int, int]:
    """86 deg angular crop; vertical anchor tunable (center / bottom / top)."""
    crop = crop or CropConfig()
    half = math.radians(fov_deg / 2)
    half_w_px = fx * math.tan(half)
    half_h_px = fy * math.tan(half)
    col0 = max(0, int(math.floor(cx - half_w_px)))
    col1 = min(w, int(math.ceil(cx + half_w_px)))

    crop_h = max(1, int(math.ceil(2.0 * half_h_px)))
    crop_h = min(crop_h, h)
    if crop.v_anchor == 'center':
        row0 = max(0, int(math.floor(cy - half_h_px)))
        row1 = min(h, int(math.ceil(cy + half_h_px)))
    elif crop.v_anchor == 'bottom':
        row1 = h
        row0 = max(0, row1 - crop_h)
    else:  # top
        row0 = 0
        row1 = min(h, crop_h)

    if crop.v_shift_px:
        row0 = max(0, min(h - 1, row0 + crop.v_shift_px))
        row1 = max(row0 + 1, min(h, row1 + crop.v_shift_px))
    return row0, row1, col0, col1


def _compute_pool_edges(crop: tuple[int, int, int, int], out_h: int, out_w: int):
    row0, row1, col0, col1 = crop
    crop_h = row1 - row0
    crop_w = col1 - col0
    row_edges = np.linspace(0, crop_h, out_h + 1).astype(int)
    col_edges = np.linspace(0, crop_w, out_w + 1).astype(int)
    return row_edges, col_edges


def _euclid_scale_crop(h: int, w: int, fx: float, fy: float, cx: float, cy: float,
                       crop: tuple[int, int, int, int]) -> np.ndarray:
    u = np.arange(w, dtype=np.float32)
    v = np.arange(h, dtype=np.float32)
    uu, vv = np.meshgrid(u, v)
    xn = (uu - cx) / fx
    yn = (vv - cy) / fy
    scale = np.sqrt(1.0 + xn * xn + yn * yn).astype(np.float32)
    row0, row1, col0, col1 = crop
    return scale[row0:row1, col0:col1]


def _min_pool_edges(euclid: np.ndarray, row_edges: np.ndarray, col_edges: np.ndarray,
                    out_h: int, out_w: int) -> np.ndarray:
    out = np.empty((out_h, out_w), dtype=np.float32)
    for i in range(out_h):
        for j in range(out_w):
            cell = euclid[row_edges[i]:row_edges[i + 1], col_edges[j]:col_edges[j + 1]]
            out[i, j] = float(cell.min())
    return out


def planar_to_range_grid_perception(planar: np.ndarray, landscape_rotated: bool,
                                    far_clip: float = POLICY_MAX_DIST,
                                    crop: CropConfig | None = None) -> np.ndarray:
    """PerceptionBuilder path: crop to training FOV -> euclid -> linspace min-pool."""
    h, w = planar.shape[:2]
    planar = _replace_nonfinite(planar, far_clip)
    planar = np.where(planar <= 1e-3, far_clip, planar).astype(np.float32)
    fx, fy, cx, cy = _intrinsics_oriented(h, w, landscape_rotated)
    crop_box = _compute_crop(fx, fy, cx, cy, h, w, POLICY_FOV_DEG, crop)
    row0, row1, col0, col1 = crop_box
    planar_crop = planar[row0:row1, col0:col1]
    euclid_scale = _euclid_scale_crop(h, w, fx, fy, cx, cy, crop_box)
    euclid = planar_crop * euclid_scale
    row_edges, col_edges = _compute_pool_edges(crop_box, OUT_H, OUT_W)
    return _min_pool_edges(euclid, row_edges, col_edges, OUT_H, OUT_W)


def raw_planar_z_to_perception_debug(
    planar_z: np.ndarray,
    flip_lr: bool = False,
    flip_ud: bool = True,
    crop: CropConfig | None = None,
) -> tuple[np.ndarray, tuple[int, int, int, int], np.ndarray]:
    """Returns (9x16 perception, (row0,row1,col0,col1), oriented planar meters)."""
    planar = np.asarray(planar_z, dtype=np.float32)
    landscape_rotated = planar.shape[1] > planar.shape[0]
    planar = orient_raw_tof(planar)
    h, w = planar.shape[:2]
    fx, fy, cx, cy = _intrinsics_oriented(h, w, landscape_rotated)
    crop_box = _compute_crop(fx, fy, cx, cy, h, w, POLICY_FOV_DEG, crop)
    range_m = planar_to_range_grid_perception(
        planar, landscape_rotated, far_clip=POLICY_MAX_DIST, crop=crop)
    encoded = encode_policy_perception(range_m, POLICY_MAX_DIST)
    encoded = _maybe_flip_grid(encoded, flip_lr, flip_ud)
    return encoded, crop_box, planar


def _maybe_flip_grid(pooled: np.ndarray, flip_lr: bool, flip_ud: bool) -> np.ndarray:
    if flip_lr:
        pooled = pooled[:, ::-1].copy()
    if flip_ud:
        pooled = pooled[::-1, :].copy()
    return pooled


def raw_planar_z_to_perception(planar_z: np.ndarray,
                               flip_lr: bool = False,
                               flip_ud: bool = True,
                               crop: CropConfig | None = None) -> np.ndarray:
    """Native TOF planar-Z grid (meters) -> DiffAero 9x16 policy perception."""
    perception, _, _ = raw_planar_z_to_perception_debug(
        planar_z, flip_lr=flip_lr, flip_ud=flip_ud, crop=crop)
    return perception


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
