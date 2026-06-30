// Stream the DiffAero 9x16 perception grid over UDP (preprocessing on-drone).
//
// Reads the VOXL "tof" MPA pipe (tof2_data_t), takes the planar-Z forward depth
// (points[i][2], metres), and runs the PerceptionBuilder-equivalent pipeline
// (svg_ground_control/.../perception_builder.py) tuned for M0178 Starling ToF:
//
//     native planar-Z (240x180 on pmd-tof-liow2)
//       -> orient   (rot90 CCW if W > H; portrait 180x240)
//       -> flips    (Starling 2 mount correction, optional)
//       -> clean     (non-finite / z<=1e-3  ->  far clip = 5 m)
//       -> crop      86 deg angular window around optical axis (training FOV)
//       -> euclid    (planar Z * per-pixel ray scale from sensor intrinsics)
//       -> min-pool  linspace bins  ->  9x16 nearest-surface range grid
//       -> encode    1 - clip(range, 0, 5) / 5   (1 = surface at lens)
//
// The resulting 9x16 float grid is sent as ONE UDP packet in the ground
// bridge's "legacy TOF2" format (magic 'TOF2', header <IIQ>, then 144 float32).
// ground/tof_udp_bridge.py already decodes this (see _handle_legacy_encoded),
// so NO ground-side change is needed -- it just republishes the grid.
//
// This is bit-for-bit identical to what the ground previously computed from the
// raw TOF3 stream, only the min-pool/encode now runs on the drone (cheap) and
// the link carries 592 bytes/frame instead of the full point cloud.
//
// Compile on VOXL:
//   g++ -std=c++14 -O2 tof_udp_stream.cpp \
//       -o tof_udp_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm
//
// Run:
//   ./tof_udp_stream <ground_pc_ip> [port=5600] [--no-flip-h] [--no-flip-v] [--stream-raw]
//
// Starling 2: default +flip-v only on the 9x16 grid (pitch). Do not flip-h by
// default — it mirrors yaw (CCW rotation appears CW). Pass --flip-h to re-enable.

#include <modal_pipe_client.h>
#include <modal_pipe_interfaces.h>

#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {

// --- DiffAero perception grid (matches perception_builder.py) --------------
constexpr int kOutW = 16;
constexpr int kOutH = 9;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kPolicyFovDeg = 86.0f;   // DiffAero training horizontal FOV
constexpr float kPolicyMaxDist = 5.0f;   // far clip + encoding clip [m]
// M0178 / pmd-tof-liow2 native sensor FOV (ModalAI datasheet: 106 deg x 86 deg)
constexpr float kSensorHFovDeg = 86.0f;
constexpr float kSensorVFovDeg = 106.0f;

// --- UDP wire format: ground bridge "legacy TOF2" (pre-encoded 9x16) --------
constexpr uint32_t kUdpMagicEnc = 0x544F4632u;  // 'TOF2'

struct EncHeader {
    uint32_t magic;
    uint32_t seq;
    uint64_t timestamp_ns;
} __attribute__((packed));

constexpr size_t kGridFloats = static_cast<size_t>(kOutH) * kOutW;  // 144
constexpr size_t kEncPacketSize = sizeof(EncHeader) + kGridFloats * sizeof(float);  // 592

// --- UDP wire format: TOF3 chunked raw oriented planar-Z (debug) ------------
constexpr uint32_t kUdpMagicRaw = 0x544F4633u;  // 'TOF3'

struct RawHeader {
    uint32_t magic;
    uint32_t seq;
    uint32_t frame_id;
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint64_t timestamp_ns;
    uint16_t width;
    uint16_t height;
} __attribute__((packed));

constexpr size_t kRawHeaderSize = sizeof(RawHeader);
constexpr size_t kMaxChunkPayload = 60000;  // bytes; fits in one UDP datagram

// Starling 2 mount correction: flips apply to the pooled 9x16 grid (same stage as
// PerceptionBuilder._maybe_flip), not the raw planar image — pre-crop flip_h
// inverts yaw when using the 86 deg crop pipeline.
struct Config {
    bool flip_lr = false;
    bool flip_ud = true;
    bool stream_raw = false;
    bool raw_only = false;  // tuning: TOF3 only, skip TOF2 encode
    enum class CropVAnchor { Center, Bottom, Top } crop_v_anchor = CropVAnchor::Center;
    int crop_v_shift_px = 0;
};

std::atomic<bool> g_stop{false};
int g_sock = -1;
sockaddr_in g_dest{};
Config g_cfg;
uint32_t g_seq = 0;
uint64_t g_rate_window_start_ns = 0;
uint32_t g_rate_window_frames = 0;

struct SensorIntrinsics {
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;
    int h = 0;
    int w = 0;
};

struct Crop {
    int row0 = 0;
    int row1 = 0;
    int col0 = 0;
    int col1 = 0;
    int h() const { return row1 - row0; }
    int w() const { return col1 - col0; }
};

static SensorIntrinsics intrinsics_for_frame(int h, int w, bool landscape_rotated) {
    SensorIntrinsics intr{};
    intr.h = h;
    intr.w = w;
    intr.cx = 0.5f * static_cast<float>(w);
    intr.cy = 0.5f * static_cast<float>(h);
    const float deg = kPi / 180.0f;
    if (landscape_rotated) {
        // After rot90: width axis = native height (106 deg), height = native width (86 deg).
        intr.fx = 0.5f * static_cast<float>(w) / std::tan(0.5f * kSensorVFovDeg * deg);
        intr.fy = 0.5f * static_cast<float>(h) / std::tan(0.5f * kSensorHFovDeg * deg);
    } else {
        intr.fx = 0.5f * static_cast<float>(w) / std::tan(0.5f * kSensorHFovDeg * deg);
        intr.fy = 0.5f * static_cast<float>(h) / std::tan(0.5f * kSensorVFovDeg * deg);
    }
    return intr;
}

static Crop compute_crop(const SensorIntrinsics& intr, float fov_deg) {
    const float half = 0.5f * fov_deg * kPi / 180.0f;
    const float half_w_px = intr.fx * std::tan(half);
    const float half_h_px = intr.fy * std::tan(half);
    Crop crop{};
    crop.col0 = std::max(0, static_cast<int>(std::floor(intr.cx - half_w_px)));
    crop.col1 = std::min(intr.w, static_cast<int>(std::ceil(intr.cx + half_w_px)));

    const int crop_h = std::max(1, std::min(intr.h, static_cast<int>(std::ceil(2.0f * half_h_px))));
    int row0 = 0;
    int row1 = intr.h;
    switch (g_cfg.crop_v_anchor) {
        case Config::CropVAnchor::Center:
            row0 = std::max(0, static_cast<int>(std::floor(intr.cy - half_h_px)));
            row1 = std::min(intr.h, static_cast<int>(std::ceil(intr.cy + half_h_px)));
            break;
        case Config::CropVAnchor::Bottom:
            row1 = intr.h;
            row0 = std::max(0, row1 - crop_h);
            break;
        case Config::CropVAnchor::Top:
            row0 = 0;
            row1 = std::min(intr.h, crop_h);
            break;
    }
    if (g_cfg.crop_v_shift_px != 0) {
        row0 = std::max(0, std::min(intr.h - 1, row0 + g_cfg.crop_v_shift_px));
        row1 = std::max(row0 + 1, std::min(intr.h, row1 + g_cfg.crop_v_shift_px));
    }
    crop.row0 = row0;
    crop.row1 = row1;
    return crop;
}

static void log_stream_rate(uint64_t timestamp_ns) {
    if (g_rate_window_start_ns == 0) {
        g_rate_window_start_ns = timestamp_ns;
    }
    g_rate_window_frames++;
    const double elapsed_s = static_cast<double>(timestamp_ns - g_rate_window_start_ns) * 1e-9;
    if (elapsed_s < 1.0) {
        return;
    }
    const double hz = static_cast<double>(g_rate_window_frames) / elapsed_s;
    std::printf("streaming %.1f Hz (9x16 perception)\n", hz);
    std::fflush(stdout);
    g_rate_window_start_ns = timestamp_ns;
    g_rate_window_frames = 0;
}

static void signal_handler(int) {
    g_stop.store(true);
}

// np.linspace(0, n, num_points)[k].astype(int) -- matches perception_builder.py.
static inline int linspace_edge(int k, int n, int num_points) {
    if (num_points <= 1) {
        return 0;
    }
    return static_cast<int>(static_cast<double>(k) * static_cast<double>(n) /
                            static_cast<double>(num_points - 1));
}

static inline float clean_planar_z(float z) {
    if (!std::isfinite(z) || z <= 1e-3f) {
        return kPolicyMaxDist;
    }
    return z;
}

// Oriented planar-Z (oh x ow, row-major) -> 9x16 perception grid.
// Matches PerceptionBuilder: crop to training FOV -> euclid -> min-pool -> encode.
static void planar_to_perception(const std::vector<float>& oriented, int oh, int ow,
                                 bool landscape_rotated,
                                 std::array<float, kGridFloats>& perception) {
    const SensorIntrinsics intr = intrinsics_for_frame(oh, ow, landscape_rotated);
    const Crop crop = compute_crop(intr, kPolicyFovDeg);
    const int crop_h = crop.h();
    const int crop_w = crop.w();
    if (crop_h <= 0 || crop_w <= 0) {
        perception.fill(0.0f);
        return;
    }

    std::vector<float> euclid(static_cast<size_t>(crop_h * crop_w));
    for (int r = 0; r < crop_h; ++r) {
        const int vr = crop.row0 + r;
        for (int c = 0; c < crop_w; ++c) {
            const int uc = crop.col0 + c;
            const float z = clean_planar_z(oriented[static_cast<size_t>(vr * ow + uc)]);
            const float xn = (static_cast<float>(uc) - intr.cx) / intr.fx;
            const float yn = (static_cast<float>(vr) - intr.cy) / intr.fy;
            const float scale = std::sqrt(1.0f + xn * xn + yn * yn);
            euclid[static_cast<size_t>(r * crop_w + c)] = z * scale;
        }
    }

    for (int oy = 0; oy < kOutH; ++oy) {
        const int r0 = linspace_edge(oy, crop_h, kOutH + 1);
        const int r1 = linspace_edge(oy + 1, crop_h, kOutH + 1);
        for (int ox = 0; ox < kOutW; ++ox) {
            const int c0 = linspace_edge(ox, crop_w, kOutW + 1);
            const int c1 = linspace_edge(ox + 1, crop_w, kOutW + 1);
            float cell_min = kPolicyMaxDist;
            for (int r = r0; r < r1; ++r) {
                for (int c = c0; c < c1; ++c) {
                    cell_min = std::min(cell_min, euclid[static_cast<size_t>(r * crop_w + c)]);
                }
            }
            const float range_m = std::max(0.0f, std::min(cell_min, kPolicyMaxDist));
            perception[static_cast<size_t>(oy * kOutW + ox)] = 1.0f - range_m / kPolicyMaxDist;
        }
    }
}

static void apply_grid_flips(std::array<float, kGridFloats>& perception) {
    if (!g_cfg.flip_lr && !g_cfg.flip_ud) {
        return;
    }
    const std::array<float, kGridFloats> tmp = perception;
    for (int oy = 0; oy < kOutH; ++oy) {
        const int sy = g_cfg.flip_ud ? (kOutH - 1 - oy) : oy;
        for (int ox = 0; ox < kOutW; ++ox) {
            const int sx = g_cfg.flip_lr ? (kOutW - 1 - ox) : ox;
            perception[static_cast<size_t>(oy * kOutW + ox)] =
                tmp[static_cast<size_t>(sy * kOutW + sx)];
        }
    }
}

struct OrientedPlanar {
    std::vector<float> data;
    int h = 0;
    int w = 0;
    bool landscape_rotated = false;
};

// Build oriented planar-Z from a tof2 frame (matches depth_preprocess orient step).
static OrientedPlanar build_oriented_planar(const tof2_data_t* tof) {
    const int width = tof->width;
    const int height = tof->height;
    const int n = width * height;

    std::vector<float> planar(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        planar[static_cast<size_t>(i)] = tof->points[i][2];
    }

    OrientedPlanar out;
    int oh, ow;
    if (width > height) {
        out.landscape_rotated = true;
        oh = width;
        ow = height;
        out.data.resize(static_cast<size_t>(oh * ow));
        for (int i = 0; i < oh; ++i) {
            for (int j = 0; j < ow; ++j) {
                out.data[static_cast<size_t>(i * ow + j)] =
                    planar[static_cast<size_t>(j * width + (width - 1 - i))];
            }
        }
    } else {
        oh = height;
        ow = width;
        out.data = std::move(planar);
    }

    out.h = oh;
    out.w = ow;
    return out;
}

static void planar_to_perception(const OrientedPlanar& frame,
                                 std::array<float, kGridFloats>& perception) {
    planar_to_perception(frame.data, frame.h, frame.w, frame.landscape_rotated, perception);
}

static void send_raw_planar(const OrientedPlanar& frame, uint64_t timestamp_ns) {
    if (frame.h <= 0 || frame.w <= 0 || frame.data.empty()) {
        return;
    }

    const size_t total_bytes = frame.data.size() * sizeof(float);
    const size_t payload_per_chunk = kMaxChunkPayload - (kMaxChunkPayload % sizeof(float));
    const uint16_t chunk_count = static_cast<uint16_t>(
        (total_bytes + payload_per_chunk - 1) / payload_per_chunk);
    if (chunk_count == 0) {
        return;
    }

    static uint32_t raw_frame_id = 0;
    const uint32_t frame_id = raw_frame_id++;
    const uint32_t seq = g_seq;
    const auto* bytes = reinterpret_cast<const char*>(frame.data.data());

    for (uint16_t ci = 0; ci < chunk_count; ++ci) {
        const size_t offset = static_cast<size_t>(ci) * payload_per_chunk;
        const size_t chunk_bytes = std::min(payload_per_chunk, total_bytes - offset);

        std::vector<char> packet(kRawHeaderSize + chunk_bytes);
        RawHeader hdr{};
        hdr.magic = kUdpMagicRaw;
        hdr.seq = seq;
        hdr.frame_id = frame_id;
        hdr.chunk_index = ci;
        hdr.chunk_count = chunk_count;
        hdr.timestamp_ns = timestamp_ns;
        hdr.width = static_cast<uint16_t>(frame.w);
        hdr.height = static_cast<uint16_t>(frame.h);
        std::memcpy(packet.data(), &hdr, kRawHeaderSize);
        std::memcpy(packet.data() + kRawHeaderSize, bytes + offset, chunk_bytes);

        const ssize_t sent = sendto(g_sock, packet.data(), packet.size(), 0,
                                    reinterpret_cast<sockaddr*>(&g_dest), sizeof(g_dest));
        if (sent != static_cast<ssize_t>(packet.size())) {
            std::fprintf(stderr, "sendto raw chunk failed (%zd)\n", sent);
        }
    }
}

static void send_perception(const tof2_data_t* tof) {
    if (tof->width <= 0 || tof->height <= 0) {
        return;
    }

    const OrientedPlanar frame = build_oriented_planar(tof);
    const uint64_t timestamp_ns = static_cast<uint64_t>(tof->timestamp_ns);
    const uint32_t seq = g_seq++;

    if (!g_cfg.raw_only) {
        std::array<float, kGridFloats> perception{};
        planar_to_perception(frame, perception);
        apply_grid_flips(perception);

        std::array<char, kEncPacketSize> packet{};
        EncHeader hdr{};
        hdr.magic = kUdpMagicEnc;
        hdr.seq = seq;
        hdr.timestamp_ns = timestamp_ns;
        std::memcpy(packet.data(), &hdr, sizeof(hdr));
        std::memcpy(packet.data() + sizeof(hdr), perception.data(), kGridFloats * sizeof(float));

        const ssize_t sent = sendto(g_sock, packet.data(), packet.size(), 0,
                                    reinterpret_cast<sockaddr*>(&g_dest), sizeof(g_dest));
        if (sent != static_cast<ssize_t>(packet.size())) {
            std::fprintf(stderr, "sendto failed (%zd)\n", sent);
        }
    }

    if (g_cfg.stream_raw || g_cfg.raw_only) {
        send_raw_planar(frame, timestamp_ns);
    }

    log_stream_rate(timestamp_ns);
}

static void tof_callback(int ch, char* data, int bytes, void* context) {
    (void)ch;
    (void)context;
    if (bytes < static_cast<int>(sizeof(tof2_data_t))) {
        return;
    }
    send_perception(reinterpret_cast<const tof2_data_t*>(data));
}

static bool parse_args(int argc, char** argv, const char** host, int* port, Config* cfg) {
    if (argc < 2) {
        return false;
    }
    *host = argv[1];
    *port = 5600;
    for (int i = 2; i < argc; ++i) {
        if (std::strcmp(argv[i], "--flip-h") == 0) {
            cfg->flip_lr = true;
        } else if (std::strcmp(argv[i], "--no-flip-h") == 0) {
            cfg->flip_lr = false;
        } else if (std::strcmp(argv[i], "--flip-v") == 0) {
            cfg->flip_ud = true;
        } else if (std::strcmp(argv[i], "--no-flip-v") == 0) {
            cfg->flip_ud = false;
        } else if (std::strcmp(argv[i], "--stream-raw") == 0) {
            cfg->stream_raw = true;
        } else if (std::strcmp(argv[i], "--raw-only") == 0) {
            cfg->raw_only = true;
            cfg->stream_raw = true;
        } else if (std::strncmp(argv[i], "--crop-v-anchor=", 16) == 0) {
            const char* val = argv[i] + 16;
            if (std::strcmp(val, "bottom") == 0) {
                cfg->crop_v_anchor = Config::CropVAnchor::Bottom;
            } else if (std::strcmp(val, "top") == 0) {
                cfg->crop_v_anchor = Config::CropVAnchor::Top;
            } else {
                cfg->crop_v_anchor = Config::CropVAnchor::Center;
            }
        } else if (std::strncmp(argv[i], "--crop-v-shift=", 15) == 0) {
            cfg->crop_v_shift_px = std::atoi(argv[i] + 15);
        } else {
            *port = std::atoi(argv[i]);
        }
    }
    return *port > 0 && *port <= 65535;
}

}  // namespace

int main(int argc, char** argv) {
    const char* host = nullptr;
    int port = 5600;
    if (!parse_args(argc, argv, &host, &port, &g_cfg)) {
        std::fprintf(stderr,
                     "Usage: %s <ground_pc_ip> [port=5600] [--no-flip-h] [--no-flip-v] "
                     "[--stream-raw] [--raw-only] "
                     "[--crop-v-anchor=center|bottom|top] [--crop-v-shift=N]\n",
                     argv[0]);
        return 1;
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    g_sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_sock < 0) {
        std::perror("socket");
        return 1;
    }

    std::memset(&g_dest, 0, sizeof(g_dest));
    g_dest.sin_family = AF_INET;
    g_dest.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, host, &g_dest.sin_addr) != 1) {
        std::fprintf(stderr, "Invalid ground PC IP: %s\n", host);
        close(g_sock);
        return 1;
    }

    const int rc = pipe_client_open(
        0, "tof", "tof_udp_stream",
        CLIENT_FLAG_EN_SIMPLE_HELPER,
        sizeof(tof2_data_t));
    if (rc != 0) {
        std::fprintf(stderr, "pipe_client_open(tof) failed: %d\n", rc);
        close(g_sock);
        return 1;
    }

    pipe_client_set_simple_helper_cb(0, tof_callback, nullptr);

    const char* anchor_label = "center";
    if (g_cfg.crop_v_anchor == Config::CropVAnchor::Bottom) {
        anchor_label = "bottom";
    } else if (g_cfg.crop_v_anchor == Config::CropVAnchor::Top) {
        anchor_label = "top";
    }
    std::printf(
        "Streaming %s to %s:%d (%zu B/frame%s%s%s crop=%s shift=%d). Ctrl-C to stop.\n",
        g_cfg.raw_only ? "TOF3 raw (raw-only)" : "TOF2 9x16",
        host, port, kEncPacketSize,
        g_cfg.flip_lr ? " +flip-h" : " -flip-h",
        g_cfg.flip_ud ? " +flip-v" : " -flip-v",
        g_cfg.stream_raw && !g_cfg.raw_only ? " +TOF3 raw" : "",
        anchor_label, g_cfg.crop_v_shift_px);

    while (!g_stop.load()) {
        usleep(100000);
    }

    pipe_client_close_all();
    close(g_sock);
    std::printf("Closed.\n");
    return 0;
}
