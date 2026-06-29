// Stream the DiffAero 9x16 perception grid over UDP (preprocessing on-drone).
//
// Reads the VOXL "tof" MPA pipe (tof2_data_t), takes the planar-Z forward depth
// (points[i][2], metres), and runs the *exact* DiffAero preprocessing pipeline
// from tof_streamer/drone/depth_preprocess.py::raw_planar_z_to_perception:
//
//     native planar-Z (H x W)
//       -> orient   (rot90 CCW if W > H, Starling portrait convention)
//       -> resize    nearest to the 64x36 render grid
//       -> clean     (non-finite / z<=1e-3  ->  far clip = 5 m)
//       -> euclid    (planar Z * per-pixel ray scale, 86 x 48.375 deg FOV)
//       -> min-pool  4x4  ->  9x16 nearest-surface range grid
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
//   ./tof_udp_stream <ground_pc_ip> [port=5600] [--flip-h] [--flip-v]

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

// --- DiffAero perception grid (matches drone/depth_preprocess.py) -----------
constexpr int kOutW = 16;
constexpr int kOutH = 9;
constexpr int kPool = 4;
constexpr int kRenderW = kOutW * kPool;  // 64
constexpr int kRenderH = kOutH * kPool;  // 36
constexpr float kPi = 3.14159265358979323846f;
constexpr float kFovXDeg = 86.0f;
constexpr float kFovYDeg = kFovXDeg * static_cast<float>(kOutH) / static_cast<float>(kOutW);  // 48.375
constexpr float kPolicyMaxDist = 5.0f;  // far clip + encoding clip [m]

// --- UDP wire format: ground bridge "legacy TOF2" (pre-encoded 9x16) --------
constexpr uint32_t kUdpMagicEnc = 0x544F4632u;  // 'TOF2'

struct EncHeader {
    uint32_t magic;
    uint32_t seq;
    uint64_t timestamp_ns;
} __attribute__((packed));

constexpr size_t kGridFloats = static_cast<size_t>(kOutH) * kOutW;  // 144
constexpr size_t kEncPacketSize = sizeof(EncHeader) + kGridFloats * sizeof(float);  // 592

struct Config {
    bool flip_h = false;
    bool flip_v = false;
};

std::atomic<bool> g_stop{false};
int g_sock = -1;
sockaddr_in g_dest{};
Config g_cfg;
uint32_t g_seq = 0;
uint64_t g_rate_window_start_ns = 0;
uint32_t g_rate_window_frames = 0;

// Per-pixel planar-Z -> Euclidean range scale at the 64x36 render grid.
// scale = sqrt(1 + xn^2 + yn^2), xn=(u-cx)/fx, yn=(v-cy)/fy.
const std::array<float, kRenderW * kRenderH>& euclid_scale() {
    static const std::array<float, kRenderW * kRenderH> kScale = [] {
        const float fx = 0.5f * kRenderW / std::tan(0.5f * kFovXDeg * kPi / 180.0f);
        const float fy = 0.5f * kRenderH / std::tan(0.5f * kFovYDeg * kPi / 180.0f);
        const float cx = 0.5f * kRenderW;
        const float cy = 0.5f * kRenderH;
        std::array<float, kRenderW * kRenderH> s{};
        for (int v = 0; v < kRenderH; ++v) {
            for (int u = 0; u < kRenderW; ++u) {
                const float xn = (static_cast<float>(u) - cx) / fx;
                const float yn = (static_cast<float>(v) - cy) / fy;
                s[static_cast<size_t>(v * kRenderW + u)] = std::sqrt(1.0f + xn * xn + yn * yn);
            }
        }
        return s;
    }();
    return kScale;
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

// linspace(0, n-1, out)[k] truncated to int -- matches numpy .astype(int) and
// depth_preprocess.hpp::resize_nearest_sample exactly.
static inline int resize_index(int k, int src, int out) {
    if (out <= 1) {
        return 0;
    }
    return static_cast<int>(static_cast<double>(k) * (src - 1) / static_cast<double>(out - 1));
}

// Native planar-Z (oh x ow, row-major) -> 9x16 perception grid.
// Replicates raw_planar_z_to_perception(): orient is applied by the caller
// (already reflected in oh/ow + the index map), then resize -> euclid -> pool.
static void planar_to_perception(const std::vector<float>& oriented, int oh, int ow,
                                 std::array<float, kGridFloats>& perception) {
    const auto& scale = euclid_scale();

    // Resize (nearest) to render grid, clean, and convert to Euclidean range.
    std::array<float, kRenderH * kRenderW> euclid{};
    for (int v = 0; v < kRenderH; ++v) {
        const int sy = resize_index(v, oh, kRenderH);
        for (int u = 0; u < kRenderW; ++u) {
            const int sx = resize_index(u, ow, kRenderW);
            float z = oriented[static_cast<size_t>(sy * ow + sx)];
            if (!std::isfinite(z) || z <= 1e-3f) {
                z = kPolicyMaxDist;  // no-return / invalid -> far (no obstacle)
            }
            euclid[static_cast<size_t>(v * kRenderW + u)] =
                z * scale[static_cast<size_t>(v * kRenderW + u)];
        }
    }

    // 4x4 min-pool to 9x16 (nearest obstacle surface per angular cell), encode.
    for (int oy = 0; oy < kOutH; ++oy) {
        for (int ox = 0; ox < kOutW; ++ox) {
            float cell_min = kPolicyMaxDist;
            for (int py = 0; py < kPool; ++py) {
                for (int px = 0; px < kPool; ++px) {
                    const int ry = oy * kPool + py;
                    const int rx = ox * kPool + px;
                    cell_min = std::min(cell_min, euclid[static_cast<size_t>(ry * kRenderW + rx)]);
                }
            }
            const float r = std::max(0.0f, std::min(cell_min, kPolicyMaxDist));
            perception[static_cast<size_t>(oy * kOutW + ox)] = 1.0f - r / kPolicyMaxDist;
        }
    }
}

// Build the oriented planar-Z image from a tof2 frame, then preprocess to 9x16.
static void build_perception(const tof2_data_t* tof, std::array<float, kGridFloats>& perception) {
    const int width = tof->width;
    const int height = tof->height;
    const int n = width * height;

    // Native planar-Z, row-major (height rows, width cols): planar[r*width+c].
    std::vector<float> planar(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        planar[static_cast<size_t>(i)] = tof->points[i][2];
    }

    // orient_raw_tof: Starling is portrait; rotate 90 deg CCW when stored wide.
    std::vector<float> oriented;
    int oh, ow;
    if (width > height) {
        oh = width;
        ow = height;
        oriented.resize(static_cast<size_t>(oh * ow));
        // np.rot90(k=1): out[i,j] = src[j, width-1-i]; src is (height,width).
        for (int i = 0; i < oh; ++i) {
            for (int j = 0; j < ow; ++j) {
                oriented[static_cast<size_t>(i * ow + j)] =
                    planar[static_cast<size_t>(j * width + (width - 1 - i))];
            }
        }
    } else {
        oh = height;
        ow = width;
        oriented = std::move(planar);
    }

    // Optional flips (off by default == exact match to the ground raw path).
    if (g_cfg.flip_v) {
        for (int r = 0; r < oh / 2; ++r) {
            for (int c = 0; c < ow; ++c) {
                std::swap(oriented[static_cast<size_t>(r * ow + c)],
                          oriented[static_cast<size_t>((oh - 1 - r) * ow + c)]);
            }
        }
    }
    if (g_cfg.flip_h) {
        for (int r = 0; r < oh; ++r) {
            for (int c = 0; c < ow / 2; ++c) {
                std::swap(oriented[static_cast<size_t>(r * ow + c)],
                          oriented[static_cast<size_t>(r * ow + (ow - 1 - c))]);
            }
        }
    }

    planar_to_perception(oriented, oh, ow, perception);
}

static void send_perception(const tof2_data_t* tof) {
    if (tof->width <= 0 || tof->height <= 0) {
        return;
    }

    std::array<float, kGridFloats> perception{};
    build_perception(tof, perception);

    std::array<char, kEncPacketSize> packet{};
    EncHeader hdr{};
    hdr.magic = kUdpMagicEnc;
    hdr.seq = g_seq++;
    hdr.timestamp_ns = static_cast<uint64_t>(tof->timestamp_ns);
    std::memcpy(packet.data(), &hdr, sizeof(hdr));
    std::memcpy(packet.data() + sizeof(hdr), perception.data(), kGridFloats * sizeof(float));

    const ssize_t sent = sendto(g_sock, packet.data(), packet.size(), 0,
                                reinterpret_cast<sockaddr*>(&g_dest), sizeof(g_dest));
    if (sent != static_cast<ssize_t>(packet.size())) {
        std::fprintf(stderr, "sendto failed (%zd)\n", sent);
    }

    log_stream_rate(static_cast<uint64_t>(tof->timestamp_ns));
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
            cfg->flip_h = true;
        } else if (std::strcmp(argv[i], "--flip-v") == 0) {
            cfg->flip_v = true;
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
        std::fprintf(stderr, "Usage: %s <ground_pc_ip> [port=5600] [--flip-h] [--flip-v]\n", argv[0]);
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

    std::printf(
        "Streaming DiffAero 9x16 perception to %s:%d over UDP (TOF2, %zu B/frame%s%s). "
        "Press Ctrl-C to stop.\n",
        host, port, kEncPacketSize,
        g_cfg.flip_h ? " +flip-h" : "", g_cfg.flip_v ? " +flip-v" : "");

    while (!g_stop.load()) {
        usleep(100000);
    }

    pipe_client_close_all();
    close(g_sock);
    std::printf("Closed.\n");
    return 0;
}
