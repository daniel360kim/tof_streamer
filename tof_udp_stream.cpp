// Stream raw ToF planar-Z over UDP (no ROS, no preprocessing on the drone).
//
// Reads the VOXL "tof" MPA pipe (tof2_data_t), extracts points[][2] (forward
// depth in meters), and sends chunked UDP packets to the ground PC. Ground side:
// ground/tof_udp_bridge.py downsamples/encodes to DiffAero 9x16 perception.
//
// Compile on VOXL:
//   g++ -std=c++14 -O2 tof_udp_stream.cpp \
//       -o tof_udp_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm
//
// Run:
//   ./tof_udp_stream <ground_pc_ip> [port=5600]

#include <modal_pipe_client.h>
#include <modal_pipe_interfaces.h>

#include <arpa/inet.h>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr uint32_t kUdpMagicRaw = 0x544F4633u;  // 'TOF3'
constexpr size_t kMaxChunkPayload = 64000;

struct RawChunkHeader {
    uint32_t magic;
    uint32_t seq;
    uint32_t frame_id;
    uint16_t chunk_index;
    uint16_t chunk_count;
    uint64_t timestamp_ns;
    uint16_t width;
    uint16_t height;
} __attribute__((packed));

std::atomic<bool> g_stop{false};
int g_sock = -1;
sockaddr_in g_dest{};
uint32_t g_seq = 0;
uint32_t g_frame_id = 0;
uint64_t g_rate_window_start_ns = 0;
uint32_t g_rate_window_frames = 0;

static void log_stream_rate(uint64_t timestamp_ns, int width, int height) {
    if (g_rate_window_start_ns == 0) {
        g_rate_window_start_ns = timestamp_ns;
    }
    g_rate_window_frames++;
    const double elapsed_s = static_cast<double>(timestamp_ns - g_rate_window_start_ns) * 1e-9;
    if (elapsed_s < 1.0) {
        return;
    }
    const double hz = static_cast<double>(g_rate_window_frames) / elapsed_s;
    std::printf("streaming %.1f Hz (%dx%d raw Z)\n", hz, width, height);
    g_rate_window_start_ns = timestamp_ns;
    g_rate_window_frames = 0;
}

static void signal_handler(int) {
    g_stop.store(true);
}

static void send_raw_frame(const tof2_data_t* tof) {
    const int width = tof->width;
    const int height = tof->height;
    if (width <= 0 || height <= 0) {
        return;
    }

    const int n = width * height;
    std::vector<float> z(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) {
        z[static_cast<size_t>(i)] = tof->points[i][2];
    }

    const size_t total_bytes = static_cast<size_t>(n) * sizeof(float);
    const uint16_t chunk_count = static_cast<uint16_t>(
        (total_bytes + kMaxChunkPayload - 1) / kMaxChunkPayload);
    const uint32_t frame_id = g_frame_id++;

    std::vector<char> packet(sizeof(RawChunkHeader) + kMaxChunkPayload);

    for (uint16_t chunk_index = 0; chunk_index < chunk_count; ++chunk_index) {
        const size_t byte_offset = static_cast<size_t>(chunk_index) * kMaxChunkPayload;
        const size_t chunk_bytes = std::min(kMaxChunkPayload, total_bytes - byte_offset);

        RawChunkHeader hdr{};
        hdr.magic = kUdpMagicRaw;
        hdr.seq = g_seq++;
        hdr.frame_id = frame_id;
        hdr.chunk_index = chunk_index;
        hdr.chunk_count = chunk_count;
        hdr.timestamp_ns = static_cast<uint64_t>(tof->timestamp_ns);
        hdr.width = static_cast<uint16_t>(width);
        hdr.height = static_cast<uint16_t>(height);

        std::memcpy(packet.data(), &hdr, sizeof(hdr));
        std::memcpy(
            packet.data() + sizeof(hdr),
            reinterpret_cast<const char*>(z.data()) + byte_offset,
            chunk_bytes);

        const size_t packet_size = sizeof(hdr) + chunk_bytes;
        const ssize_t sent = sendto(
            g_sock, packet.data(), packet_size, 0,
            reinterpret_cast<sockaddr*>(&g_dest), sizeof(g_dest));
        if (sent != static_cast<ssize_t>(packet_size)) {
            std::fprintf(stderr, "sendto failed (%zd)\n", sent);
        }
    }

    log_stream_rate(static_cast<uint64_t>(tof->timestamp_ns), width, height);
}

static void tof_callback(int ch, char* data, int bytes, void* context) {
    (void)ch;
    (void)context;

    if (bytes < static_cast<int>(sizeof(tof2_data_t))) {
        return;
    }

    const tof2_data_t* tof = reinterpret_cast<const tof2_data_t*>(data);
    send_raw_frame(tof);
}

static bool parse_args(int argc, char** argv, const char** host, int* port) {
    if (argc < 2) {
        return false;
    }
    *host = argv[1];
    *port = (argc >= 3) ? std::atoi(argv[2]) : 5600;
    return *port > 0 && *port <= 65535;
}

}  // namespace

int main(int argc, char** argv) {
    const char* host = nullptr;
    int port = 5600;
    if (!parse_args(argc, argv, &host, &port)) {
        std::fprintf(stderr, "Usage: %s <ground_pc_ip> [port=5600]\n", argv[0]);
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
        "Streaming raw ToF planar-Z to %s:%d over UDP (chunked). Press Ctrl-C to stop.\n",
        host, port);

    while (!g_stop.load()) {
        usleep(100000);
    }

    pipe_client_close_all();
    close(g_sock);
    std::printf("Closed.\n");
    return 0;
}
