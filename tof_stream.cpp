
// Compile on VOXL:
//   g++ -std=c++14 -O2 ex1_hello_pipe.cpp \
//       -o ex1_hello_pipe /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm
//
// Run: timeout 5s ./ex1_hello_pipe

#include <modal_pipe_client.h>
#include <modal_pipe_interfaces.h>

#include <atomic>
#include <csignal>
#include <cstdio>
#include <unistd.h>

static std::atomic<bool> g_stop{false};

static void signal_handler(int) {
    g_stop.store(true);
}

// Every MPA data callback has this exact signature.
// ch     — the channel id you assigned in pipe_client_open
// data   — raw bytes from the pipe
// bytes  — how many bytes arrived
// context — the void* you registered; null here
static void tof_callback(int ch, char* data, int bytes, void* context) {
    (void)ch;
    (void)context;

    // Guard against short reads before casting.
    if (bytes < (int)sizeof(tof2_data_t)) {
        return;
    }
    const tof2_data_t* tof = reinterpret_cast<const tof2_data_t*>(data);

    static int frame_count = 0;
    std::printf("frame %d  ts=%llu  dims=%dx%d\n",
                frame_count++,
                (unsigned long long)tof->timestamp_ns,
                tof->width, tof->height);
    std::fflush(stdout);
}

int main() {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Open channel 0 on pipe named "tof".
    // EN_PIPE_CLIENT_SIMPLE_HELPER sets up a helper thread that calls your
    // callback whenever data arrives — you don't have to read() yourself.
    const int rc = pipe_client_open(
        0,                                // channel id (your choice, 0-based)
        "tof",                            // pipe name (matches /dev/mpa/tof)
        "",                               // client name string
        CLIENT_FLAG_EN_SIMPLE_HELPER,     // spawn background helper thread
        sizeof(tof2_data_t)               // read buffer size
    );
    if (rc != 0) {
        std::fprintf(stderr, "pipe_client_open(tof) failed: %d\n", rc);
        return 1;
    }

    // Register the data callback AFTER opening.
    // context = nullptr because we don't need to share state yet.
    pipe_client_set_simple_helper_cb(0, tof_callback, nullptr);

    std::printf("Listening on tof pipe. Press Ctrl-C to stop.\n");

    while (!g_stop.load()) {
        usleep(100000);  // 100 ms — just keeping main alive
    }

    pipe_client_close_all();
    std::printf("Closed.\n");
    return 0;
}
