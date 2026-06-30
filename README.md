# ToF Streamer

Reads ToF sensor data from MPA on the Starling 2 Max and streams it to the ground PC.

**Recommended for AirStack / DiffAero:** use the **UDP path** (`tof_udp_stream.cpp` →
`ground/tof_udp_bridge.py`). The drone runs DiffAero-aligned preprocessing
on-board (orient → **86° FOV crop** with M0178 intrinsics → euclid → linspace
min-pool to 9×16 → encode) and streams the pre-encoded **9×16 TOF2** grid over
UDP (~592 B/frame). The ground bridge decodes and republishes on domain 1 — Jazzy
never joins the VOXL's Foxy DDS domain (which crashes `voxl_mpa_to_ros2`).

The legacy ROS 2 path (sections 2–4) still works for drone-local debugging but
must not be subscribed from Jazzy on domain 0.

## Quick start — UDP (recommended)

### Finding `GROUND_IP`

The drone sends UDP to your **ground PC's Wi-Fi IP on the same network as the
VOXL** (not localhost, not a Docker bridge). The AirStack robot container uses
host networking, so use the **host** address.

On the **ground PC**:

```bash
export VOXL_IP=192.168.123.167   # your drone IP
ip -4 addr show                    # wlp* / wlan* → inet 192.168.x.x
# or auto-detect the source IP for traffic to the drone:
export GROUND_IP=$(ip -4 route get "$VOXL_IP" | awk '{print $7; exit}')
echo "GROUND_IP=$GROUND_IP"
```

Use `192.168.x.x` on the drone subnet — **not** `127.0.0.1`, `172.17.0.1`, or
`172.31.0.1`. From the drone: `ping -c 1 $GROUND_IP`.

### On the drone (one tmux session, no ROS)

```bash
export VOXL_IP=192.168.123.167   # your drone IP
export GROUND_IP=192.168.123.134 # from steps above

scp tof_udp_stream.cpp root@$VOXL_IP:/home/root/
ssh root@$VOXL_IP
```

On the drone:

```bash
# Confirm ToF server is up
systemctl status voxl-camera-server
voxl-inspect-cam tof_depth -n   # SDK 1.5: use inspect-cam, not voxl-inspect-pipe

cd /home/root
g++ -std=c++14 -O2 tof_udp_stream.cpp \
    -o tof_udp_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm

./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100
# Starling 2 Max default (bench-tuned; see "Tuning vertical crop" below).
# Default: +flip-v on 9x16 grid (pitch). --flip-h mirrors yaw — off by default.
# Pass --no-flip-v if vertical tilt looks inverted.
# Pass --stream-raw for optional TOF3 debug streaming.
```

### On the ground (Jazzy, domain 1)

Inside the AirStack robot container or `conda activate ros2`:

```bash
cd ~/tof_streamer
ROS_DOMAIN_ID=1 python3 ground/tof_udp_bridge.py --port 5600 \
  --topic /drone_1/perception/tof
```

Verify:

```bash
ros2 topic hz /drone_1/perception/tof    # ~10–30 Hz
ros2 topic echo /drone_1/perception/tof --once   # 144 floats, 9×16
```

No `domain_bridge`, no Foxy on the ground, no `ROS_DOMAIN_ID=0` probing.

### Tuning vertical crop (ceiling in frame)

Default crop is **center** (86° window centered on the optical axis). If the **ceiling**
dominates the top of the raw ToF view, use a **bottom** anchor to keep the lower
86° of vertical FOV (forward / ground) and discard the top.

**Recommended: tune on the ground first** (no drone recompile per try):

```bash
# VOXL — raw TOF3 only
./tof_udp_stream $GROUND_IP 5600 --raw-only

# Ground — raw topic only, ignore onboard TOF2
python3 ground/tof_udp_bridge.py --port 5600 \
  --raw-topic /drone_1/perception/tof_raw --ignore-tof2

# Ground — live crop preview + publishes /drone_1/perception/tof
python3 ground/tof_crop_tune.py \
  --crop-v-anchor bottom --crop-v-shift 0
```

Open RViz on **`/svg/drone_1/tof_crop_debug`**: left = raw with **green crop box**,
right = upscaled 9×16. Restart `tof_crop_tune.py` with different flags:

| Flag | Effect |
|------|--------|
| `--crop-v-anchor bottom` | Keep bottom of frame (drop ceiling first) |
| `--crop-v-anchor top` | Keep top (rarely useful) |
| `--crop-v-shift -100` | AirStack / DiffAero default (Starling 2 Max) |
| `--crop-v-shift 20` | Shift crop down 20 px (discard more ceiling) |
| `--crop-v-shift -N` | Shift crop up N px (less ceiling discarded) |

When satisfied, deploy on the VOXL (AirStack / DiffAero default for Starling 2 Max):

```bash
./tof_udp_stream $GROUND_IP 5600 --crop-v-anchor=bottom --crop-v-shift=-100
```

Or use the same crop flags on `tof_udp_bridge.py --preprocess-tof3` for ground-side preprocessing.

---

## 1. Test that `tof_stream.cpp` still runs on the drone

Set the drone's IP once:

```bash
export VOXL_IP=192.168.8.1   # replace with your drone's actual IP
```

### Copy the file to the drone

```bash
scp tof_stream.cpp root@$VOXL_IP:/home/root/
```

### SSH in and compile it

```bash
ssh root@$VOXL_IP
```

Then on the drone:

```bash
cd /home/root
g++ -std=c++14 -O2 tof_stream.cpp \
    -o tof_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm
```

### Confirm the `tof` pipe exists before running

```bash
ls /run/mpa/tof/    # or /dev/mpa/tof depending on VOXL version
voxl-list-pipes | grep -i tof
```

### Run it

```bash
timeout 5s ./tof_stream
```

You should see output like:

```
Listening on tof pipe. Press Ctrl-C to stop.
frame 0  ts=...  dims=...x...
frame 1  ts=...  dims=...x...
```

If `pipe_client_open(tof) failed`, the ToF server isn't publishing — check with:

```bash
voxl-inspect-pipe tof
```

or make sure the ToF service is running:

```bash
systemctl status voxl-tof-server   # name may differ; check `systemctl list-units | grep tof`
```

## 2. Bring up `/tof_depth` over ROS2 (bridge)

Before running a node that processes and republishes ToF data, the
`voxl_mpa_to_ros2` bridge must be running on the drone — it mirrors the raw
`tof` MPA pipe into a ROS2 topic (`/tof_depth`) that any ROS2 node (on the
drone or, over the network, on your Mac) can subscribe to directly.

### Confirm the ToF server is up first

Same check as step 1 — the bridge has nothing to mirror if this isn't running:

```bash
ssh root@$VOXL_IP
systemctl status voxl-tof-server   # name may differ; check `systemctl list-units | grep tof`
voxl-inspect-pipe tof
```

### Start the bridge in a tmux session (so it survives detaching)

```bash
ssh root@$VOXL_IP
tmux new -s ros2
```

Inside the tmux session:

```bash
set +u
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 HOME=/root
export COLCON_TRACE="" COLCON_PYTHON_EXECUTABLE=/usr/bin/python3
source /opt/ros/foxy/setup.bash
source /opt/ros/foxy/mpa_to_ros2/install/setup.bash
ros2 run voxl_mpa_to_ros2 voxl_mpa_to_ros2_node
```

Leave this running and detach with `Ctrl+B` then `D`.

### Confirm `/tof_depth` is actually publishing

From a second SSH session to the drone (same env vars as above, minus the
`ros2 run` line):

```bash
ssh root@$VOXL_IP
export ROS_DOMAIN_ID=0
source /opt/ros/foxy/setup.bash
ros2 topic list | grep tof_depth
ros2 topic hz /tof_depth   # expect ~10 Hz; Ctrl+C to stop
```

Once `/tof_depth` is live and publishing at a steady rate, it's safe to start
the processing node (`drone/tof_processor_node.py`) on the drone in its own
tmux pane — it subscribes to `/tof_depth`, does its processing, and publishes
a custom topic that your Mac node subscribes to over the same `ROS_DOMAIN_ID`.

---

## 3. Run the Perception Node

This is the node that subscribes to `/tof_depth` and processes each frame
(`drone/perception_node.py`, using `drone/depth_preprocess.py`). It must run
**after** the bridge from step 2 is already publishing `/tof_depth`.

### Copy the files to the drone

```bash
scp drone/perception_node.py drone/depth_preprocess.py drone/topics.py root@$VOXL_IP:/home/root/
```

### SSH in, source ROS2, and run it

In a new tmux pane (so the bridge from step 2 keeps running alongside it):

```bash
ssh root@$VOXL_IP
tmux new -s perception
```

Inside the tmux session:

```bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 HOME=/root
source /opt/ros/foxy/setup.bash
cd /home/root
python3 perception_node.py
```

You should see a stream of printed 9x16 perception arrays, one per `/tof_depth`
frame. Detach with `Ctrl+B` then `D` to leave it running.

If nothing prints, confirm `/tof_depth` is still publishing (`ros2 topic hz
/tof_depth` from step 2) — the node depends entirely on the bridge being up
first.

### One-liner (scp + run in one go)

From your dev machine:

```bash
scp drone/perception_node.py drone/depth_preprocess.py drone/topics.py root@$VOXL_IP:/home/root/ && \
ssh root@$VOXL_IP "export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 HOME=/root && \
  source /opt/ros/foxy/setup.bash && \
  cd /home/root && python3 perception_node.py"
```

---

## 4. Read the perception array on your Mac

`perception_node.py` publishes the 9x16 perception array on
`/tof_streamer/perception` (`Float32MultiArray`). `mac/perception_viewer.py`
subscribes to that topic and reshapes it back into a numpy array — no
custom networking required, since this is plain ROS2/DDS over your WiFi link
(same mechanism `/diffaero/*` uses in `tof_integration`).

### Requirements

- Mac is on the same network as the drone (its ROS2 Foxy bridge environment,
  e.g. `conda activate ros2` if that's how you have ROS2 set up on the Mac).
- `ROS_DOMAIN_ID` matches what you exported on the drone (`0` in the examples
  above).

### Run it

```bash
conda activate ros2   # or however you activate your ROS2 Foxy env on Mac
export ROS_DOMAIN_ID=0
cd "tof_streamer/mac"
python3 perception_viewer.py
```

With the bridge (step 2) and the perception node (step 3) both running on the
drone, you should see the same 9x16 arrays start printing here, in real time.

If nothing arrives, sanity-check end to end:

```bash
ros2 topic list | grep tof_streamer        # topic should be visible from the Mac too
ros2 topic hz /tof_streamer/perception     # confirm it's actually publishing
```

If the topic isn't visible at all, it's almost always `ROS_DOMAIN_ID`
mismatch or the Mac/drone not actually being on the same network/subnet for
DDS multicast discovery to work.

---

## One-liner (scp + compile + run in one go)

From your dev machine:

```bash
scp tof_stream.cpp root@$VOXL_IP:/home/root/ && \
ssh root@$VOXL_IP "cd /home/root && \
  g++ -std=c++14 -O2 tof_stream.cpp -o tof_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm && \
  timeout 5s ./tof_stream"
```
