# ToF Streamer

Reads ToF sensor data from MPA on the Starling 2 Max and (eventually) re-publishes it over MPA.

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
