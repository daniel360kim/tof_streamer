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

## One-liner (scp + compile + run in one go)

From your dev machine:

```bash
scp tof_stream.cpp root@$VOXL_IP:/home/root/ && \
ssh root@$VOXL_IP "cd /home/root && \
  g++ -std=c++14 -O2 tof_stream.cpp -o tof_stream /usr/lib64/libmodal_pipe.so -lpthread -lrt -lm && \
  timeout 5s ./tof_stream"
```
