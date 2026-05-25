# benchmark_rt — Go2 Pro Real-Time Benchmark Suite (C++)

A collection of hardware-validated real-time benchmark tools for the Unitree Go2 Pro quadruped robot. All tools communicate over DDS via `unitree_sdk2` and measure latency, jitter, packet loss, and IMU noise directly on the physical hardware.

## Tools

| Binary | What it measures |
|---|---|
| `latency_v3_fix` | Frame-aligned round-trip latency (LowCmd → LowState.tau_est) |
| `latency_intrp_v4` | Sub-frame interpolated round-trip latency |
| `encoder_jitter` | Per-joint encoder arrival timing jitter |
| `packet_loss_test` | DDS packet loss rate over a configurable duration |
| `pulse_sweep` | Torque pulse-width sweep to characterise actuator response |
| `imu_logger` | Raw IMU data logger for offline noise analysis |
| `go2_eskf_logger` | Simultaneous joints + IMU + contact logger for ESKF development |

## Prerequisites

- Ubuntu 22.04, Linux real-time kernel recommended
- C++17 compiler (GCC ≥ 11 or Clang ≥ 14)
- [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2) installed system-wide
- Robot connected in **low-level mode** on a known network interface (e.g. `eth0`)

### Install unitree_sdk2

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2 && mkdir -p build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

## Build

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

Binaries are placed in `build/`.

## Usage

All tools take the network interface as the first positional argument.

```bash
# Frame-aligned DDS latency (default: 0.5 Nm pulse, 300 samples)
./build/latency_v3_fix eth0 B [--tau <Nm>] [--n <samples>]

# Sub-frame interpolated latency
./build/latency_intrp_v4 eth0 B

# Encoder / arrival jitter
./build/encoder_jitter eth0 [duration_s]

# Packet loss (default 30 min)
./build/packet_loss_test eth0 [duration_s]

# Torque pulse-width sweep
./build/pulse_sweep eth0

# IMU data logger
./build/imu_logger eth0 [duration_s] [output.csv]

# Full state logger (joints + IMU + contact)
./build/go2_eskf_logger eth0 [duration_s] [output_prefix]
```

Output is written as a JSON summary + CSV raw samples after collection ends (no disk I/O inside DDS callbacks).

## Real-time setup

For lowest-jitter measurements, run as root or grant capabilities and use a PREEMPT_RT kernel:

```bash
sudo setcap cap_sys_nice,cap_ipc_lock+ep build/latency_v3_fix
# or simply: sudo ./build/latency_v3_fix eth0 B
```

See [`include/rt_utils.hpp`](include/rt_utils.hpp) for `mlockall`, `SCHED_FIFO`, CPU pinning, and PI-mutex helpers used across all tools.

## Results

The `results/` directory contains reference measurements collected on a Go2 Pro at firmware 1.0.x. See [`docs/benchmark_decisions.md`](docs/benchmark_decisions.md) for methodology notes and design decisions.

## Project structure

```
benchmark_rt/
├── CMakeLists.txt
├── include/          # Shared headers (stats, rt_utils, crc32)
├── src/              # One .cpp per benchmark tool
├── results/          # Reference measurement data (CSV + JSON)
└── docs/             # Methodology and design notes
```

## License

MIT
