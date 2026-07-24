# PID Tuning Assistant

Read-only PX4 telemetry recorder and PID tuning advisor.
Connects to PX4 via uXRCE-DDS, records flight data, detects step-response events,
and produces heuristic PID recommendations + ML-ready exports.

> **SAFETY DISCLAIMER**
> This tool **never** writes to the drone, arms motors, or changes parameters.
> All recommendations are **heuristic starting points** — not final values.
> Always validate changes in SITL (Gazebo) before flying real hardware.
> Change **one parameter at a time** and re-fly to verify.

---

## Table of Contents

1. [Architecture overview](#architecture-overview)
2. [Two input source modes](#two-input-source-modes)
3. [Prerequisites](#prerequisites)
4. [Step 0 — Verify px4_msgs branch matches firmware](#step-0--verify-px4_msgs-branch-matches-firmware)
5. [Step 1 — Add internal setpoint topics to dds_topics.yaml](#step-1--add-internal-setpoint-topics-to-dds_topicsyaml)
6. [Step 2 — Install the package](#step-2--install-the-package)
7. [Step 3 — Start the uXRCE-DDS Agent](#step-3--start-the-uxrce-dds-agent)
8. [Step 4 — Launch the tool](#step-4--launch-the-tool)
9. [Step 5 — Test flight procedure (DDS mode)](#step-5--test-flight-procedure-dds-mode)
10. [ULog mode — offline analysis](#ulog-mode--offline-analysis)
11. [Vehicle profiles](#vehicle-profiles)
12. [Using the Web UI](#using-the-web-ui)
13. [Understanding the analysis](#understanding-the-analysis)
14. [Export formats](#export-formats)
15. [Offline / SITL mode](#offline--sitl-mode)
16. [Troubleshooting](#troubleshooting)

---

## Two input source modes

The tool supports **two independent, non-conflicting input modes**:

| Mode | When to use | What you get |
|---|---|---|
| **DDS (live)** | Drone connected via uXRCE-DDS | Real-time recording, step detection, heuristic recommendations |
| **ULog (offline)** | `.ulg` file from SD card or QGC download | Everything above + high-rate FRF, stability margins, vibration analysis, plant recovery, PID synthesis |

Both modes produce the same Analysis tab output (recommendations, metrics, export).
ULog additionally unlocks the **HiFi Analysis** section with frequency-domain results.

**DDS-only installs do not need pyulog** — the dependency is optional.

---

## Architecture overview

```
PX4 firmware
  └─ uXRCE-DDS client (TELEM2, 921600 baud)
        │ UDP/Serial
  uXRCE-DDS Agent  (companion computer)
        │ ROS 2 topics
  TelemetryNode (rclpy, background thread)
        │ thread-safe RecordingBuffer (deque)
  FastAPI server  ──WS──►  Browser (Plotly live charts)
        │
        └─► PIDAnalyzer (numpy/scipy, no ROS dep)
               │
               ├─► step detection per axis × mode
               ├─► metrics: overshoot, rise time, settling, FFT freq, noise
               ├─► recommendations engine (heuristic rules)
               └─► exports: JSON / CSV / Parquet / llm_prompt.txt
```

**Topics recorded** (all `/fmu/out/`, px4_msgs):

| Topic | Message | Notes |
|---|---|---|
| `vehicle_angular_velocity` | `VehicleAngularVelocity` | Rate measurements |
| `vehicle_attitude` | `VehicleAttitude` | Quaternion → Euler |
| `vehicle_status` | `VehicleStatus` | nav_state (flight mode) |
| `battery_status` | `BatteryStatus` | Voltage, remaining |
| `vehicle_local_position` | `VehicleLocalPosition` | Drift context |
| `actuator_motors` | `ActuatorMotors` | Motor commands |
| `vehicle_rates_setpoint` | `VehicleRatesSetpoint` | **[NEEDS dds_topics.yaml]** |
| `vehicle_attitude_setpoint` | `VehicleAttitudeSetpoint` | **[NEEDS dds_topics.yaml]** |
| `rate_ctrl_status` | `RateCtrlStatus` | **[NEEDS dds_topics.yaml]** |
| `vehicle_torque_setpoint` | `VehicleTorqueSetpoint` | **[NEEDS dds_topics.yaml]** |

---

## Prerequisites

- ROS 2 Humble (Ubuntu 22.04)
- Python 3.10+
- `px4_msgs` built from the branch matching your PX4 firmware (see Step 0)
- uXRCE-DDS Agent installed on companion computer
- Python packages: `fastapi uvicorn[standard] websockets numpy scipy pandas pyarrow python-multipart`

```bash
# DDS mode (minimum):
pip install fastapi "uvicorn[standard]" websockets numpy scipy pandas pyarrow python-multipart pyyaml

# ULog mode (adds pyulog):
pip install "pid_tuning_assistant[ulog]"
# or manually:
pip install pyulog
```

### Dependency provenance — rosdep vs pip

The dependencies split into two groups, which matters when installing on a fresh
companion or in CI:

| Provided by | Packages | How installed |
|---|---|---|
| **rosdep** (declared in `package.xml`) | `python3-numpy`, `python3-scipy`, `python3-pandas`, `python3-yaml`, plus `rclpy` / `px4_msgs` / `offboard_interfaces` / `sverk_interfaces` | `rosdep install --from-paths src --ignore-src -r -y` |
| **pip only** (declared in `setup.py`, *not* `package.xml`) | `fastapi`, `uvicorn[standard]`, **`python-multipart`** (required — FastAPI cannot parse `UploadFile` uploads without it), `pyarrow`, and the optional `pyulog` | the `pip install …` line above, or `pip install .[ulog]` |

The web-layer packages are **deliberately absent from `package.xml`**: they have
no reliable rosdep key on ROS 2 Humble, so declaring them would break
`rosdep install`. Install them with pip. `python-multipart` is the easy one to
miss — without it `/api/params` and `/api/ulog/upload` return an error even
though nothing looks wrong at import time.

---

## Step 0 — Verify px4_msgs branch matches firmware

px4_msgs messages change between PX4 releases. A mismatch causes silent
deserialization errors or subscription failures.

```bash
# 1. Check your PX4 firmware version (on PX4 console or QGC)
#    Example: PX4 v1.15.0

# 2. Find the matching px4_msgs branch:
#    https://github.com/PX4/px4_msgs/branches
#    Branch naming: "release/1.15"

# 3. Clone the correct branch into your workspace:
cd ~/your_ws/src
git clone --branch release/1.15 https://github.com/PX4/px4_msgs.git

# 4. Build
cd ~/your_ws
colcon build --packages-select px4_msgs
source install/setup.bash

# 5. Verify a message exists:
ros2 interface show px4_msgs/msg/VehicleAngularVelocity
# Should print field definitions, NOT "Unknown message type"
```

---

## Step 1 — Add internal setpoint topics to dds_topics.yaml

By default, PX4 does **not** publish rate setpoints, attitude setpoints,
rate controller status, or torque setpoints over DDS.
Without them, step-response comparison is impossible (only measurements recorded).

**Edit `src/modules/uxrce_dds_client/dds_topics.yaml` in your PX4 source:**

```yaml
publications:
  # ... existing entries ...

  # ADD THESE:
  - topic: /fmu/out/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/out/vehicle_attitude_setpoint
    type: px4_msgs::msg::VehicleAttitudeSetpoint

  - topic: /fmu/out/rate_ctrl_status
    type: px4_msgs::msg::RateCtrlStatus

  - topic: /fmu/out/vehicle_torque_setpoint
    type: px4_msgs::msg::VehicleTorqueSetpoint
```

**Then rebuild the PX4 firmware and flash:**

```bash
# For Pixhawk 6C / FMUv6c:
make px4_fmu-v6c_default

# For Pixhawk 4 / FMUv5:
make px4_fmu-v5_default

# Flash (with PX4 connected via USB):
make px4_fmu-v6c_default upload
```

> **Note:** If you cannot rebuild firmware (e.g. using a pre-built binary),
> the tool will still work but will only record measurements, not setpoints.
> Step-response analysis will be skipped with a warning in the UI.
> The tool will **not crash**.

> **UART bandwidth budget.** In the sverk stack the PID block of DDS topics in
> `fmu/dds_topics.yaml` is enabled by the `fmu/matek_h743-slim_pid_on.px4`
> firmware. Over a **1.5 Mbaud UART** link, publishing all **6 rate-loop topics**
> (`vehicle_angular_velocity`, `vehicle_rates_setpoint`, `vehicle_torque_setpoint`,
> `vehicle_attitude`, `vehicle_attitude_setpoint`, `rate_ctrl_status`) at full
> rate (~400 Hz) sits at the edge of the link's throughput. If the **Data Quality**
> panel starts reporting drops/jitter, either keep only the topics you actually
> need (trim `dds_topics.yaml`) or move the uXRCE-DDS agent to a **UDP/Ethernet**
> transport, which has ample headroom.

---

## Step 2 — Install the package

```bash
cd ~/your_ws
# Copy or symlink the package:
cp -r /path/to/pid_tuning_assistant src/

colcon build --packages-select pid_tuning_assistant
source install/setup.bash
```

---

## Step 3 — Start the uXRCE-DDS Agent

On the companion computer (e.g. Orange Pi Zero 3W):

```bash
# Serial connection (TELEM2, typically /dev/ttyS1 or /dev/ttyAMA0):
MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600

# OR UDP (if PX4 client configured for UDP):
MicroXRCEAgent udp4 -p 8888
```

Verify DDS bridge is working:

```bash
# On ROS 2 side — should see PX4 topics:
ros2 topic list | grep fmu
ros2 topic hz /fmu/out/vehicle_angular_velocity
# Expected: ~250 Hz
```

---

## Step 4 — Launch the tool

```bash
# Default: web UI on port 8086  (8080 is used by web_video_server)
ros2 run pid_tuning_assistant pid_tuning_assistant

# Custom port:
ros2 run pid_tuning_assistant pid_tuning_assistant --port 9090

# Offline mode (no ROS, for replaying saved CSV/JSON):
ros2 run pid_tuning_assistant pid_tuning_assistant --no-ros
```

Open browser: **http://\<companion-ip\>:8086**

(From GCS laptop on same network: `http://192.168.1.x:8086`)

> **Security.** The server binds `0.0.0.0` and, by default, has **no
> authentication** — any host on the network can drive recording, upload files,
> and read telemetry. On any shared/untrusted network set an API token:
>
> ```bash
> export PID_ASSIST_TOKEN="$(openssl rand -hex 16)"   # or auth_token in config/params.yaml
> ros2 run pid_tuning_assistant pid_tuning_assistant
> ```
>
> When a token is set, every `/api/*` request needs
> `Authorization: Bearer <token>` (the UI stores it in a sidebar **API token**
> field). Without a token the tool starts, but logs a warning.

---

## Step 5 — Test flight procedure (DDS mode)

> **Always test in SITL (Gazebo) first.** See [Offline / SITL mode](#offline--sitl-mode).

### Pre-flight checklist

- [ ] uXRCE-DDS Agent running, topics visible (`ros2 topic list | grep fmu`)
- [ ] Tool running, browser open at `http://<ip>:8086`
- [ ] Live charts showing data (roll_rate, pitch_rate updating)
- [ ] Flight mode indicator in header showing current mode
- [ ] Battery indicator updating
- [ ] (Optional) Load `.params` file exported from QGC → **Load .params** button

### Recommended single-flight test sequence

The goal: collect step-response data in all three modes **in one continuous recording**.

```
1. ARM the drone (RC, not via tool)
2. In Web UI → enter run name, e.g. "flight_001_maiden"
3. Click [▶ Start] in Web UI
4. TAKEOFF to ~2 m, hover in STABILIZED mode

── STABILIZED (mode 15 / STAB) ──────────────────────────────────────
5. With RC right stick: give 3–5 deliberate PITCH step inputs
   (quick jab forward ~30%, hold 0.5 s, return to center)
   Wait 2 s between inputs for settling
6. Repeat for ROLL axis (left stick left/right jabs)
7. Repeat for YAW (left stick yaw, 3 jabs)

── ALTITUDE hold (mode 1 / ALTCTL) ──────────────────────────────────
8. Switch to Altitude mode
9. Repeat pitch/roll step inputs (3 each)
   Note: yaw control same as Stabilized

── POSITION hold (mode 2 / POSCTL) ──────────────────────────────────
10. Switch to Position mode
11. Repeat pitch/roll step inputs (3 each)
    Note: in Position, stick commands position velocity, not rate directly

── Stop and land ─────────────────────────────────────────────────────
12. Click [■ Stop] in Web UI  ← analysis runs automatically
13. Land, disarm
14. Switch to Analysis tab in browser
15. Review recommendations table
16. Download llm_prompt.txt and paste into an LLM for second opinion
```

### Tips for good step data

- **Stick jabs, not smooth sweeps** — the analyzer detects step *discontinuities*
  in the setpoint signal. Smooth ramps won't trigger step detection.
- **Let the drone settle** 1–2 s between inputs. Overlapping transients corrupt metrics.
- **Fly in calm conditions** for the first tune. Wind adds disturbance that inflates
  I-term recommendations.
- **Minimum 3 events per axis per mode** for statistically meaningful averages.

---

## ULog mode — offline analysis

ULog files (`.ulg`) are produced by PX4 on the SD card during every flight.
Download them via QGC (Analyze → Log Download) or directly from the SD card.

### Install pyulog

```bash
pip install pyulog
# or via the package extra:
pip install "pid_tuning_assistant[ulog]"
```

### Upload in the Web UI

1. Open the tool in browser.
2. In the **Source** selector (top of sidebar), choose **ULog file**.
3. Click **Browse**, select your `.ulg` file.
4. Click **Upload & Analyse**.

Analysis runs server-side (a few seconds for a typical 5-minute log).
The UI switches to the Analysis tab automatically.

### Required: set `SDLOG_PROFILE` before recording

The **default `SDLOG_PROFILE=1` logs the rate loop at only 50 Hz** (20 ms
intervals on `vehicle_angular_velocity`, `vehicle_rates_setpoint`,
`vehicle_torque_setpoint`). At that rate the HiFi frequency-domain analysis is
gated off (`MIN_RATE_FOR_FRF_HZ=100`) and will correctly refuse to run.

Enable the high-rate bit **before the flight you intend to tune** (requires an
FMU reboot to take effect):

```
# default(1) | high-rate(16) → 17.  Reboot the FMU afterwards.
param set SDLOG_PROFILE 17
```

Actual logged rates once high-rate is enabled:

| Topic | Rate | Set by |
|---|---|---|
| `vehicle_angular_velocity` | ~400 Hz | `IMU_GYRO_RATEMAX` |
| `vehicle_torque_setpoint`  | ~400 Hz | `IMU_GYRO_RATEMAX` |
| `vehicle_rates_setpoint`   | ~100 Hz (outside ACRO) | attitude loop |

### What ULog unlocks

With high-rate logging enabled, ULog provides the high-rate rate-loop signals,
raw gyro, ESC telemetry, filter diagnostics, and autotune results that are not
available over DDS at runtime.

| Feature | DDS | ULog |
|---|---|---|
| Step detection + heuristic recommendations | yes | yes |
| Rate / jitter / data quality stats | yes | yes |
| Frequency Response Function (FRF) | no | yes (if chirp/sweep excitation) |
| Stability margins (GM / PM) | no | yes |
| Loop delay estimate | no | yes |
| Vibration / resonance analysis | no | yes (500 Hz gyro) |
| Motor harmonic correlation | no | yes (ESC RPM) |
| Raw vs filtered gyro characterization | no | yes |
| ARX system identification (VAF) | no | yes |
| Plant recovery G(jw) | no | yes |
| PID synthesis (loop-shaping) | no | yes |
| Before / after step simulation | no | yes |
| Auto-populate PID gains from log | no | yes |

### Sweep maneuver for best FRF quality

Step inputs give good time-domain metrics but poor frequency coverage.
For the highest-quality FRF (and thus plant recovery + PID synthesis),
fly a **frequency sweep** maneuver on the roll or pitch axis:

```
1. Hover in STABILIZED mode at safe altitude (>5 m)
2. With roll stick: apply sinusoidal inputs that slowly increase in frequency
   — start at ~0.5 Hz, ramp to ~20 Hz over 20–30 seconds
   — amplitude: ~15–20% stick (small deflections, stay in linear range)
3. Release stick, let drone settle 3 s
4. Repeat for pitch axis
5. Land, download .ulg
```

> **Tip:** excitation quality is shown in the Capability badges card.
> "sweep / good" means FRF and plant recovery are reliable.
> "step / marginal" means margins and plant are approximate.

### VIO / EV tab — external-vision fuse tuning (ULog only)

A dedicated **VIO / EV** section (below the HiFi cards) analyses how EKF2 fuses
**external vision**, automating what is otherwise done by hand with `listener`
in the MAVLink console. It is **source-agnostic**: whatever EKF2 fuses as EV —
ArUco/VPE, 3D-lidar odometry (LIO), 2D-lidar scan-match, camera VIO, mocap —
lands in the same PX4 EV topics and is analysed identically. The tab reads
`estimator_status_flags.cs_ev_*`, `estimator_innovations.ev_*`,
`estimator_ev_pos_bias` and (when present) `estimator_aid_src_ev_*` /
`vehicle_visual_odometry` — never any specific ArUco topic.

It reports, each with an explicit confidence:

- **Test-ratios per axis** (`ev_hpos`, `ev_vpos`, EV-yaw via the shared
  `heading` field, and `ev_hvel/ev_vvel` when EV velocity is fused): mean, p95,
  and **% of samples > 1.0** (rejected by the innovation gate).
- **Innovation bias** classified as **delay-like** (grows with speed →
  `EKF2_EV_DELAY`) vs **extrinsic-like** (constant, does not grow with speed →
  `EKF2_EV_POS_*` / frame alignment), cross-checked with `estimator_ev_pos_bias`.
- **Fusion dropouts / coasting**: intervals where `cs_ev_*` is false, `reject_*`
  is set, or the filter is dead-reckoning / in const-position mode.
- **NEES / consistency** for `EKF2_EVP_NOISE` / `EKF2_EVA_NOISE` guidance
  (`estimator_ev_pos_bias.innov_test_ratio` is used as a proxy on default logs).
- **EV rate & latency** from `vehicle_visual_odometry` vs the expected rate
  (`vio_expected_rate_hz` in `config/params.yaml`, default 30 Hz — set it to
  your source's cadence: 2D-lidar ~10–12, 3D SLAM ~10–20, mocap 100+).
- **`EKF2_EV_DELAY` estimate** (our cross-correlation of EV-velocity vs EKF
  velocity — labelled as *our* estimate, not PX4's official sweep method), and
  **EKF2_EV_\*** recommendations. **READ-ONLY** — no FMU writes.

**Default logs are enough for most of it.** `SDLOG_PROFILE=1` does **not** log
`estimator_aid_src_ev_*` or `vehicle_visual_odometry`, so NEES falls back to the
`ev_pos_bias` proxy and the **EV_DELAY estimate is gated off** (2 Hz
`estimator_innovations` is too slow to cross-correlate). To unlock the full
analysis and the delay estimate, drop the topics into
`/fs/microsd/etc/logging/logger_topics.txt` (one per line):

```
estimator_aid_src_ev_pos
estimator_aid_src_ev_yaw
estimator_aid_src_ev_hgt
vehicle_visual_odometry
```

The capability badges show which topic sets were found and the tab warns when
`aid_src` is missing.

> **Domain reference:** thresholds, `EKF2_EV_CTRL` bitmask, height strategy
> (`EKF2_HGT_REF`), the `quality`/`EKF2_EV_QMIN` gotcha and the `EKF2_EV_DELAY`
> tuning method are documented in
> [`odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md`](../../odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md).
> Optical flow, range-height and the other aid sources are covered by the
> generalised **EKF2 aid sources** tab (below).

### EKF2 aid sources tab — EV is one source among many (ULog only)

The **EKF2 aid sources** tab generalises the EV analysis into a **registry** of
every source EKF2 fuses — **external vision, optical flow (MTF-02), range-height
(lidar), GPS, baro, mag** — and runs the *same* metric machine (test-ratios,
innovation bias, dropouts, NEES, rate, delay estimate, per-source
recommendations) over each. External vision is simply the `ev` registry entry;
the registry, not the machine, knows each source's innovation fields, `cs_*`
flags, parameters and DOF (`pid_tuning_assistant/aid_source_analyzer.py`).

For the indoor EV + flow + range stack the first three show up **active** and
GPS/baro/mag as **inactive** cards (no false recommendations). A **top-level
alarm** flags `cs_fake_pos` / `cs_inertial_dead_reckoning` ("all odometry lost —
coasting") across every source. The tab has a **source selector** and
capability badges for which sources are present/active.

Registry facts baked in (verified against PX4 v1.15.4 `msg/` + `module.yaml`):

- **gnss/gps prefix trap** — aid topics are `estimator_aid_src_gnss_*` while
  `estimator_innovations` fields are `gps_*` and flags are `cs_gps*`;
  `estimator_aid_src_gps_*` does **not** exist.
- Gate params are named per source — EV → `EKF2_EVP_GATE`/`EKF2_EVV_GATE`,
  flow → `EKF2_OF_GATE`, range → `EKF2_RNG_GATE`, GPS → `EKF2_GPS_P_GATE`/
  `EKF2_GPS_V_GATE`, baro → `EKF2_BARO_GATE`, mag → `EKF2_MAG_GATE`/
  `EKF2_HDG_GATE`. The **nonexistent** `EKF2_EV_GATE` / `EKF2_EVA_GATE` /
  `EKF2_REQ_FIX` / `EKF2_MAG_DECL` / `EKF2_MAG_YAWLIM` are never suggested
  (guarded in code and by `test_aid_sources.py`).

As with the EV tab, **delay and NEES for any source need its
`estimator_aid_src_<src>` topic in the log** — the default `SDLOG_PROFILE=1`
carries none of them (only `estimator_aid_src_aux_global_position`), so enable
the ones you need via `logger_topics.txt`. Endpoint: `GET /api/aid_sources`.
**READ-ONLY** — no FMU writes.

---

## VIO live monitor (DDS)

A live **watchdog** for external-vision (EV) fusion, shown on the **Live** tab
in DDS mode (separate from the offline VIO/EV ULog tab above). It replaces the
manual `listener estimator_status_flags` / `listener vehicle_visual_odometry`
checks in the MAVLink console for "is it fusing *right now*" questions. Like the
ULog tab it is **source-agnostic** — it watches the PX4 EV interface, not ArUco
topics, so it works identically for ArUco/VPE, 3D-lidar odometry, 2D scan-match,
camera VIO and mocap.

### Responsibility split — live (DDS) vs offline (ULog)

The two views are deliberately complementary, because DDS and ULog expose
different things:

| Question | Where | Why |
|---|---|---|
| Is EV fusing now? At what rate / latency? Is the filter coasting? | **Live (DDS)** | `estimator_status_flags` + `vehicle_odometry` + the EV inputs are on the DDS bus |
| Innovation consistency (test-ratios / NEES), `EKF2_EV_DELAY` estimate, extrinsic-vs-delay bias | **Offline (ULog)** | `estimator_innovations`, `estimator_aid_src_ev_*`, `estimator_ev_pos_bias` are **ULog-only** — not bridged over DDS |

The live panel therefore carries a banner making this explicit: numeric
innovation consistency and the `EKF2_EV_DELAY` estimate live in the ULog tab.

### What the live panel shows

* Fusion "lamps": `cs_ev_pos` / `cs_ev_yaw` / `cs_ev_hgt` / `cs_ev_vel`
  (green = fused, red = not, grey = unknown), a large **coasting** alarm on
  `cs_inertial_dead_reckoning`, and `reject_hor_pos/ver_pos/yaw` edge counters.
* EV-input rate and latency (`now − timestamp_sample`), `quality`,
  `reset_counter`, and finiteness of `position`/`q`.
* EV-input vs EKF-output position divergence.
* A summary **OK / WARN / CRIT** verdict. Expected EV rate is the
  `vio_expected_rate_hz` config parameter (default 30 — ArUco cadence; a 2D
  lidar runs ~10–12, mocap 100+), never hard-coded. CRIT on coasting, on an EV
  input silent longer than the timeout, or on EV position not fused while armed.

Served over the existing `/ws/stream` WebSocket (a `vio` field) and, as a
polling alternative, `GET /api/vio/live`. Both are **read-only** snapshots.

### Required topics

The panel needs these in [`fmu/dds_topics.yaml`](../../../../../fmu/dds_topics.yaml)
(the first two ship there already; the EV input is published by the VPE node):

* `/fmu/out/estimator_status_flags`
* `/fmu/out/vehicle_odometry`
* `/fmu/in/vehicle_visual_odometry` (and optionally `/fmu/in/vehicle_mocap_odometry`,
  the second EV input used by many lidar/mocap bridges)

Absent topics **degrade** the panel (it shows "unavailable") rather than
crashing the node — the VIO topics are an optional subscription group, not
required. The same [domain reference](../../odometry/vpe/px4_local_pose_publisher/PX4_EKF2_TUNING_AND_DEBUG.md)
supplies the latency ceiling (`EKF2_DELAY_MAX` = 200 ms) and fusion-flag meaning.

---

## Vehicle profiles

Profiles shift all analysis thresholds for the vehicle type.
Edit `config/params.yaml`:

```yaml
vehicle_profile: "default"   # options: default | racer | cinematic
```

| Profile | Overshoot threshold | Rise time threshold | Noise threshold | Use for |
|---|---|---|---|---|
| `default` | 20% | 350 ms | 0.08 rad/s | General 5" freestyle, cargo |
| `racer` | 30% | 200 ms | 0.12 rad/s | 3"–5" race quads (aggressive tunes) |
| `cinematic` | 10% | 500 ms | 0.04 rad/s | Heavy cinema gimbals (smooth flight) |

---

## Using the Web UI

### Sidebar controls

| Control | Description |
|---|---|
| **Run name** | Label for this recording session (used in filenames) |
| **▶ Start / ■ Stop** | Start/stop telemetry recording |
| **Axis** | Select roll / pitch / yaw for live chart |
| **Loop** | Rate loop or Attitude loop view |
| **Load .params** | Upload QGC `.params` file for absolute gain values |
| **Topics** | ● green = receiving data, ● yellow = subscribed (no data yet), ● red = error |

### Live tab

- **Rate/Attitude chart**: setpoint (blue) vs measured (green) for selected axis
- **Control output**: torque setpoint (purple) — "fuzziness" = D amplifying noise
- **Motor commands**: all 4 motors — asymmetric noise between motors → mechanical issue

### Sidebar — Source selector

Switch between **DDS** (live recording) and **ULog file** modes.
In ULog mode, upload a `.ulg` file directly from the sidebar.

### Analysis tab (appears after Stop or ULog upload)

- **Warnings**: missing topics, insufficient data, data quality flags
- **Recommendations table**: param / current / suggested / Δ% / confidence / reason
- **Metrics table**: per mode × axis — overshoot, rise time, settling, dominant freq, noise
- **Data Quality panel**: per-topic rate, jitter, drop %, Nyquist frequency
- **Downloads**: JSON, CSV, Parquet, LLM prompt

### HiFi Analysis section (ULog only)

- **Capability badges**: which algorithms ran successfully (FRF, margins, vibration, plant, ARX, filter)
- **Excitation quality**: step vs sweep, good / marginal / poor
- **Bode plots**: magnitude (dB) + phase (°) + coherence overlay; axis selector
- **Stability margins**: gain margin (dB), phase margin (°), gain crossover frequency
- **Vibration spectrum**: Welch PSD of gyro Z axis; detected peaks with motor harmonic labels and filter suggestions
- **Before / After simulation**: step response with current PID vs synthesized PID
- **Synthesized gains**: suggested K / Kp / Ki / Kd from loop-shaping (only shown when plant R² > threshold)
- **Loop delays table**: per-axis delay in ms from cross-correlation and phase slope

### History tab

List of all runs recorded in current session with frame counts and recommendation counts.

---

## Understanding the analysis

### Rate PID is global — use mode comparison

PX4's rate PID (MC_*RATE_K/P/I/D) is the **same** across all flight modes.
Only the outer loops change (attitude controller in Stabilized, position controller in Position).

| Observation | Likely cause |
|---|---|
| Oscillation in Stabilized **and** Position | Rate loop — reduce K |
| OK in Stabilized, worse in Altitude | Attitude P too high or rate margins too thin |
| OK in Stabilized, much worse in Position | Position controller amplifying rate loop marginal stability |
| Drift in Position but stable oscillation-free | I-term too low for disturbance rejection |

### Recommendation rules implemented

| Rule | Trigger | Action |
|---|---|---|
| High-freq oscillation | freq > 1 Hz, damping < 0.5 | Reduce `MC_{AX}RATE_K` −15% |
| Overshoot > 20% | overshoot_mean > 20 | Reduce K −20%, offer D +15% |
| Slow low-freq oscillation | 0.2–1 Hz in STAB/POS | Reduce `MC_{AX}_P` −15% |
| Sluggish rise | rise_time > 350 ms | Increase K +15% |
| Steady-state error | ss_error > 0.05 rad/s, settling > 1.5 s | Increase I +20% |
| Noisy control signal | noise_rms > 0.08 | Reduce D −25%, lower `IMU_GYRO_CUTOFF` |
| Cross-mode divergence | STAB OK, POSCTL bad | Informational note |

### Confidence levels

- **high**: clear signal, multiple events, unambiguous metrics
- **medium**: plausible but could have confounds (wind, vibration)
- **low**: marginal threshold, insufficient events

---

## Export formats

### JSON (`*_analysis.json`)

```json
{
  "schema_version": "1.0",
  "meta": { "run_name", "timestamp", "params", "topic_status", "n_frames" },
  "warnings": [],
  "segments": [
    {
      "mode": "STAB", "axis": "roll", "n_events": 5,
      "metrics": { "overshoot_pct_mean", "rise_time_mean_s", ... },
      "events": [ { "t_start", "overshoot_pct", "dominant_freq_hz", ... } ]
    }
  ],
  "recommendations": [ { "param", "current", "suggested", "delta_pct", ... } ],
  "disclaimer": "..."
}
```

Best for: downstream ML, structured analysis, version control.

### CSV (`*_telemetry.csv`)

Columns: `t, nav_state, roll_rate_sp, roll_rate, torque_roll, pitch_rate_sp,
pitch_rate, torque_pitch, yaw_rate_sp, yaw_rate, torque_yaw,
roll_sp, roll, pitch_sp, pitch, m0, m1, m2, m3, v_bat, bat_remaining`

Downsampled to max 5000 rows. Best for: spreadsheets, quick plotting, pandas.

### Parquet (`*_telemetry.parquet`)

Full-resolution time series. Best for: ML training pipelines, Arrow/Polars.

### LLM prompt (`*_llm_prompt.txt`)

Ready-to-paste prompt including vehicle description, current params,
per-mode metrics summary, and heuristic recommendations.
Paste directly into ChatGPT or a local LLM for second opinion.

---

## Offline / SITL mode

### SITL with Gazebo (recommended first step)

```bash
# Terminal 1 — PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo-classic_iris

# Terminal 2 — uXRCE-DDS Agent (UDP)
MicroXRCEAgent udp4 -p 8888

# Terminal 3 — Tool (--no-ros skips the node if you just want the web server)
source ~/your_ws/install/setup.bash
ros2 run pid_tuning_assistant pid_tuning_assistant
```

In SITL, perform the same step-input sequence using keyboard/joystick in QGC.

### Offline analysis from saved CSV

```python
# Load a previously exported CSV back into the analyzer:
import pandas as pd
from pid_tuning_assistant.analyzer import PIDAnalyzer
from pid_tuning_assistant.data_buffer import TelemetryFrame

df = pd.read_csv("run_001_telemetry.csv")
frames = [TelemetryFrame(**{k: v for k, v in row.items() if pd.notna(v)})
          for _, row in df.iterrows()]

analyzer = PIDAnalyzer(params={})   # or load params dict
result = analyzer.analyse(frames, run_name="offline_replay", topic_status={})
print(analyzer.export_llm_prompt(result))
```

---

## PX4 native autotune integration (prompt 80)

PX4 ships its own attitude-rate autotune, `mc_autotune_attitude_control`
(indirect adaptive: online closed-loop RLS identification of an ARX(2,2,1)
model + GMVC synthesis, Yamamoto/Fujii/Kaneda 1998). It tunes exactly the loops
this assistant targets, so its result can be compared side-by-side with the
model-based synthesis here.

### Procedure (onboard autotune)

1. Set `MC_AT_EN = 1` and **reboot** (the flag is `@reboot_required`).
2. Take off and hold a stable hover in **Altitude mode**. Precondition: the
   aircraft must already be controllable ("if it stabilizes in ≤ 2 oscillations,
   it is ready").
3. Trigger it — either fly quick roll/pitch/yaw inputs or set `MC_AT_START = 1`
   (the qgc **PX4 Autotune** page does this). The module injects **filtered
   square-step** commands (0 ↔ `MC_AT_SYSID_AMP`, wash-out τ ≈ 0.2 s) on each
   axis in turn for ~40 s (19–68 s).
4. Land and disarm. With the default `MC_AT_APPLY = 1` the new gains are written
   **after disarm**; `0` = log-only, `2` = apply in the air.
5. Abort at any time: move a stick (> 0.05), change mode, or exceed the 20 s
   per-axis timeout. Set `COM_RC_OVERRIDE = 3` so stick input aborts.

Sanity gate `areGainsGood()`: all gains > 0 and `kc < 0.5`, `i < 10`, `d < 0.1`,
`att_p < 12` — otherwise the run fails and nothing is applied.

### Limitations

The onboard autotune assumes a **linear SISO** plant with no cross-axis coupling
("2 poles, 2 zeros"); it drifts under insufficient excitation and needs a frame
that is not overly flexible. Its filtered-step excitation is a known weak point
(upstream is moving toward a sine sweep, PR #21857).

### How this tool uses it

* **Trigger + monitor** — the qgc **PX4 Autotune** page (`web/qgc`) starts the
  run and shows the gains **before → after**. Live state
  (`autotune_attitude_control_status`) is not streamed over DDS by default; the
  page reads the applied `MC_*RATE_*` / `MC_*_P` back through `fmu_control`
  (no firmware rebuild). Streaming the status topic is optional — uncomment the
  block in `fmu/dds_topics.yaml` and rebuild.
* **Offline analysis** — a high-rate ULog (`SDLOG_PROFILE=17`) that contains
  `autotune_attitude_control_status` is parsed by `parse_autotune_series()`
  (`ulog_adapter.py`): per-axis converged ARX coefficients `[a1 a2 b0 b1 b2]`
  are extracted (last sample in each axis's identification state) and the plant
  is reconstructed (`num=[b0,b1,b2]`, `den=[1,a1,a2]`, plus a continuous
  2nd-order equivalent).
* **Comparison** (`GET /api/compare`) — per axis, three gain sets side by side:
  **current** (loaded params) · **PX4 autotune** (gains reproduced from the
  logged ARX coeffs via the same GMVC pipeline) · **analyzer** (loop-shaping
  synthesis in `plant_estimator.py`), each with its identified model
  (ARX(2,2,1) vs 2nd-order+delay) and goodness metric (PX4 `fitness` vs
  analyzer VAF). The view is **read-only** — it never writes gains; apply via
  the qgc page. It degrades to two columns when the log has no autotune run.

### Relation to the excitation block (prompts 70–74)

The onboard autotune is the **in-firmware** equivalent of external system-ID: it
injects excitation and identifies a model inside PX4. The excitation block
(prompts 70–74) is the **external, more flexible** alternative — sweep /
multisine excitation and reference-based unbiased identification that the
onboard step-based path cannot do (see prompt 70 §on why an external node cannot
inject at the same point). The two are **complementary**; the comparison view
can show all three gain sources (current / PX4 autotune / analyzer) at once.

---

## Troubleshooting

### No topics receiving data (all yellow dots)

```bash
# Check DDS Agent is running and connected:
ros2 topic list | grep fmu
# If empty → Agent not running or TELEM2 baud mismatch

# Check baud rate — PX4 default TELEM2 is 921600:
MicroXRCEAgent serial --dev /dev/ttyS1 -b 921600

# For Orange Pi Zero 3W with 1.5M baud UART:
MicroXRCEAgent serial --dev /dev/ttyS3 -b 1500000
```

### VehicleRatesSetpoint / VehicleAttitudeSetpoint never go active

These require `dds_topics.yaml` modification + firmware rebuild (see Step 1).
Without them, step-response comparison is impossible. The tool will warn
and skip step detection, but will still record rate measurements.

### High noise / "fuzzy" torque signal

Check `IMU_GYRO_CUTOFF` (PX4 v1.15 default **40 Hz**; the 30 Hz default is
`IMU_DGYRO_CUTOFF`, the separate D-term derivative filter).
Vibration from unbalanced props is the most common cause.
Balance props, tighten motor screws, add anti-vibration mounts.

### Analysis shows 0 step events

Step detection requires deliberate RC stick jabs (threshold: 0.15 rad/s change
in rate setpoint). Smooth stick movements will not trigger detection.
Ensure `VehicleRatesSetpoint` topic is active (green dot).

### px4_msgs import error at startup

```bash
# Verify px4_msgs is built and sourced:
source ~/your_ws/install/setup.bash
python3 -c "from px4_msgs.msg import VehicleAngularVelocity; print('OK')"
```

If it fails, rebuild: `colcon build --packages-select px4_msgs`

### Web UI not reachable from GCS laptop

```bash
# Check firewall on companion:
sudo ufw allow 8086

# Check tool is listening on 0.0.0.0 (not 127.0.0.1):
ros2 run pid_tuning_assistant pid_tuning_assistant --host 0.0.0.0 --port 8086
```

---

## File structure

```
pid_tuning_assistant/
├── package.xml
├── setup.py
├── setup.cfg
├── config/
│   └── params.yaml          # Runtime config (thresholds, nav_state map)
├── resource/
│   └── pid_tuning_assistant # ament resource marker
├── static/
│   └── index.html           # Full web UI (Plotly, no build step needed)
├── examples/
│   ├── example_export.json  # Sample analysis output
│   └── llm_prompt.txt       # Sample LLM prompt
├── test/
│   ├── test_estimator.py     # 28 unit tests — 2nd-order fit, Nyquist, coherence
│   └── test_hifi.py          # Unit tests — FRF, ARX, plant fit, delays, saturation
└── pid_tuning_assistant/
    ├── __init__.py
    ├── data_buffer.py        # Thread-safe TelemetryFrame buffer + TopicStats
    ├── telemetry_node.py     # ROS 2 node — event-driven, PX4 epoch
    ├── analyzer.py           # Step detection, 2nd-order fit, recommendations
    ├── model_estimator.py    # Nyquist guard, resample, fit_second_order, coherence
    ├── input_source.py       # InputSource ABC, DdsRecordingSource, SourceMetadata
    ├── ulog_adapter.py       # ULog→TelemetryFrame adapter (pyulog optional)
    ├── hifi_analyzer.py      # FRF, stability margins, loop delay, vibration, ARX
    ├── plant_estimator.py    # Plant recovery, PID synthesis, step simulation
    ├── param_parser.py       # QGC .params file parser
    ├── web_server.py         # FastAPI + WebSocket + ULog endpoints
    └── main.py               # Entry point (ROS thread + uvicorn)
```
