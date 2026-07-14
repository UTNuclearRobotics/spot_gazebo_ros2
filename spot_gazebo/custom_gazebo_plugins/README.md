# rfid_pd_plugin

Probability-of-detection (PD) RFID reader system plugin for **Gazebo Ignition
Fortress (ign-gazebo 6.x)** / ROS 2 Humble. Implements the tag-antenna PD
model from Alajami et al., *"Simulation of RFID Systems in ROS-Gazebo"*
(IEEE RFID-TA 2022), ported to the gz-sim ECS architecture.

Designed to pair with `server_rack_rfid.sdf`: every model whose name starts
with `rfid_tag_` is treated as a tag. No plugins are required on the tags
themselves for this read path (the `CommsEndpoint` plugins in that world are
for the alternative RFComms approach and can coexist or be deleted).

## PD model

Tag position expressed in antenna-frame spherical coordinates (boresight =
**+X** of the antenna frame):

```
G(thetaH, thetaV) = 2^-[ (2*thetaH / dThetaH)^2 + (2*thetaV / dThetaV)^2 ]
PD                = clamp( 0.5 * (r0 / R)^2 * G , 0 , pd_max )
```

Anchor points, matching the paper: `PD = 0.5` at `R = r0` on boresight;
`1/R^2` surface-power-density decay; probability halves at the -3 dB
beamwidth edges (`dThetaH`, `dThetaV`). The paper does not publish its
closed form, so the Gaussian main-lobe reconstruction above is used —
if you fit `r0` and the beamwidths to your real reader the read-rate
curves should land close to Fig. 5 of the paper. Each read cycle
(`update_rate` Hz), every tag is independently Bernoulli-sampled against
its instantaneous PD, matching the paper's definition of `r0` as the
distance at which half the tags are read within an interrogation window.

Tags behind the antenna plane (|thetaH| >= 90 deg) are never read.

## Build

```bash
# deps (Fortress dev packages; already present if you built other ign plugins)
sudo apt install libignition-gazebo6-dev

cd rfid_pd_plugin
mkdir build && cd build
cmake ..
make -j

# Fortress uses the IGN_ prefixed env var (GZ_SIM_SYSTEM_PLUGIN_PATH is Garden+)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$PWD:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

Inside `cisco_dev`, add the export to the container entrypoint or an env
file so launch files inherit it.

## Usage (SDF)

Attach as a **model plugin** on the robot carrying the antenna. With the
Spot include in `server_rack_rfid.sdf`:

```xml
<model name="spot">
  <pose>0 -2.0 0.7 0 0 0</pose>
  <include merge="true">
    <uri>package://spot_description_sim/models/spot</uri>
  </include>

  <plugin filename="RfidPdReader" name="rfid::RfidPdReader">
    <!-- link the antenna is mounted on; resolved lazily, so include-merge
         ordering is not a problem -->
    <antenna_link>body</antenna_link>
    <!-- mounting offset of the antenna frame in the link frame;
         boresight is +X of this frame -->
    <antenna_pose>0.4 0 0.05 0 0 0</antenna_pose>

    <tag_prefix>rfid_tag_</tag_prefix>

    <!-- RFID system constants (calibrate to your reader/antenna/tags) -->
    <r0>2.5</r0>                    <!-- m, PD=0.5 boresight distance -->
    <delta_theta_h>65</delta_theta_h>  <!-- deg, -3dB azimuth beamwidth -->
    <delta_theta_v>65</delta_theta_v>  <!-- deg, -3dB elevation beamwidth -->
    <max_range>10.0</max_range>     <!-- m, hard cutoff -->
    <pd_max>0.99</pd_max>           <!-- per-cycle PD ceiling -->

    <update_rate>10</update_rate>   <!-- Hz, interrogation cycles -->
    <topic>/rfid/reads</topic>
    <seed>42</seed>                 <!-- omit for random_device seeding -->

    <!-- Passive-tag TF broadcast -->
    <publish_tf>true</publish_tf>
    <tf_topic>/rfid/tf</tf_topic>
    <antenna_frame_id>rfid_antenna</antenna_frame_id>
    <antenna_parent_frame_id>body</antenna_parent_frame_id>  <!-- defaults to antenna_link -->
    <rssi_r0>-60.0</rssi_r0>        <!-- dBm, backscatter RSSI at r0 boresight -->
    <tf_position_noise_stddev>0.0</tf_position_noise_stddev> <!-- m, per axis -->
  </plugin>
</model>
```

`delta_theta_h/v = 65` approximates a Keonn Advantenna-SP11-class patch
antenna; the paper's experiments used the AdvanReader-160 + SP11 combo.

## Outputs & ROS 2 bridge

**1. Read reports** — one `ignition::msgs::StringMsg` per cycle on `<topic>`
(default `/rfid/reads`), JSON with per-detection range and synthesized
backscatter RSSI:

```json
{"t": 12.400,
 "detected": [{"id": "rfid_tag_ups", "r": 1.923, "rssi": -63.4}],
 "n_tags": 10}
```

RSSI uses the passive-tag two-way link budget, anchored at `rssi_r0` dBm
on boresight at `r0`:

```
RSSI = rssi_r0 - 40*log10(R/r0) + 2 * 10*log10(G(thetaH, thetaV))
```

i.e. 1/R^4 decay (12 dB per distance doubling — backscatter, not one-way)
and the antenna pattern applied on both the forward and return paths.

**2. Detection-driven TF** — `ignition::msgs::Pose_V` on `<tf_topic>`
(default `/rfid/tf`) using the `frame_id`/`child_frame_id` header-data
convention (same as gz-sim DiffDrive), so it bridges to
`tf2_msgs/msg/TFMessage` with the stock bridge. Per cycle:

```
antenna_parent_frame_id  ->  antenna_frame_id     (always)
antenna_frame_id         ->  rfid_tag_<name>      (detected tags ONLY)
```

Passive-tag semantics: a tag has no frame until the reader energizes and
reads it; tag frames are parented to the antenna (a reader only knows tags
relative to itself); undetected tags go stale in the TF buffer. Set
`tf_position_noise_stddev` (m, per axis) to model reader localization
uncertainty; default 0 broadcasts ground-truth relative pose, matching the
paper's convention.

**Bridge both** (stamps are sim time — this belongs with your sim-time
bridge config, alongside the TF/depth pipeline, not the wall-time one):

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/rfid/reads@std_msgs/msg/String@ignition.msgs.StringMsg" \
  "/rfid/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V" \
  --ros-args -r /rfid/tf:=/tf
```

Or in a bridge YAML (sim-time config):

```yaml
- ros_topic_name: /rfid/reads
  gz_topic_name: /rfid/reads
  ros_type_name: std_msgs/msg/String
  gz_type_name: ignition.msgs.StringMsg
  direction: GZ_TO_ROS
- ros_topic_name: /tf
  gz_topic_name: /rfid/tf
  ros_type_name: tf2_msgs/msg/TFMessage
  gz_type_name: ignition.msgs.Pose_V
  direction: GZ_TO_ROS
```

In RViz (with `use_sim_time`), enable the TF display: `rfid_antenna` hangs
off `body`, and tag frames pop into existence as Spot reads them —
the paper's Fig. 1b view, but detection-gated.

## Verification ideas

* Static sweep: park the antenna at fixed distances (1.75 m, 3.5 m as in the
  paper), let it dwell 120 s, plot unique tags vs. time from `/rfid/reads`,
  and compare curve shape against Fig. 5.
* `ign topic -e -t /rfid/reads` for a quick smoke test without ROS.
* Set `<seed>` for reproducible runs when regression-testing mission logic.

## Extensions (not implemented)

* Runtime re-coloring of detected tags (paper-style red/grey) — requires
  pushing material component updates through the ECM.
* Multi-antenna support: attach one plugin instance per antenna link with
  distinct `<topic>` values; instances are independent.


---

# RfidReader — multi-port reader (body + arm end effector)

`RfidReader.cc` generalizes `RfidPdReader` into the paper's actual hardware
architecture (Sec. IV-A, Block 3): **one reader, N antenna ports** — the
AdvanReader-160 drives four SP11 antennas over separate ports. Each
`<antenna>` element declares its own link, mounting pose, `r0`, beamwidths,
and optional `rssi_r0` override. PD model, RSSI link budget, JSON transport,
and passive-tag TF semantics are identical to `RfidPdReader`.

**Articulated links:** antenna poses are resolved from the ECM every cycle,
so an antenna on the arm end effector tracks the kinematic chain with no
extra configuration — point the arm (MoveIt or teleop) and the read zone
follows. Verify the end-effector link name in your `spot_description_sim`
(common candidates: `hand`, `arm_link_wr1`); the console warns
`Antenna ... link not found` and keeps retrying if it's wrong.

**Port multiplexing:** `<multiplex>true</multiplex>` (default) interrogates
one port per read cycle, round-robin — matching real reader firmware. Each
antenna's effective interrogation rate is therefore `update_rate / n_ports`;
raise `update_rate` if you add ports and want per-port rates unchanged.
`false` fires all ports every cycle (a tag in two beams can then legitimately
appear twice in one report, once per antenna — dedup by EPC downstream, as
real inventory middleware does).

**JSON schema** (superset of RfidPdReader's — detections gain `antenna`,
top level gains `reader` and `active`):

```json
{"t": 12.400, "reader": "advanreader_160", "active": ["hand"],
 "detected": [{"id": "rfid_tag_switch_2", "r": 0.42, "rssi": -41.7,
               "antenna": "hand"}],
 "n_tags": 10}
```

**TF frames:** every port broadcasts `<link> -> rfid_antenna_<name>` each
cycle (geometry never goes stale); detected tags are parented to the antenna
that read them: `rfid_antenna_hand -> rfid_tag_switch_2`. Antenna `<name>`s
are prefixed to avoid colliding with existing robot frames (an antenna named
`hand` gets frame `rfid_antenna_hand`, not `hand`).

**Bridge:** identical to RfidPdReader (same topics, same sim-time caveat).

**Don't run RfidReader and RfidPdReader simultaneously** on the same tag
population unless you intend two independent readers — reads will double.

Example (as wired into `worlds/server_rack_rfid.sdf`): chassis coverage
antenna (`r0=2.5`, 65 deg) on port 1 + close-range verification antenna on
the end effector (`r0=1.0`, 80 deg wide beam, hotter `rssi_r0=-55`) on
port 2. The intended workflow: the body antenna finds *that* a tag exists
during a Nav2 sweep; the arm antenna confirms *which rack unit* it is by
pointing at it from ~0.3-0.5 m, where its PD approaches the ceiling and
the body antenna's geometry can't discriminate adjacent 2RU faceplates.
