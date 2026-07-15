# RFID reader plugins

RFID reader system plugins for **Gazebo Ignition Fortress (ign-gazebo 6.x)**
/ ROS 2 Humble. The detection model derives from Alajami et al.,
*"Simulation of RFID Systems in ROS-Gazebo"* (IEEE RFID-TA 2022), ported to
the gz-sim ECS architecture and extended with TX power control, a stochastic
link, and occlusion.

Two plugins live here:

| | |
|---|---|
| **`RfidReader.cc`** | The one to use. Multi-port capable, defaults target the Vulcan RFID Iron. |
| **`RfidPdReader.cc`** | Legacy single-antenna ancestor. **Diverged** — see the note at the bottom. |

Pairs with `worlds/server_rack_rfid.sdf`: every model whose name starts with
`rfid_tag_` is treated as a tag. No plugins are required on the tags
themselves (the `CommsEndpoint` plugins in that world are for the
alternative RFComms approach and can coexist or be deleted).

---

## Hardware being modeled

Defaults target the **Vulcan RFID Iron USB Reader**
(`VUL-USB-IRON-INTANT-ENC-US`), arm-mounted:

| Datasheet | Value | Consequence |
|---|---|---|
| Antenna Ports | **None / Integrated Antenna** | One `<antenna>`. The multi-port path is dormant. |
| Transmit Power | **0 → 27 dBm, 0.5 dBm steps** | `p_min` / `p_max` / `power_step` |
| Max Read Distance | **"Testing Recommended"** | `r0` is a *derived guess*. See below. |
| Max Read Rate | 150 tags/s | Above `update_rate`; not currently a constraint. |
| Dimensions | 72 × 72 × 23 mm | Electrically small at λ≈33 cm; ~0–2 dBi assumed. |

**A second Iron is not a second port.** Each Iron is an independent USB
reader. Model it as its own `<plugin>` block with its own `<topic>`, not as
a second `<antenna>` — the multiplex path models ports time-sharing one
reader's RF chain, which two Irons don't do.

**Regulatory:** 27 dBm into ~0–2 dBi is ~29 dBm EIRP against the FCC 15.247
36 dBm cap, and 27 dBm conducted against the 30 dBm limit. The Iron cannot
breach either. `p_max` is a hardware limit, not a legal one.

---

## Detection model

Tag position in antenna-frame spherical coordinates (boresight = **+X**):

```
G(θH, θV) = 2^-[ (2θH/ΔθH)² + (2θV/ΔθV)² ]
r0_eff    = r0 · 10^((dP + off_tag − A)/20)
PD        = clamp( 0.5 · (r0_eff/R)² · G , 0 , pd_max )
RSSI      = rssi_r0 + dP + off_tag − 2A − 40·log10(R/r0)
            + 2·10·log10(G) + N(0, σ_rssi²)
```

`dP = power_dbm − p_ref`; `A` = path occlusion [dB]. Every tag is
Bernoulli-sampled against its PD once per interrogation cycle.

Anchors, matching the paper: `PD = 0.5` at `R = r0` on boresight, 1/R²
surface-power decay, probability halving at the −3 dB beamwidth edges. The
paper does not publish its closed form, so the Gaussian main-lobe above is a
**reconstruction** — worth remembering before trusting it.

The two channels carry **different distance exponents** (1/R² forward,
1/R⁴ backscatter). That asymmetry is what makes `off_tag` separable from
range, and it's why occlusion costs RSSI twice what it costs detection
range.

### TX power

`<r0>` and `<rssi_r0>` are characterized at `<p_ref>`. Commanding power `P`
shifts both consistently: the forward link's 1/R² gives `r0 ∝ √P`, while
backscatter is linear in `P` so RSSI shifts dB-for-dB.

Commands are JSON on `<power_topic>`:

```
{"power_dbm": 24.0}                  -> all ports
{"port": "hand", "power_dbm": 24.0}  -> one port
```

Quantized to `<power_step>`, clamped to `[<p_min>, <p_max>]`.

> **The echo is authoritative.** The command path is asynchronous w.r.t. the
> read cycle. Never attribute reads to a *commanded* power by wall-clock
> ordering — wait for `active[].power_dbm` in the report to change, then
> count from that report's `cycle`.

### Occlusion

`<occluder>` elements are attenuation volumes: oriented boxes charging the
direct antenna→tag segment `<attenuation_db>` for entering plus
`<attenuation_db_per_m>` × the chord traversed.

Attenuation is in **dB, not binary line-of-sight**, deliberately: at λ≈33 cm
a metal rack diffracts and leaks rather than casting a hard shadow, so a
raycast hit test would model a physics UHF does not have.

`A` applies **once** to the forward link (an occluded tag needs more power to
wake) and **twice** to RSSI (the backscatter recrosses the same material).

> **Known gap:** this models blocking, not leakage. Real UHF in a metal aisle
> has both more attenuation *and* more spurious reads via reflection off
> neighbouring racks. Nothing here creates a read along a path that doesn't
> exist, so the sim stays optimistic about false localization. Closing that
> needs a reflection model, not a bigger dB.

### Stochastic link

- `<rssi_noise_stddev>` — per-read Gaussian on RSSI (multipath fading).
- `<tag_offset_stddev>` — a fixed per-tag dB offset lumping sensitivity and
  backscatter-efficiency deviation (detuning on metal, mounting,
  manufacturing spread). Drawn once per tag from `(seed, hash(tag name))`,
  *not* from the main RNG stream: ECM enumeration order isn't guaranteed
  stable, so a stream draw would be irreproducible even at a fixed seed.
  Keying on the name also means a tag keeps its offset across a
  despawn/respawn — which is what a physical tag being moved and re-read
  should look like.

Both default to `0.0` (deterministic link). Both are **outputs of bench
characterization**, not values to be invented — see
`rfid_inventory_tracking/doc/characterization_protocol.md`.

### Tags

Tag *poses* are resolved from the ECM every cycle, so a tag that **moves** is
tracked correctly. The tag *entity set* is maintained via
`EachNew`/`EachRemoved`, so tags **spawned or deleted** mid-run are picked up
too. `SyncTags` runs every iteration rather than on the read cadence —
those callbacks only report entities changed *that* iteration, so sampling
them at `update_rate` would miss spawns between read cycles.

---

## Ground truth vs. hardware

The simulator emits things a real reader never produces. Anything an
estimator meant to run on hardware consumes must have a hardware
counterpart:

| | Hardware equivalent? |
|---|---|
| `map → rfid_antenna_<port>` TF | **Yes** — nav stack + arm FK |
| `map → rfid_tag_<id>` TF | **No** — validation only |
| `r` field in the read report | **No** — a reader returns EPC + RSSI, not range |
| `n_tags` field | **No** — leaks the count inventory exists to discover |
| detections, `rssi`, `power_dbm`, `cycle` | **Yes** |

Set `<report_n_tags>false</report_n_tags>` once consumers score
non-detections against their own expected-asset manifest.

---

## `r0` and the 12-inch spec

The datasheet declines to state read distance, so **12 in = 0.305 m** is the
working assumption.

**"Maximum read distance" is not `r0`.** `r0` is where a *single*
interrogation has PD = 0.5; a quoted read range is what a *dwell* gets you:

```
P(≥1 read in N interrogations) = 1 − (1 − PD)^N
want 0.5 at R = 0.305 m with N = 10  ->  PD = 0.067
PD = 0.5·(r0/R)²                     ->  r0 = 0.112 m
```

So `r0 = 0.11 m` at 27 dBm. That N is an assumption; the number is only as
good as it.

**`<max_range>` is a compute guard (0.5 m), not the range spec.** A hard
cutoff at the true read distance would put a discontinuity in the likelihood
exactly where an estimator is most sensitive — a detection at 11.9 in versus
none at 12.1 in would carry infinite information — and no reader behaves that
way. The 12-inch spec lives in `r0`, as a smooth rolloff.

### The power sweep has a floor

`r0(P) = 0.11 · 10^((P−27)/20)`, so `r0(0 dBm) = 4.9 mm` — well inside the
~1.4–3.2 cm far-field boundary for this aperture, where the 1/R² and 1/R⁴
exponents stop holding. **The usable sweep is ~15 → 27 dBm**, not 0 → 27:
`r0` from ~3 cm to ~11 cm.

That's still enough. At 80° beamwidth the −3 dB footprint is `2·R·tan(40°)`:
~18 cm (≈4 RU) at r0 = 11 cm, ~5 cm (≈1 RU) at r0 = 3 cm — the sweep resolves
a tag from "somewhere in these 4 rack units" down to "this one". But an
estimator swept to 0 dBm is fitting physics that isn't there.

---

## Build

```bash
sudo apt install libignition-gazebo6-dev

cd custom_gazebo_plugins
mkdir build && cd build
cmake .. && make -j

# Fortress uses the IGN_ prefix (GZ_SIM_SYSTEM_PLUGIN_PATH is Garden+)
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=$PWD:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH
```

Inside `cisco_dev`, add the export to the container entrypoint or an env file
so launch files inherit it.

## Usage

See the fully-commented block in `worlds/server_rack_rfid.sdf` for a working
configuration. Full SDF element reference is in the header comment of
`RfidReader.cc`.

## ROS 2 bridge

Already wired in `spot_bringup_sim/config/spot_bridge.yaml`. Stamps are sim
time, so these belong with the sim-time bridge config:

```yaml
- ros_topic_name: /rfid/reads          # GZ_TO_ROS, std_msgs/String
- ros_topic_name: /tf                  # GZ_TO_ROS, tf2_msgs/TFMessage, from /rfid/tf
- ros_topic_name: /rfid/set_power      # ROS_TO_GZ, std_msgs/String
```

Smoke test:

```bash
ros2 topic pub --once /rfid/set_power std_msgs/msg/String \
  '{data: "{\"port\": \"hand\", \"power_dbm\": 20.0}"}'
ros2 topic echo /rfid/reads   # -> "active":[{"name":"hand","power_dbm":20.00}]
```

## Read report

```json
{"t": 12.400, "cycle": 124, "reader": "vulcan_iron",
 "active": [{"name": "hand", "power_dbm": 27.0}],
 "detected": [{"id": "rfid_tag_ups", "r": 0.084, "rssi": -42.1,
               "antenna": "hand", "power_dbm": 27.0}],
 "n_tags": 10}
```

> **A report is published every cycle, including when `detected` is empty.**
> An empty report is evidence — *this pose, this power, nothing heard* — and
> is the primary input to non-detection scoring. Do not throttle, coalesce,
> or drop it anywhere downstream. Conversely, a port *not* interrogated on a
> cycle yields no record and no evidence: absence of a record ≠ absence of a
> detection.

## TF

```
<link>            ->  rfid_antenna_<name>   (every cycle — geometry never goes stale)
rfid_antenna_hand ->  rfid_tag_<name>       (detected tags ONLY)
```

Antenna names are prefixed to avoid colliding with robot frames (an antenna
named `hand` gets `rfid_antenna_hand`, not `hand`). Tag frames are parented
to the antenna because a reader knows tags relative to itself; undetected
tags go stale in the buffer.

## Reproducibility

`<seed>` fixes the run. The drawn seed is **always logged**, so a run started
without an explicit seed can still be reproduced after the fact:

```
[RfidReader]   seed=1234567  (set <seed>1234567</seed> to reproduce this run)
```

## Verification ideas

* Static sweep: park the antenna at fixed distances, dwell, plot read rate
  vs. range from `/rfid/reads`. This is Test A of the characterization
  protocol, run against the sim — useful for validating the fitting pipeline
  before hardware lands.
* Power sweep: fix geometry, step power, confirm the detection set nests.
* `ign topic -e -t /rfid/reads` for a smoke test without ROS.

---

## Note on `RfidPdReader.cc`

The single-antenna ancestor of `RfidReader`. It has **diverged**: it has no
TX power control, no RSSI noise, no per-tag offsets, no occlusion, and no
`cycle`/`power_dbm` report fields. Its PD model is `RfidReader`'s minus every
extension above.

This is a liability — two copies of a measurement model that *is* the
research contribution. It should be retired, or reduced to a thin
single-antenna configuration of `RfidReader`. Until then, treat
`RfidReader.cc` as the only source of truth for the model.

**Don't run both at once** on the same tag population unless you intend two
independent readers — reads will double.
