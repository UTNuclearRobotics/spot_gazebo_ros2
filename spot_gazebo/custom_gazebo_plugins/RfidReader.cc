/*
 * RfidReader — RFID reader system plugin for Gazebo Ignition Fortress
 * (ign-gazebo 6.x). One reader driving N antenna ports.
 *
 * SDF configuration (attach as a MODEL plugin to the robot):
 *
 *   <plugin filename="RfidReader" name="rfid::RfidReader">
 *     <reader_id>vulcan_iron</reader_id>
 *     <tag_prefix>rfid_tag_</tag_prefix>
 *     <update_rate>10</update_rate>        interrogation cycles [Hz]
 *     <multiplex>true</multiplex>          round-robin ports
 *     <topic>/rfid/reads</topic>
 *     <publish_tf>true</publish_tf>
 *     <tf_topic>/rfid/tf</tf_topic>
 *     <rssi_r0>-60.0</rssi_r0>             [dBm] default; per-antenna override
 *     <tf_position_noise_stddev>0.0</tf_position_noise_stddev>   [m]
 *     <max_range>0.5</max_range>           [m] COMPUTE GUARD ONLY, not the
 *                                          reader's range spec. Read range
 *                                          belongs in <r0>; a hard cutoff at
 *                                          the real read distance puts a
 *                                          discontinuity in the likelihood.
 *                                          Keep well beyond the useful range.
 *     <pd_max>0.99</pd_max>                per-cycle PD ceiling
 *     <seed>42</seed>                      always logged; omit = random
 *
 *     <power_topic>/rfid/set_power</power_topic>   JSON StringMsg commands:
 *                                          {"power_dbm":24.0}  -> all ports
 *                                          {"port":"hand","power_dbm":24.0}
 *     <p_ref>27.0</p_ref>                  [dBm] power at which <r0> and
 *                                          <rssi_r0> are characterized
 *     <p_min>0.0</p_min>                   [dBm] command clamp
 *     <p_max>27.0</p_max>
 *     <power_step>0.5</power_step>         [dB] quantization; 0 = continuous
 *
 *     <rssi_noise_stddev>1.5</rssi_noise_stddev>   per-read fading [dB]
 *     <tag_offset_stddev>2.0</tag_offset_stddev>   per-tag spread [dB]
 *     <report_n_tags>true</report_n_tags>  ground truth; validation only
 *
 *     <occluder>                           repeat per attenuation volume
 *       <name>rack_body</name>
 *       <pose>0 4.04 1.015 0 0 0</pose>    world frame, or model frame if
 *                                          <model> is set
 *       <size>0.60 1.03 2.03</size>        [m]
 *       <model>server_rack</model>         optional: ride a model's pose
 *       <attenuation_db>6.0</attenuation_db>               [dB] per traversal
 *       <attenuation_db_per_m>25.0</attenuation_db_per_m>  [dB/m] x chord
 *     </occluder>
 *
 *     <antenna>                            one per physical reader antenna
 *       <name>hand</name>                  TF frame: rfid_antenna_<name>
 *       <link>arm0_hand</link>             MUST name a link in the ECM, i.e.
 *                                          an SDF link name, NOT a URDF one
 *       <pose>0 0 0 0 0 0</pose>           in link frame; boresight = +X
 *       <r0>0.11</r0>                      [m] PD=0.5 range at p_ref, clear air
 *       <delta_theta_h>80</delta_theta_h>  [deg] -3dB beamwidth
 *       <delta_theta_v>80</delta_theta_v>  [deg] -3dB beamwidth
 *       <rssi_r0>-45.0</rssi_r0>           [dBm] at r0, boresight, p_ref
 *       <power_dbm>27.0</power_dbm>        [dBm] optional initial; default
 *                                          p_ref
 *     </antenna>
 *   </plugin>
 *
 * JSON read report on <topic>:
 *   {"t":12.400,"cycle":124,"reader":"vulcan_iron",
 *    "active":[{"name":"hand","power_dbm":27.0}],
 *    "detected":[{"id":"rfid_tag_ups","r":0.084,"rssi":-42.1,
 *                 "antenna":"hand","power_dbm":27.0}],
 *    "n_tags":10}
 *
 * A report is published EVERY cycle, including when "detected" is empty. An
 * empty report is evidence — this pose, this power, nothing heard — and is
 * the primary input to non-detection scoring. Do not drop, throttle or
 * coalesce it downstream.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>

#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/msgs/pose_v.pb.h>
#include <ignition/msgs/Utility.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>

namespace rfid
{

class RfidReader
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPostUpdate
{
  private: struct Antenna
  {
    std::string name;          ///< port name; TF frame = rfid_antenna_<name>
    std::string linkName;
    std::string parentFrame;   ///< ROS parent frame for TF (default: link)
    ignition::math::Pose3d pose;   ///< mounting offset in link frame
    double r0{0.11};           ///< PD=0.5 range at p_ref, clear air
    double dThetaH{80.0 * M_PI / 180.0};
    double dThetaV{80.0 * M_PI / 180.0};
    double rssiR0{-45.0};      ///< RSSI at r0, boresight, p_ref
    double powerDbm{27.0};     ///< commanded TX power (guarded by mutex)
    ignition::gazebo::Entity link{ignition::gazebo::kNullEntity};
    bool warned{false};
  };

  /// \brief A tag and its fixed link offset.
  private: struct TagInfo
  {
    std::string name;
    double offsetDb{0.0};   ///< lumped sensitivity + backscatter deviation
  };

  /// \brief An attenuation volume. Charges the antenna->tag segment
  /// attenDb for entering plus attenDbPerM per metre of chord.
  private: struct Occluder
  {
    std::string name;
    std::string modelName;     ///< optional: pose is relative to this model
    ignition::math::Pose3d pose;
    ignition::math::Vector3d size;
    double attenDb{0.0};
    double attenDbPerM{0.0};
    ignition::gazebo::Entity model{ignition::gazebo::kNullEntity};
    bool warned{false};
  };

  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager &_ecm,
      ignition::gazebo::EventManager & /*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      ignerr << "[RfidReader] Plugin must be attached to a model.\n";
      return;
    }

    // ---- reader-level parameters ---------------------------------------
    this->readerId =
        _sdf->Get<std::string>("reader_id", this->readerId).first;
    this->tagPrefix =
        _sdf->Get<std::string>("tag_prefix", this->tagPrefix).first;
    this->updateRate =
        _sdf->Get<double>("update_rate", this->updateRate).first;
    this->multiplex = _sdf->Get<bool>("multiplex", this->multiplex).first;
    this->maxRange  = _sdf->Get<double>("max_range", this->maxRange).first;
    this->pdMax     = _sdf->Get<double>("pd_max", this->pdMax).first;
    this->publishTf = _sdf->Get<bool>("publish_tf", this->publishTf).first;
    this->tfNoiseStddev = _sdf->Get<double>(
        "tf_position_noise_stddev", this->tfNoiseStddev).first;

    // ---- TX power -------------------------------------------------------
    this->pRef = _sdf->Get<double>("p_ref", this->pRef).first;
    this->pMin = _sdf->Get<double>("p_min", this->pMin).first;
    this->pMax = _sdf->Get<double>("p_max", this->pMax).first;
    this->powerStep =
        _sdf->Get<double>("power_step", this->powerStep).first;
    if (this->powerStep < 0.0)
    {
      ignwarn << "[RfidReader] <power_step> must be >= 0; disabling "
              << "quantization.\n";
      this->powerStep = 0.0;
    }
    if (this->pMin > this->pMax)
    {
      ignerr << "[RfidReader] <p_min> (" << this->pMin << ") > <p_max> ("
             << this->pMax << "); swapping.\n";
      std::swap(this->pMin, this->pMax);
    }
    if (this->pRef < this->pMin || this->pRef > this->pMax)
    {
      ignwarn << "[RfidReader] <p_ref> (" << this->pRef << " dBm) is outside "
              << "[<p_min>, <p_max>] = [" << this->pMin << ", " << this->pMax
              << "] dBm. r0/rssi_r0 are characterized at p_ref, so no "
              << "commandable power can reproduce the reference link.\n";
    }

    // ---- stochastic link ------------------------------------------------
    this->rssiNoiseStddev =
        _sdf->Get<double>("rssi_noise_stddev", this->rssiNoiseStddev).first;
    this->tagOffsetStddev =
        _sdf->Get<double>("tag_offset_stddev", this->tagOffsetStddev).first;
    this->reportNTags =
        _sdf->Get<bool>("report_n_tags", this->reportNTags).first;

    const double defaultRssiR0 =
        _sdf->Get<double>("rssi_r0", -60.0).first;

    const std::string topic =
        _sdf->Get<std::string>("topic", "/rfid/reads").first;
    const std::string tfTopic =
        _sdf->Get<std::string>("tf_topic", "/rfid/tf").first;
    const std::string powerTopic =
        _sdf->Get<std::string>("power_topic", "/rfid/set_power").first;

    // Drawn seed is retained and logged so a run without an explicit <seed>
    // can still be reproduced.
    this->seedUsed = _sdf->Get<unsigned int>(
        "seed", std::random_device{}()).first;
    this->rng.seed(this->seedUsed);

    if (this->updateRate <= 0.0)
      this->updateRate = 10.0;
    this->period = std::chrono::duration<double>(1.0 / this->updateRate);

    // Clone: GetElement/GetNextElement are non-const in libsdformat.
    auto sdfClone = _sdf->Clone();

    // ---- occluders ------------------------------------------------------
    if (sdfClone->HasElement("occluder"))
    {
      int idx = 0;
      for (auto elem = sdfClone->GetElement("occluder"); elem;
           elem = elem->GetNextElement("occluder"), ++idx)
      {
        Occluder o;
        o.name = elem->Get<std::string>(
            "name", "occluder" + std::to_string(idx)).first;
        o.modelName = elem->Get<std::string>("model", "").first;
        o.pose = elem->Get<ignition::math::Pose3d>(
            "pose", ignition::math::Pose3d::Zero).first;
        o.size = elem->Get<ignition::math::Vector3d>(
            "size", ignition::math::Vector3d::Zero).first;
        o.attenDb = elem->Get<double>("attenuation_db", 0.0).first;
        o.attenDbPerM =
            elem->Get<double>("attenuation_db_per_m", 0.0).first;

        if (o.size.X() <= 0.0 || o.size.Y() <= 0.0 || o.size.Z() <= 0.0)
        {
          ignerr << "[RfidReader] Occluder '" << o.name << "' has a "
                 << "non-positive <size>; skipping.\n";
          continue;
        }
        this->occluders.push_back(o);
      }
    }

    // ---- antenna ports --------------------------------------------------
    if (sdfClone->HasElement("antenna"))
    {
      int idx = 0;
      for (auto elem = sdfClone->GetElement("antenna"); elem;
           elem = elem->GetNextElement("antenna"), ++idx)
      {
        Antenna a;
        a.name = elem->Get<std::string>(
            "name", "port" + std::to_string(idx)).first;
        a.linkName = elem->Get<std::string>("link", "body").first;
        a.parentFrame = elem->Get<std::string>(
            "parent_frame", a.linkName).first;
        a.pose = elem->Get<ignition::math::Pose3d>(
            "pose", ignition::math::Pose3d::Zero).first;
        a.r0 = elem->Get<double>("r0", 0.11).first;
        a.dThetaH = elem->Get<double>("delta_theta_h", 80.0).first
            * M_PI / 180.0;
        a.dThetaV = elem->Get<double>("delta_theta_v", 80.0).first
            * M_PI / 180.0;
        a.rssiR0 = elem->Get<double>("rssi_r0", defaultRssiR0).first;
        a.powerDbm = this->ConditionPower(
            elem->Get<double>("power_dbm", this->pRef).first);

        // <pose> is resolved in the LINK frame but TF is published under
        // <parent_frame>; they agree only if the frames are coincident.
        if (a.parentFrame != a.linkName)
        {
          ignwarn << "[RfidReader] Antenna '" << a.name
                  << "': <parent_frame> ('" << a.parentFrame
                  << "') differs from <link> ('" << a.linkName
                  << "'). <pose> is interpreted in the LINK frame but the "
                  << "antenna TF is published under <parent_frame> — these "
                  << "agree only if the two frames are coincident. Prefer "
                  << "omitting <parent_frame> and baking any offset into "
                  << "<pose>.\n";
        }

        this->antennas.push_back(a);
      }
    }
    if (this->antennas.empty())
    {
      ignerr << "[RfidReader] No <antenna> elements configured; "
             << "reader will do nothing.\n";
      return;
    }

    this->pub = this->node.Advertise<ignition::msgs::StringMsg>(topic);
    if (this->publishTf)
      this->tfPub = this->node.Advertise<ignition::msgs::Pose_V>(tfTopic);

    if (!this->node.Subscribe(powerTopic, &RfidReader::OnPowerCmd, this))
    {
      ignerr << "[RfidReader] Failed to subscribe to power topic '"
             << powerTopic << "'; TX power will stay at its initial value.\n";
    }

    ignmsg << "[RfidReader] '" << this->readerId << "' with "
           << this->antennas.size() << " antenna port(s), "
           << (this->multiplex ? "multiplexed" : "simultaneous")
           << ", rate=" << this->updateRate << "Hz, topic=" << topic
           << ", tf=" << (this->publishTf ? tfTopic : "off") << "\n";
    ignmsg << "[RfidReader]   seed=" << this->seedUsed
           << "  (set <seed>" << this->seedUsed
           << "</seed> to reproduce this run)\n";
    ignmsg << "[RfidReader]   power: ref=" << this->pRef << "dBm, range=["
           << this->pMin << ", " << this->pMax << "]dBm, step="
           << this->powerStep << "dB, cmd topic=" << powerTopic << "\n";
    ignmsg << "[RfidReader]   noise: rssi_stddev="
           << this->rssiNoiseStddev << "dB, tag_offset_stddev="
           << this->tagOffsetStddev << "dB\n";
    ignmsg << "[RfidReader]   occluders: " << this->occluders.size() << "\n";
    for (const auto &o : this->occluders)
    {
      ignmsg << "[RfidReader]     '" << o.name << "' " << o.attenDb
             << "dB + " << o.attenDbPerM << "dB/m"
             << (o.modelName.empty() ? "" : " on model '" + o.modelName + "'")
             << "\n";
    }
    for (const auto &a : this->antennas)
    {
      ignmsg << "[RfidReader]   port '" << a.name << "' on link '"
             << a.linkName << "' r0=" << a.r0 << "m @" << a.powerDbm
             << "dBm\n";
    }
  }

  /// \brief Extract one numeric JSON value. Flat schema only — no nesting,
  /// no arrays, no escapes.
  private: static bool JsonNumber(const std::string &_s,
      const std::string &_key, double &_out)
  {
    const std::string k = "\"" + _key + "\"";
    auto p = _s.find(k);
    if (p == std::string::npos)
      return false;
    p = _s.find(':', p + k.size());
    if (p == std::string::npos)
      return false;
    try
    {
      _out = std::stod(_s.substr(p + 1));
    }
    catch (const std::exception &)
    {
      return false;
    }
    return true;
  }

  /// \brief Minimal extraction of one string JSON value.
  private: static bool JsonString(const std::string &_s,
      const std::string &_key, std::string &_out)
  {
    const std::string k = "\"" + _key + "\"";
    auto p = _s.find(k);
    if (p == std::string::npos)
      return false;
    p = _s.find(':', p + k.size());
    if (p == std::string::npos)
      return false;
    const auto q1 = _s.find('"', p + 1);
    if (q1 == std::string::npos)
      return false;
    const auto q2 = _s.find('"', q1 + 1);
    if (q2 == std::string::npos)
      return false;
    _out = _s.substr(q1 + 1, q2 - q1 - 1);
    return true;
  }

  /// \brief Quantize to <power_step>, then clamp to [p_min, p_max].
  /// Quantize first so the clamp bounds stay exactly representable.
  private: double ConditionPower(double _p) const
  {
    double q = _p;
    if (this->powerStep > 0.0)
      q = std::round(_p / this->powerStep) * this->powerStep;
    return std::clamp(q, this->pMin, this->pMax);
  }

  /// \brief TX power command callback. Runs on an ign-transport thread,
  /// NOT the PostUpdate thread — hence powerMutex.
  private: void OnPowerCmd(const ignition::msgs::StringMsg &_msg)
  {
    const std::string &s = _msg.data();

    double p = 0.0;
    if (!JsonNumber(s, "power_dbm", p))
    {
      ignwarn << "[RfidReader] set_power: no numeric 'power_dbm' in '"
              << s << "'; expected {\"power_dbm\": 24.0} or "
              << "{\"port\": \"hand\", \"power_dbm\": 24.0}.\n";
      return;
    }
    if (!std::isfinite(p))
    {
      ignwarn << "[RfidReader] set_power: non-finite power ignored.\n";
      return;
    }

    const double applied = this->ConditionPower(p);
    if (std::abs(applied - p) > 1e-9)
    {
      ignwarn << "[RfidReader] set_power: " << p << "dBm -> " << applied
              << "dBm (step=" << this->powerStep << "dB, limits ["
              << this->pMin << ", " << this->pMax << "]dBm).\n";
    }

    std::string port;
    const bool hasPort = JsonString(s, "port", port);

    bool matched = false;
    {
      std::lock_guard<std::mutex> lock(this->powerMutex);
      for (auto &a : this->antennas)
      {
        if (hasPort && a.name != port)
          continue;
        a.powerDbm = applied;
        matched = true;
      }
    }

    if (!matched)
    {
      ignwarn << "[RfidReader] set_power: no antenna port named '" << port
              << "'.\n";
      return;
    }
    ignmsg << "[RfidReader] set_power: "
           << (hasPort ? "port '" + port + "'" : std::string("all ports"))
           << " -> " << applied << "dBm\n";
  }

  /// \brief Draw a tag's fixed link offset [dB] from (seed, tag name), not
  /// from the main RNG stream: ECM enumeration order is not stable across
  /// runs, so a stream draw would not reproduce at a fixed seed.
  private: double DrawTagOffset(const std::string &_name) const
  {
    if (this->tagOffsetStddev <= 0.0)
      return 0.0;
    const auto h = static_cast<uint64_t>(std::hash<std::string>{}(_name));
    std::seed_seq seq{
        static_cast<uint32_t>(this->seedUsed),
        static_cast<uint32_t>(h & 0xffffffffull),
        static_cast<uint32_t>((h >> 32) & 0xffffffffull)};
    std::mt19937 gen(seq);
    std::normal_distribution<double> d(0.0, this->tagOffsetStddev);
    return d(gen);
  }

  /// \brief Chord length [m] of segment _a->_b through an oriented box.
  /// Slab method in the box frame; 0 if the segment misses.
  private: static double SegmentBoxChord(
      const ignition::math::Vector3d &_a,
      const ignition::math::Vector3d &_b,
      const ignition::math::Pose3d &_boxPose,
      const ignition::math::Vector3d &_size)
  {
    const ignition::math::Vector3d a =
        _boxPose.Rot().RotateVectorReverse(_a - _boxPose.Pos());
    const ignition::math::Vector3d b =
        _boxPose.Rot().RotateVectorReverse(_b - _boxPose.Pos());
    const ignition::math::Vector3d d = b - a;
    const ignition::math::Vector3d h = _size * 0.5;

    double t0 = 0.0, t1 = 1.0;
    for (int i = 0; i < 3; ++i)
    {
      const double di = (i == 0) ? d.X() : (i == 1) ? d.Y() : d.Z();
      const double ai = (i == 0) ? a.X() : (i == 1) ? a.Y() : a.Z();
      const double hi = (i == 0) ? h.X() : (i == 1) ? h.Y() : h.Z();
      if (std::abs(di) < 1e-12)
      {
        if (ai < -hi || ai > hi)
          return 0.0;          // parallel to this slab and outside it
        continue;
      }
      double tn = (-hi - ai) / di;
      double tf = ( hi - ai) / di;
      if (tn > tf)
        std::swap(tn, tf);
      t0 = std::max(t0, tn);
      t1 = std::min(t1, tf);
      if (t0 > t1)
        return 0.0;
    }
    return (t1 - t0) * d.Length();
  }

  /// \brief Total one-way attenuation [dB] on the direct antenna->tag path.
  private: double PathAttenuationDb(
      const ignition::math::Vector3d &_a,
      const ignition::math::Vector3d &_b,
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    double att = 0.0;
    for (auto &o : this->occluders)
    {
      ignition::math::Pose3d pose = o.pose;
      if (!o.modelName.empty())
      {
        if (o.model == ignition::gazebo::kNullEntity)
        {
          o.model = _ecm.EntityByComponents(
              ignition::gazebo::components::Name(o.modelName),
              ignition::gazebo::components::Model());
          if (o.model == ignition::gazebo::kNullEntity)
          {
            if (!o.warned)
            {
              ignwarn << "[RfidReader] Occluder '" << o.name << "': model '"
                      << o.modelName << "' not found (will keep trying); "
                      << "this volume attenuates nothing meanwhile.\n";
              o.warned = true;
            }
            continue;
          }
        }
        pose = ignition::gazebo::worldPose(o.model, _ecm) * o.pose;
      }

      const double chord = SegmentBoxChord(_a, _b, pose, o.size);
      if (chord > 0.0)
        att += o.attenDb + o.attenDbPerM * chord;
    }
    return att;
  }

  /// \brief Maintain the set of tag entities (world models named
  /// <tag_prefix>*) via EachNew/EachRemoved. Poses are NOT cached; they are
  /// resolved from the ECM every cycle.
  private: void SyncTags(
      const ignition::gazebo::EntityComponentManager &_ecm)
  {
    _ecm.EachNew<ignition::gazebo::components::Model,
                 ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_e,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *_name) -> bool
        {
          if (_name->Data().rfind(this->tagPrefix, 0) == 0)
          {
            TagInfo ti;
            ti.name = _name->Data();
            ti.offsetDb = this->DrawTagOffset(ti.name);
            this->tags[_e] = ti;
            if (this->tagsSeeded)
            {
              ignmsg << "[RfidReader] Tag '" << ti.name
                     << "' appeared (offset=" << ti.offsetDb << "dB).\n";
            }
          }
          return true;
        });

    _ecm.EachRemoved<ignition::gazebo::components::Model,
                     ignition::gazebo::components::Name>(
        [&](const ignition::gazebo::Entity &_e,
            const ignition::gazebo::components::Model *,
            const ignition::gazebo::components::Name *_name) -> bool
        {
          if (this->tags.erase(_e) > 0)
          {
            ignmsg << "[RfidReader] Tag '" << _name->Data()
                   << "' removed.\n";
          }
          return true;
        });

    if (!this->tagsSeeded)
    {
      // EachNew only reports entities created this iteration, so the tags
      // that existed at world load must be swept once.
      _ecm.Each<ignition::gazebo::components::Model,
                ignition::gazebo::components::Name>(
          [&](const ignition::gazebo::Entity &_e,
              const ignition::gazebo::components::Model *,
              const ignition::gazebo::components::Name *_name) -> bool
          {
            if (_name->Data().rfind(this->tagPrefix, 0) == 0 &&
                this->tags.find(_e) == this->tags.end())
            {
              TagInfo ti;
              ti.name = _name->Data();
              ti.offsetDb = this->DrawTagOffset(ti.name);
              this->tags[_e] = ti;
            }
            return true;
          });
      this->tagsSeeded = true;
      ignmsg << "[RfidReader] Found " << this->tags.size()
             << " tags with prefix '" << this->tagPrefix << "'.\n";
      if (this->tagOffsetStddev > 0.0)
      {
        for (const auto &[e, ti] : this->tags)
        {
          ignmsg << "[RfidReader]   tag '" << ti.name << "' offset="
                 << ti.offsetDb << "dB\n";
        }
      }
    }
  }

  public: void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (!this->model.Valid(_ecm) || this->antennas.empty())
      return;

    // Every iteration, not on the read cadence: EachNew/EachRemoved report
    // only THIS iteration's changes, so throttling here would miss spawns.
    this->SyncTags(_ecm);

    if (_info.paused)
      return;

    const std::chrono::duration<double> simTime(_info.simTime);
    if (simTime - this->lastUpdate < this->period)
      return;
    this->lastUpdate = simTime;

    if (this->tags.empty())
      return;

    // Snapshot commanded powers once per cycle under lock, so every
    // detection in this report is attributable to one definite power.
    // The report echo, not the command, is the authoritative record.
    std::vector<double> power(this->antennas.size());
    {
      std::lock_guard<std::mutex> lock(this->powerMutex);
      for (size_t i = 0; i < this->antennas.size(); ++i)
        power[i] = this->antennas[i].powerDbm;
    }

    // Port schedule: TDM round-robin, or all ports at once.
    std::vector<size_t> activePorts;
    if (this->multiplex)
      activePorts.push_back(this->cycleCount % this->antennas.size());
    else
      for (size_t i = 0; i < this->antennas.size(); ++i)
        activePorts.push_back(i);
    const size_t thisCycle = this->cycleCount;
    ++this->cycleCount;

    struct Detection
    {
      std::string tagName;
      std::string antennaName;
      std::string antennaFrame;
      double range;
      double rssi;
      double powerDbm;
      ignition::math::Pose3d relPose;
    };
    std::vector<Detection> detections;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    std::normal_distribution<double> rssiNoise(0.0, this->rssiNoiseStddev);

    ignition::msgs::Pose_V tfMsg;

    // Broadcast ALL antenna frames every cycle so they never go stale.
    std::vector<ignition::math::Pose3d> antennaWorld(this->antennas.size());
    std::vector<bool> antennaOk(this->antennas.size(), false);
    for (size_t i = 0; i < this->antennas.size(); ++i)
    {
      auto &a = this->antennas[i];
      if (a.link == ignition::gazebo::kNullEntity)
      {
        // A bad <link> does NOT fail loudly: warn once, skip this antenna
        // every cycle, antennaOk stays false and it never interrogates —
        // reads keep publishing empty detections, indistinguishable from an
        // empty rack. Check this warning before trusting a run of no reads.
        a.link = this->model.LinkByName(_ecm, a.linkName);
        if (a.link == ignition::gazebo::kNullEntity)
        {
          if (!a.warned)
          {
            // URDF fixed-joint children (virtual hand/tool0 frames) are
            // lumped into their parent link by URDF->SDF and won't appear.
            std::string available;
            for (const auto &l : this->model.Links(_ecm))
            {
              auto *n =
                  _ecm.Component<ignition::gazebo::components::Name>(l);
              if (n)
                available += " '" + n->Data() + "'";
            }
            ignwarn << "[RfidReader] Antenna '" << a.name << "': link '"
                    << a.linkName << "' not found in model '"
                    << this->model.Name(_ecm)
                    << "' (will keep trying). Available links:"
                    << available << "\n";
            a.warned = true;
          }
          continue;
        }
      }
      antennaWorld[i] =
          ignition::gazebo::worldPose(a.link, _ecm) * a.pose;
      antennaOk[i] = true;

      if (this->publishTf)
      {
        this->AddTransform(tfMsg, a.parentFrame,
            "rfid_antenna_" + a.name, a.pose, simTime);
      }
    }

    // ---- interrogate active ports ---------------------------------------
    for (const size_t portIdx : activePorts)
    {
      if (!antennaOk[portIdx])
        continue;
      const auto &a = this->antennas[portIdx];
      const auto &aWorld = antennaWorld[portIdx];

      const double dP = power[portIdx] - this->pRef;

      for (const auto &[tagEntity, tag] : this->tags)
      {
        const ignition::math::Pose3d tagWorld =
            ignition::gazebo::worldPose(tagEntity, _ecm);

        const ignition::math::Vector3d relPos =
            aWorld.Rot().RotateVectorReverse(
                tagWorld.Pos() - aWorld.Pos());

        const double range = relPos.Length();
        if (range < 1e-3 || range > this->maxRange)
          continue;

        const double thetaH = std::atan2(relPos.Y(), relPos.X());
        if (std::abs(thetaH) >= M_PI_2)   // main lobe only
          continue;
        const double thetaV = std::atan2(
            relPos.Z(),
            std::sqrt(relPos.X() * relPos.X() + relPos.Y() * relPos.Y()));
        if (std::abs(thetaV) >= M_PI_2)   // gate both axes symmetrically
          continue;

        const double aH = 2.0 * thetaH / a.dThetaH;
        const double aV = 2.0 * thetaV / a.dThetaV;
        const double gain = std::exp2(-(aH * aH + aV * aV));

        // Occlusion [dB] on the direct path: charged once to the forward
        // link, twice to RSSI (backscatter recrosses the same material).
        const double occDb = this->occluders.empty() ? 0.0
            : this->PathAttenuationDb(aWorld.Pos(), tagWorld.Pos(), _ecm);

        const double r0Eff =
            a.r0 * std::pow(10.0, (dP + tag.offsetDb - occDb) / 20.0);

        const double pd = std::clamp(
            0.5 * (r0Eff * r0Eff) / (range * range) * gain,
            0.0, this->pdMax);

        if (uniform(this->rng) >= pd)
          continue;

        // Anchored at the REFERENCE geometry a.r0: power, tag and occlusion
        // terms are added explicitly, so using r0Eff here double-counts them.
        const double rssi = a.rssiR0
            + dP
            + tag.offsetDb
            - 2.0 * occDb
            - 40.0 * std::log10(range / a.r0)
            + 2.0 * 10.0 * std::log10(gain)
            + (this->rssiNoiseStddev > 0.0 ? rssiNoise(this->rng) : 0.0);

        const ignition::math::Quaterniond relRot =
            aWorld.Rot().Inverse() * tagWorld.Rot();

        detections.push_back({tag.name, a.name, "rfid_antenna_" + a.name,
            range, rssi, power[portIdx],
            ignition::math::Pose3d(relPos, relRot)});
      }
    }

    // ---- JSON read report -------------------------------------------------
    char buf[256];
    std::snprintf(buf, sizeof(buf), "{\"t\":%.6f,\"cycle\":%zu,",
        simTime.count(), thisCycle);
    std::string json = buf;
    json += "\"reader\":\"" + this->readerId + "\",\"active\":[";
    // Each active port carries the power it was interrogated at; this is
    // what makes an empty "detected" list interpretable downstream.
    for (size_t i = 0; i < activePorts.size(); ++i)
    {
      std::snprintf(buf, sizeof(buf), "{\"name\":\"%s\",\"power_dbm\":%.2f}",
          this->antennas[activePorts[i]].name.c_str(),
          power[activePorts[i]]);
      json += buf;
      if (i + 1 < activePorts.size())
        json += ",";
    }
    json += "],\"detected\":[";
    for (size_t i = 0; i < detections.size(); ++i)
    {
      // "r" is ground truth with no hardware counterpart — validation only.
      std::snprintf(buf, sizeof(buf),
          "{\"id\":\"%s\",\"r\":%.3f,\"rssi\":%.1f,\"antenna\":\"%s\","
          "\"power_dbm\":%.2f}",
          detections[i].tagName.c_str(), detections[i].range,
          detections[i].rssi, detections[i].antennaName.c_str(),
          detections[i].powerDbm);
      json += buf;
      if (i + 1 < detections.size())
        json += ",";
    }
    json += "]";
    // Ground truth with no hardware counterpart — validation only.
    if (this->reportNTags)
      json += ",\"n_tags\":" + std::to_string(this->tags.size());
    json += "}";

    ignition::msgs::StringMsg msg;
    msg.set_data(json);
    this->pub.Publish(msg);

    // ---- TF: detected tags parented to the antenna that read them ---------
    if (!this->publishTf)
      return;

    std::normal_distribution<double> gauss(0.0, this->tfNoiseStddev);
    for (const auto &det : detections)
    {
      ignition::math::Pose3d p = det.relPose;
      if (this->tfNoiseStddev > 0.0)
      {
        p.Pos() += ignition::math::Vector3d(
            gauss(this->rng), gauss(this->rng), gauss(this->rng));
      }
      this->AddTransform(tfMsg, det.antennaFrame, det.tagName, p, simTime);
    }

    this->tfPub.Publish(tfMsg);
  }

  /// \brief Append one stamped transform, using the frame_id /
  /// child_frame_id header-data convention ros_gz_bridge's
  /// Pose_V <-> tf2_msgs/TFMessage conversion expects.
  private: void AddTransform(ignition::msgs::Pose_V &_msg,
      const std::string &_frame, const std::string &_child,
      const ignition::math::Pose3d &_pose,
      const std::chrono::duration<double> &_t) const
  {
    auto *pose = _msg.add_pose();
    pose->set_name(_child);

    auto *header = pose->mutable_header();
    const auto sec = static_cast<int64_t>(_t.count());
    header->mutable_stamp()->set_sec(sec);
    header->mutable_stamp()->set_nsec(
        static_cast<int32_t>((_t.count() - static_cast<double>(sec)) * 1e9));

    auto *frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(_frame);
    auto *child = header->add_data();
    child->set_key("child_frame_id");
    child->add_value(_child);

    ignition::msgs::Set(pose, _pose);
  }

  // ---- state ----------------------------------------------------------
  private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  private: std::vector<Antenna> antennas;
  private: std::vector<Occluder> occluders;
  private: std::unordered_map<ignition::gazebo::Entity, TagInfo> tags;
  private: bool tagsSeeded{false};
  private: size_t cycleCount{0};

  private: std::string readerId{"vulcan_iron"};
  private: std::string tagPrefix{"rfid_tag_"};
  private: double updateRate{10.0};
  private: bool multiplex{true};
  private: double maxRange{0.5};      ///< compute guard, not a range spec
  private: double pdMax{0.99};
  private: bool publishTf{true};
  private: double tfNoiseStddev{0.0};

  // Power defaults: Vulcan Iron datasheet, 0..27 dBm in 0.5 dB steps.
  private: double pRef{27.0};              ///< reference TX power [dBm]
  private: double pMin{0.0};               ///< command clamp [dBm]
  private: double pMax{27.0};
  private: double powerStep{0.5};          ///< command quantization [dB]
  private: double rssiNoiseStddev{0.0};    ///< per-read fading [dB]
  private: double tagOffsetStddev{0.0};    ///< per-tag spread [dB]
  private: bool reportNTags{true};
  private: unsigned int seedUsed{0};
  private: std::mutex powerMutex;          ///< guards Antenna::powerDbm

  private: std::chrono::duration<double> period{0.1};
  private: std::chrono::duration<double> lastUpdate{-1.0};

  private: std::mt19937 rng;
  private: ignition::transport::Node node;
  private: ignition::transport::Node::Publisher pub;
  private: ignition::transport::Node::Publisher tfPub;
};

}  // namespace rfid

IGNITION_ADD_PLUGIN(
    rfid::RfidReader,
    ignition::gazebo::System,
    rfid::RfidReader::ISystemConfigure,
    rfid::RfidReader::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(rfid::RfidReader, "rfid::RfidReader")