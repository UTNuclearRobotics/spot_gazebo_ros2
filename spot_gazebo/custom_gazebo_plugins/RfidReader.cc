/*
 * RfidReader — multi-port RFID reader system plugin for Gazebo Ignition
 * Fortress (ign-gazebo 6.x).
 *
 * Models a reader in the sense of the paper's hardware architecture
 * (Alajami et al., IEEE RFID-TA 2022, Sec. IV-A Block 3): one reader
 * (e.g. Keonn AdvanReader-160) driving N antenna ports (e.g. Keonn SP11),
 * each antenna mounted on a different robot link with its own pose offset
 * and RF characteristics. Ports are time-division multiplexed by default,
 * as in the real hardware: each read cycle interrogates one port,
 * round-robin.
 *
 * Because antenna poses are resolved from the ECM every cycle, antennas on
 * ARTICULATED links (e.g. Spot's arm end effector) automatically track the
 * kinematic chain — point the arm, and the read zone follows.
 *
 * Per-antenna PD model and passive-tag semantics are identical to
 * RfidPdReader.cc (see that file / README for derivations):
 *     G  = 2^-[ (2*thetaH/dThetaH)^2 + (2*thetaV/dThetaV)^2 ]
 *     PD = clamp( 0.5 * (r0/R)^2 * G , 0 , pd_max )
 *     RSSI = rssi_r0 - 40*log10(R/r0) + 2*10*log10(G)      (1/R^4 backscatter)
 * Detected tags get TF frames parented to the antenna that read them;
 * undetected tags have no frames (passive tags exist to the system only
 * while energized).
 *
 * SDF configuration (attach as MODEL plugin to the robot):
 *
 *   <plugin filename="RfidReader" name="rfid::RfidReader">
 *     <reader_id>advanreader_160</reader_id>
 *     <tag_prefix>rfid_tag_</tag_prefix>
 *     <update_rate>10</update_rate>        port interrogation cycles [Hz]
 *     <multiplex>true</multiplex>          round-robin ports (false = all
 *                                          ports every cycle)
 *     <topic>/rfid/reads</topic>
 *     <publish_tf>true</publish_tf>
 *     <tf_topic>/rfid/tf</tf_topic>
 *     <rssi_r0>-60.0</rssi_r0>             default; per-antenna override
 *     <tf_position_noise_stddev>0.0</tf_position_noise_stddev>
 *     <max_range>10.0</max_range>
 *     <pd_max>0.99</pd_max>
 *     <seed>42</seed>
 *
 *     <antenna>                            repeat per port (1..N)
 *       <name>body_front</name>            TF frame: rfid_antenna_<name>
 *       <link>body</link>
 *       <pose>0.4 0 0.05 0 0 0</pose>      boresight = +X of this frame
 *       <parent_frame>body</parent_frame>  optional; defaults to <link>
 *       <r0>2.5</r0>
 *       <delta_theta_h>65</delta_theta_h>
 *       <delta_theta_v>65</delta_theta_v>
 *       <rssi_r0>-60.0</rssi_r0>           optional override
 *     </antenna>
 *   </plugin>
 *
 * JSON output per cycle on <topic>:
 *   {"t":12.4,"reader":"advanreader_160","active":["hand"],
 *    "detected":[{"id":"rfid_tag_ups","r":0.61,"rssi":-48.2,
 *                 "antenna":"hand"}],
 *    "n_tags":10}
 */

#include <chrono>
#include <cmath>
#include <cstdio>
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
    double r0{2.5};
    double dThetaH{65.0 * M_PI / 180.0};
    double dThetaV{65.0 * M_PI / 180.0};
    double rssiR0{-60.0};
    ignition::gazebo::Entity link{ignition::gazebo::kNullEntity};
    bool warned{false};
  };

  // ------------------------------------------------------------------ //
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

    const double defaultRssiR0 =
        _sdf->Get<double>("rssi_r0", -60.0).first;

    const std::string topic =
        _sdf->Get<std::string>("topic", "/rfid/reads").first;
    const std::string tfTopic =
        _sdf->Get<std::string>("tf_topic", "/rfid/tf").first;

    const unsigned int seed = _sdf->Get<unsigned int>(
        "seed", std::random_device{}()).first;
    this->rng.seed(seed);

    if (this->updateRate <= 0.0)
      this->updateRate = 10.0;
    this->period = std::chrono::duration<double>(1.0 / this->updateRate);

    // ---- antenna ports --------------------------------------------------
    // Clone for mutable traversal of repeated <antenna> elements
    // (GetElement/GetNextElement are non-const in libsdformat).
    auto sdfClone = _sdf->Clone();
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
        a.r0 = elem->Get<double>("r0", 2.5).first;
        a.dThetaH = elem->Get<double>("delta_theta_h", 65.0).first
            * M_PI / 180.0;
        a.dThetaV = elem->Get<double>("delta_theta_v", 65.0).first
            * M_PI / 180.0;
        a.rssiR0 = elem->Get<double>("rssi_r0", defaultRssiR0).first;
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

    ignmsg << "[RfidReader] '" << this->readerId << "' with "
           << this->antennas.size() << " antenna port(s), "
           << (this->multiplex ? "multiplexed" : "simultaneous")
           << ", rate=" << this->updateRate << "Hz, topic=" << topic
           << ", tf=" << (this->publishTf ? tfTopic : "off") << "\n";
    for (const auto &a : this->antennas)
    {
      ignmsg << "[RfidReader]   port '" << a.name << "' on link '"
             << a.linkName << "' r0=" << a.r0 << "\n";
    }
  }

  // ------------------------------------------------------------------ //
  public: void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->model.Valid(_ecm) || this->antennas.empty())
      return;

    const std::chrono::duration<double> simTime(_info.simTime);
    if (simTime - this->lastUpdate < this->period)
      return;
    this->lastUpdate = simTime;

    // Lazily enumerate tag models once (tags are static world models).
    if (!this->tagsCached)
    {
      _ecm.Each<ignition::gazebo::components::Model,
                ignition::gazebo::components::Name>(
          [&](const ignition::gazebo::Entity &_e,
              const ignition::gazebo::components::Model *,
              const ignition::gazebo::components::Name *_name) -> bool
          {
            if (_name->Data().rfind(this->tagPrefix, 0) == 0)
              this->tags[_e] = _name->Data();
            return true;
          });
      this->tagsCached = true;
      ignmsg << "[RfidReader] Found " << this->tags.size()
             << " tags with prefix '" << this->tagPrefix << "'.\n";
    }
    if (this->tags.empty())
      return;

    // Port schedule: TDM round-robin (real reader behavior) or all ports.
    std::vector<size_t> activePorts;
    if (this->multiplex)
      activePorts.push_back(this->cycleCount % this->antennas.size());
    else
      for (size_t i = 0; i < this->antennas.size(); ++i)
        activePorts.push_back(i);
    ++this->cycleCount;

    struct Detection
    {
      std::string tagName;
      std::string antennaName;
      std::string antennaFrame;
      double range;
      double rssi;
      ignition::math::Pose3d relPose;
    };
    std::vector<Detection> detections;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    ignition::msgs::Pose_V tfMsg;

    // Resolve poses and broadcast ALL antenna frames every cycle (frames
    // are geometry and should never go stale, even for inactive ports).
    std::vector<ignition::math::Pose3d> antennaWorld(this->antennas.size());
    std::vector<bool> antennaOk(this->antennas.size(), false);
    for (size_t i = 0; i < this->antennas.size(); ++i)
    {
      auto &a = this->antennas[i];
      if (a.link == ignition::gazebo::kNullEntity)
      {
        a.link = this->model.LinkByName(_ecm, a.linkName);
        if (a.link == ignition::gazebo::kNullEntity)
        {
          if (!a.warned)
          {
            // List what actually exists: URDF fixed-joint children (e.g.
            // virtual hand/tool0 frames) are lumped into their parent link
            // during URDF->SDF conversion and won't appear here.
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
      // Resolved from the ECM every cycle: articulated links (arm end
      // effector) automatically carry the antenna through the chain.
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

      for (const auto &[tagEntity, tagName] : this->tags)
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

        const double aH = 2.0 * thetaH / a.dThetaH;
        const double aV = 2.0 * thetaV / a.dThetaV;
        const double gain = std::exp2(-(aH * aH + aV * aV));

        const double pd = std::clamp(
            0.5 * (a.r0 * a.r0) / (range * range) * gain,
            0.0, this->pdMax);

        if (uniform(this->rng) >= pd)
          continue;

        const double rssi = a.rssiR0
            - 40.0 * std::log10(range / a.r0)
            + 2.0 * 10.0 * std::log10(gain);

        const ignition::math::Quaterniond relRot =
            aWorld.Rot().Inverse() * tagWorld.Rot();

        detections.push_back({tagName, a.name, "rfid_antenna_" + a.name,
            range, rssi, ignition::math::Pose3d(relPos, relRot)});
      }
    }

    // ---- JSON read report -------------------------------------------------
    char buf[192];
    std::string json = "{\"t\":" + std::to_string(simTime.count()) +
        ",\"reader\":\"" + this->readerId + "\",\"active\":[";
    for (size_t i = 0; i < activePorts.size(); ++i)
    {
      json += "\"" + this->antennas[activePorts[i]].name + "\"";
      if (i + 1 < activePorts.size())
        json += ",";
    }
    json += "],\"detected\":[";
    for (size_t i = 0; i < detections.size(); ++i)
    {
      std::snprintf(buf, sizeof(buf),
          "{\"id\":\"%s\",\"r\":%.3f,\"rssi\":%.1f,\"antenna\":\"%s\"}",
          detections[i].tagName.c_str(), detections[i].range,
          detections[i].rssi, detections[i].antennaName.c_str());
      json += buf;
      if (i + 1 < detections.size())
        json += ",";
    }
    json += "],\"n_tags\":" + std::to_string(this->tags.size()) + "}";

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

  // ------------------------------------------------------------------ //
  /// \brief Append one stamped transform (frame_id / child_frame_id
  /// header-data convention understood by ros_gz_bridge's
  /// Pose_V <-> tf2_msgs/TFMessage conversion).
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
  private: std::unordered_map<ignition::gazebo::Entity, std::string> tags;
  private: bool tagsCached{false};
  private: size_t cycleCount{0};

  private: std::string readerId{"rfid_reader"};
  private: std::string tagPrefix{"rfid_tag_"};
  private: double updateRate{10.0};
  private: bool multiplex{true};
  private: double maxRange{10.0};
  private: double pdMax{0.99};
  private: bool publishTf{true};
  private: double tfNoiseStddev{0.0};

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