/*
 * RfidPdReader — probability-of-detection RFID reader system plugin for
 * Gazebo Ignition Fortress (ign-gazebo 6.x).
 *
 * Implements the PD model described in:
 *   Alajami, Pous, Moreno, "Simulation of RFID Systems in ROS-Gazebo",
 *   IEEE RFID-TA 2022  (and the extended IEEE Access 2022 version).
 *
 * PD model
 * --------
 * Tag position in antenna-frame spherical coordinates (boresight = +X):
 *     G(thetaH, thetaV) = 2^-[ (2*thetaH/dThetaH)^2 + (2*thetaV/dThetaV)^2 ]
 *     PD = clamp( 0.5 * (r0/R)^2 * G , 0 , pd_max )
 * Anchors: PD(r0,0,0) = 0.5; 1/R^2 surface-power decay; PD halves at the
 * -3 dB beamwidth edges. Each read cycle every tag is Bernoulli-sampled.
 *
 * Passive-tag semantics
 * ---------------------
 * Tags are passive: unpowered world models with no plugins. A tag "exists"
 * to the system only while the reader energizes and reads it:
 *   - TF frames for tags are broadcast ONLY on cycles where the tag was
 *     detected, and are parented to the ANTENNA frame (a reader knows tags
 *     relative to itself, never in world coordinates). Undetected tags go
 *     stale in the TF buffer.
 *   - Each detection carries a synthesized backscatter RSSI using the
 *     passive-tag two-way link budget (1/R^4, antenna gain applied twice):
 *         RSSI [dBm] = rssi_r0 - 40*log10(R/r0) + 2 * 10*log10(G)
 *     anchored so RSSI = rssi_r0 at boresight r0.
 *   - Optional Gaussian noise on the broadcast tag position models reader
 *     localization uncertainty (tf_position_noise_stddev, default 0 =
 *     ground truth, matching the paper's convention).
 *
 * Outputs
 * -------
 * 1) <topic> (default /rfid/reads), ignition::msgs::StringMsg, JSON:
 *      {"t":12.4,"detected":[{"id":"rfid_tag_ups","r":1.92,"rssi":-63.4}],
 *       "n_tags":10}
 *    Bridges to std_msgs/String with a stock parameter_bridge.
 * 2) <tf_topic> (default /rfid/tf), ignition::msgs::Pose_V with
 *    frame_id / child_frame_id header data (same convention as gz-sim's
 *    DiffDrive TF output). Bridges to tf2_msgs/TFMessage:
 *      antenna_parent_frame -> antenna_frame   (every cycle)
 *      antenna_frame        -> <tag name>      (detected tags only)
 *
 * Attach as a MODEL plugin on the robot carrying the antenna (see README).
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

class RfidPdReader
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPostUpdate
{
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
      ignerr << "[RfidPdReader] Plugin must be attached to a model.\n";
      return;
    }

    // ---- SDF parameters ------------------------------------------------
    this->antennaLinkName =
        _sdf->Get<std::string>("antenna_link", this->antennaLinkName).first;
    this->antennaPose =
        _sdf->Get<ignition::math::Pose3d>("antenna_pose",
                                          this->antennaPose).first;
    this->tagPrefix =
        _sdf->Get<std::string>("tag_prefix", this->tagPrefix).first;
    this->r0        = _sdf->Get<double>("r0", this->r0).first;
    this->maxRange  = _sdf->Get<double>("max_range", this->maxRange).first;
    this->pdMax     = _sdf->Get<double>("pd_max", this->pdMax).first;
    this->updateRate = _sdf->Get<double>("update_rate",
                                         this->updateRate).first;

    const double dThetaHDeg =
        _sdf->Get<double>("delta_theta_h", 65.0).first;
    const double dThetaVDeg =
        _sdf->Get<double>("delta_theta_v", 65.0).first;
    this->dThetaH = dThetaHDeg * M_PI / 180.0;
    this->dThetaV = dThetaVDeg * M_PI / 180.0;

    // Passive-tag / TF parameters
    this->publishTf =
        _sdf->Get<bool>("publish_tf", this->publishTf).first;
    this->antennaFrameId =
        _sdf->Get<std::string>("antenna_frame_id",
                               this->antennaFrameId).first;
    // Parent ROS frame of the antenna; defaults to the antenna link name,
    // which matches robot_state_publisher frame naming for the sim model.
    this->antennaParentFrameId =
        _sdf->Get<std::string>("antenna_parent_frame_id",
                               this->antennaLinkName).first;
    this->rssiR0 = _sdf->Get<double>("rssi_r0", this->rssiR0).first;
    this->tfNoiseStddev =
        _sdf->Get<double>("tf_position_noise_stddev",
                          this->tfNoiseStddev).first;

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

    this->pub = this->node.Advertise<ignition::msgs::StringMsg>(topic);
    if (this->publishTf)
      this->tfPub = this->node.Advertise<ignition::msgs::Pose_V>(tfTopic);

    ignmsg << "[RfidPdReader] antenna_link=" << this->antennaLinkName
           << " r0=" << this->r0
           << " beamwidths(deg)=" << dThetaHDeg << "/" << dThetaVDeg
           << " rate=" << this->updateRate << "Hz"
           << " topic=" << topic
           << " tf=" << (this->publishTf ? tfTopic : "off") << "\n";
  }

  // ------------------------------------------------------------------ //
  public: void PostUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      const ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (_info.paused || !this->model.Valid(_ecm))
      return;

    // Throttle to update_rate (sim time).
    const std::chrono::duration<double> simTime(_info.simTime);
    if (simTime - this->lastUpdate < this->period)
      return;
    this->lastUpdate = simTime;

    // Lazily resolve the antenna link (robust to <include merge="true">
    // and lazy entity creation ordering).
    if (this->antennaLink == ignition::gazebo::kNullEntity)
    {
      this->antennaLink =
          this->model.LinkByName(_ecm, this->antennaLinkName);
      if (this->antennaLink == ignition::gazebo::kNullEntity)
      {
        if (!this->warnedNoLink)
        {
          ignwarn << "[RfidPdReader] Link '" << this->antennaLinkName
                  << "' not found in model '" << this->model.Name(_ecm)
                  << "' (will keep trying).\n";
          this->warnedNoLink = true;
        }
        return;
      }
    }

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
      ignmsg << "[RfidPdReader] Found " << this->tags.size()
             << " tags with prefix '" << this->tagPrefix << "'.\n";
    }
    if (this->tags.empty())
      return;

    // Antenna world pose: link pose composed with the mounting offset.
    const ignition::math::Pose3d antennaWorld =
        ignition::gazebo::worldPose(this->antennaLink, _ecm) *
        this->antennaPose;

    // ---- PD evaluation + Bernoulli sampling per tag --------------------
    struct Detection
    {
      std::string name;
      double range;
      double rssi;
      ignition::math::Pose3d relPose;  // tag pose in antenna frame
    };
    std::vector<Detection> detections;
    std::uniform_real_distribution<double> uniform(0.0, 1.0);

    for (const auto &[tagEntity, tagName] : this->tags)
    {
      const ignition::math::Pose3d tagWorld =
          ignition::gazebo::worldPose(tagEntity, _ecm);

      // Tag pose in the antenna frame (boresight = +X).
      const ignition::math::Vector3d relPos =
          antennaWorld.Rot().RotateVectorReverse(
              tagWorld.Pos() - antennaWorld.Pos());

      const double range = relPos.Length();
      if (range < 1e-3 || range > this->maxRange)
        continue;

      const double thetaH = std::atan2(relPos.Y(), relPos.X());
      // Main-lobe model only: nothing behind the antenna plane is read.
      if (std::abs(thetaH) >= M_PI_2)
        continue;
      const double thetaV = std::atan2(
          relPos.Z(),
          std::sqrt(relPos.X() * relPos.X() + relPos.Y() * relPos.Y()));

      const double gain = this->Gain(thetaH, thetaV);
      const double pd = std::clamp(
          0.5 * (this->r0 * this->r0) / (range * range) * gain,
          0.0, this->pdMax);

      if (uniform(this->rng) >= pd)
        continue;

      // Passive backscatter link budget: 1/R^4 => 40*log10(R), antenna
      // gain applied on both forward and return paths => 2*10*log10(G).
      const double rssi = this->rssiR0
          - 40.0 * std::log10(range / this->r0)
          + 2.0 * 10.0 * std::log10(gain);

      const ignition::math::Quaterniond relRot =
          antennaWorld.Rot().Inverse() * tagWorld.Rot();

      detections.push_back({tagName, range, rssi,
          ignition::math::Pose3d(relPos, relRot)});
    }

    // ---- Publish JSON read report ---------------------------------------
    char buf[128];
    std::string json = "{\"t\":" + std::to_string(simTime.count()) +
                       ",\"detected\":[";
    for (size_t i = 0; i < detections.size(); ++i)
    {
      std::snprintf(buf, sizeof(buf),
          "{\"id\":\"%s\",\"r\":%.3f,\"rssi\":%.1f}",
          detections[i].name.c_str(), detections[i].range,
          detections[i].rssi);
      json += buf;
      if (i + 1 < detections.size())
        json += ",";
    }
    json += "],\"n_tags\":" + std::to_string(this->tags.size()) + "}";

    ignition::msgs::StringMsg msg;
    msg.set_data(json);
    this->pub.Publish(msg);

    // ---- Publish TF (Pose_V -> tf2_msgs/TFMessage via ros_gz_bridge) ----
    if (!this->publishTf)
      return;

    ignition::msgs::Pose_V tfMsg;

    // Antenna frame is broadcast every cycle so it never goes stale.
    this->AddTransform(tfMsg, this->antennaParentFrameId,
        this->antennaFrameId, this->antennaPose, simTime);

    // Detected tags only: a passive tag exists to the system only while
    // the reader energizes it. Parented to the antenna frame.
    std::normal_distribution<double> gauss(0.0, this->tfNoiseStddev);
    for (const auto &det : detections)
    {
      ignition::math::Pose3d p = det.relPose;
      if (this->tfNoiseStddev > 0.0)
      {
        p.Pos() += ignition::math::Vector3d(
            gauss(this->rng), gauss(this->rng), gauss(this->rng));
      }
      this->AddTransform(tfMsg, this->antennaFrameId, det.name, p, simTime);
    }

    this->tfPub.Publish(tfMsg);
  }

  // ------------------------------------------------------------------ //
  /// \brief Normalized main-lobe gain; 0.5 at the -3 dB beamwidth edges.
  private: double Gain(double _thetaH, double _thetaV) const
  {
    const double aH = 2.0 * _thetaH / this->dThetaH;
    const double aV = 2.0 * _thetaV / this->dThetaV;
    return std::exp2(-(aH * aH + aV * aV));
  }

  /// \brief Append one stamped transform to a Pose_V message using the
  /// frame_id / child_frame_id header-data convention understood by
  /// ros_gz_bridge's Pose_V <-> tf2_msgs/TFMessage conversion.
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
  private: ignition::gazebo::Entity antennaLink{
      ignition::gazebo::kNullEntity};
  private: bool warnedNoLink{false};

  private: std::unordered_map<ignition::gazebo::Entity, std::string> tags;
  private: bool tagsCached{false};

  // ---- parameters (defaults are Keonn-SP11-ish placeholders; tune) -----
  private: std::string antennaLinkName{"body"};
  private: ignition::math::Pose3d antennaPose{0, 0, 0, 0, 0, 0};
  private: std::string tagPrefix{"rfid_tag_"};
  private: double r0{2.5};        ///< [m] PD=0.5 boresight distance
  private: double dThetaH{65.0 * M_PI / 180.0};   ///< [rad] -3dB beamwidth
  private: double dThetaV{65.0 * M_PI / 180.0};   ///< [rad] -3dB beamwidth
  private: double maxRange{10.0}; ///< [m] hard cutoff (compute saving)
  private: double pdMax{0.99};    ///< per-cycle PD ceiling
  private: double updateRate{10.0};               ///< [Hz] read cycles

  // ---- passive-tag / TF parameters --------------------------------------
  private: bool publishTf{true};
  private: std::string antennaFrameId{"rfid_antenna"};
  private: std::string antennaParentFrameId{""};
  private: double rssiR0{-60.0};  ///< [dBm] backscatter RSSI at r0, boresight
  private: double tfNoiseStddev{0.0};  ///< [m] per-axis position noise

  private: std::chrono::duration<double> period{0.1};
  private: std::chrono::duration<double> lastUpdate{-1.0};

  private: std::mt19937 rng;
  private: ignition::transport::Node node;
  private: ignition::transport::Node::Publisher pub;
  private: ignition::transport::Node::Publisher tfPub;
};

}  // namespace rfid

IGNITION_ADD_PLUGIN(
    rfid::RfidPdReader,
    ignition::gazebo::System,
    rfid::RfidPdReader::ISystemConfigure,
    rfid::RfidPdReader::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(rfid::RfidPdReader, "rfid::RfidPdReader")
