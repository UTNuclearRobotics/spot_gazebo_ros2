/*
 * JointStateInitializer — set initial joint positions at spawn for
 * Gazebo Ignition Fortress (ign-gazebo 6.x). Attach as a MODEL plugin.
 *
 * Why this exists: neither SDF's //joint/axis/initial_position (parsed but
 * never honored by ign physics) nor JointTrajectoryController's
 * <initial_position> (a PID *target*, not a state — the controller slams the
 * joint there dynamically, kicking the robot) can actually spawn a joint at a
 * nonzero angle. This plugin writes JointPositionReset / JointVelocityReset
 * components on the first unpaused step, which the physics system consumes as
 * a kinematic teleport: no transient, no reaction torque on the body.
 *
 * SDF usage:
 *   <plugin filename="JointStateInitializer"
 *           name="spot_gazebo::JointStateInitializer">
 *     <joint name="arm0_shoulder_pitch">-3.1360</joint>
 *     <joint name="arm0_elbow_pitch">3.1337</joint>
 *     ...
 *   </plugin>
 */

#include <string>
#include <utility>
#include <vector>

#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/JointPositionReset.hh>
#include <ignition/gazebo/components/JointVelocityReset.hh>
#include <ignition/plugin/Register.hh>

namespace spot_gazebo
{

class JointStateInitializer
    : public ignition::gazebo::System,
      public ignition::gazebo::ISystemConfigure,
      public ignition::gazebo::ISystemPreUpdate
{
  public: void Configure(
      const ignition::gazebo::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      ignition::gazebo::EntityComponentManager & /*_ecm*/,
      ignition::gazebo::EventManager & /*_eventMgr*/) override
  {
    this->model = ignition::gazebo::Model(_entity);

    auto sdfClone = _sdf->Clone();
    for (auto elem = sdfClone->GetElement("joint"); elem;
         elem = elem->GetNextElement("joint"))
    {
      if (!elem->HasAttribute("name"))
      {
        ignerr << "[JointStateInitializer] <joint> missing 'name' attribute; "
                  "skipping\n";
        continue;
      }
      this->targets.emplace_back(
          elem->GetAttribute("name")->GetAsString(), elem->Get<double>());
    }
  }

  public: void PreUpdate(
      const ignition::gazebo::UpdateInfo &_info,
      ignition::gazebo::EntityComponentManager &_ecm) override
  {
    if (this->done)
      return;

    // (Re)assert the resets every step until the first unpaused one, so the
    // values survive a paused GUI start; physics consumes them on the first
    // real step and we never touch the joints again.
    for (const auto &[name, position] : this->targets)
    {
      auto joint = this->model.JointByName(_ecm, name);
      if (joint == ignition::gazebo::kNullEntity)
      {
        ignerr << "[JointStateInitializer] joint '" << name
               << "' not found in model '" << this->model.Name(_ecm) << "'\n";
        continue;
      }
      _ecm.SetComponentData<
          ignition::gazebo::components::JointPositionReset>(joint, {position});
      _ecm.SetComponentData<
          ignition::gazebo::components::JointVelocityReset>(joint, {0.0});
    }

    if (!_info.paused)
      this->done = true;
  }

  private: ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
  private: std::vector<std::pair<std::string, double>> targets;
  private: bool done{false};
};

}  // namespace spot_gazebo

IGNITION_ADD_PLUGIN(
    spot_gazebo::JointStateInitializer,
    ignition::gazebo::System,
    spot_gazebo::JointStateInitializer::ISystemConfigure,
    spot_gazebo::JointStateInitializer::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
    spot_gazebo::JointStateInitializer, "JointStateInitializer")
