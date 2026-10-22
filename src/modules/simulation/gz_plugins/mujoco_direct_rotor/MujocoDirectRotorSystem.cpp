#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <vector>

// MuJoCo SITE transmission: scalar actuator force multiplies its six-axis
// gear. No rotor speed, motor lag, or invented aerodynamic constants.
class MujocoDirectRotorSystem final : public gz::sim::System,
  public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  struct Rotor { unsigned channel; gz::math::Vector3d point, axis;
    double force, moment; };
  gz::sim::Entity link{gz::sim::kNullEntity};
  gz::transport::Node node;
  gz::transport::Node::Publisher telemetry;
  std::mutex mutex;
  std::vector<double> command;
  std::vector<Rotor> rotors;
  double scale{1}, maximum{130};
public:
  void Configure(const gz::sim::Entity &entity,
    const std::shared_ptr<const sdf::Element> &config,
    gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    auto sdf = config->Clone();
    link = gz::sim::Model(entity).LinkByName(ecm, sdf->Get<std::string>("link_name"));
    if (link == gz::sim::kNullEntity) throw std::runtime_error("Direct rotor link not found");
    scale = sdf->Get<double>("input_scale", 1).first;
    maximum = sdf->Get<double>("ctrl_max", 130).first;
    for (auto r = sdf->GetElement("rotor"); r; r = r->GetNextElement("rotor")) {
      Rotor rotor{r->Get<unsigned>("channel"), r->Get<gz::math::Vector3d>("position"),
        r->Get<gz::math::Vector3d>("axis"), r->Get<double>("force_coefficient"),
        r->Get<double>("moment_coefficient")};
      if (std::abs(rotor.axis.Length()-1) > 1e-9) throw std::runtime_error("Rotor axis must be unit length");
      rotors.push_back(rotor);
    }
    const auto topic = sdf->Get<std::string>("topic",
      "/" + gz::sim::Model(entity).Name(ecm) + "/command/motor_speed").first;
    telemetry = node.Advertise<gz::msgs::Double_V>(topic + "/applied");
    if (!node.Subscribe(topic, &MujocoDirectRotorSystem::OnCommand, this))
      throw std::runtime_error("Direct rotor topic subscription failed");
  }
  void OnCommand(const gz::msgs::Actuators &message) {
    std::lock_guard<std::mutex> guard(mutex);
    command.assign(message.velocity().begin(), message.velocity().end());
  }
  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    std::vector<double> values;
    { std::lock_guard<std::mutex> guard(mutex); values = command; }
    const auto pose = gz::sim::worldPose(link, ecm);
    gz::msgs::Double_V applied;
    // Each telemetry record is ctrl, world force XYZ, world reaction torque XYZ.
    for (const auto &r : rotors) {
      double ctrl = r.channel < values.size() ? values[r.channel]*scale : 0;
      ctrl = std::isfinite(ctrl) ? std::clamp(ctrl, 0.0, maximum) : 0;
      const auto axis = pose.Rot().RotateVector(r.axis);
      const auto force = axis*(r.force*ctrl);
      const auto torque = axis*(r.moment*ctrl);
      gz::sim::Link(link).AddWorldWrench(ecm, force, torque, r.point);
      applied.add_data(ctrl);
      for (unsigned i=0; i<3; ++i) applied.add_data(force[i]);
      for (unsigned i=0; i<3; ++i) applied.add_data(torque[i]);
    }
    telemetry.Publish(applied);
  }
};
GZ_ADD_PLUGIN(MujocoDirectRotorSystem, gz::sim::System,
  MujocoDirectRotorSystem::ISystemConfigure, MujocoDirectRotorSystem::ISystemPreUpdate)
