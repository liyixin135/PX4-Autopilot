#include <set>
#include <fstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/CollisionBitmask.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/common/Console.hh>
#include <cstdlib>

// DART implements category/mask OR semantics. Mesh loading in gz-sim bypasses
// the SDF category field, so use the official Physics command components.
// Bit 0: environment, bit 1: UAV, bit 2: platform, bit 3: other real links.
class SelectiveDeltaCollisionSystem : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
  gz::sim::Entity model{gz::sim::kNullEntity};
  std::set<gz::sim::Entity> configured;
  std::map<std::string, gz::sim::Entity> links, joints;
  std::ofstream probe;
  bool upward{false};
  bool workspaceStop{true};
  bool announced{false};
  double nextSample{0};
  std::map<std::string, std::pair<gz::math::Vector3d,gz::math::Vector3d>> upperAnchors;
  gz::math::Vector2d guideXY{0,0}; bool guideSet{false};
  const std::map<std::string, std::string> uppers{
    {"delta2_lower_arm_4", "delta_elbow_2"},
    {"delta2_lower_arm_5", "delta_elbow_2"},
    {"delta1_lower_arm_1", "delta_elbow_1"},
    {"delta1_lower_arm_2", "delta_elbow_1"},
    {"delta_lower_arm_5", "delta_elbow_3"},
    {"delta_lower_arm_6", "delta_elbow_3"}};
  const std::map<std::string, gz::math::Vector3d> upperAnchorModel{
    {"delta2_lower_arm_4",{0.1745,-0.0262,0}}, {"delta2_lower_arm_5",{0.1745,0.0262,0}},
    {"delta1_lower_arm_1",{0.1745,0.0262,0}}, {"delta1_lower_arm_2",{0.1745,-0.0262,0}},
    {"delta_lower_arm_5",{0.1745,0.0262,0}}, {"delta_lower_arm_6",{0.1745,-0.0262,0}}};
public:
  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    model = entity;
    { std::ofstream marker("/tmp/selective_delta_plugin_loaded", std::ios::app);
      marker << "Configure entity " << entity << "\n"; }
    const char *probeEnv=std::getenv("SELECTIVE_DELTA_PROBE_FILE");
    if (sdf->HasElement("probe_file") || probeEnv) {
      probe.open(probeEnv ? probeEnv : sdf->Get<std::string>("probe_file"));
      upward = probeEnv || sdf->Get<bool>("upward_load", false).first;
      workspaceStop = std::getenv("SELECTIVE_DELTA_ALLOW_WORKSPACE_CROSS") == nullptr;
      probe << "t,force,q1,q2,q3,effort1,effort2,effort3,platform_x,platform_y,platform_z,closure_error,upper_error\n";
      ecm.Each<gz::sim::components::Name, gz::sim::components::ParentEntity>(
        [&](const gz::sim::Entity &e, const auto *n, const auto *p) {
          if (p->Data() != model) return true;
          if (ecm.Component<gz::sim::components::Link>(e)) {
            links[n->Data()] = e;
            gz::sim::Link(e).EnableVelocityChecks(ecm);
          }
          if (ecm.Component<gz::sim::components::Joint>(e)) {
            joints[n->Data()] = e;
            if (!ecm.Component<gz::sim::components::JointPosition>(e))
              ecm.CreateComponent(e, gz::sim::components::JointPosition());
          }
          return true;
        });
    }
  }
  static double Force(double t) {
    const double levels[] = {0,1,2,5,10,20,50,100};
    int i = std::min(7, static_cast<int>(t / 2));
    double previous = i == 0 ? 0 : levels[i-1];
    return previous + (levels[i] - previous) * std::min(1.0, std::fmod(t,2.0) / 0.5);
  }
  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override
  {
    using namespace gz::sim;
    ecm.Each<components::Collision, components::ParentEntity>(
      [&](const Entity &entity, const components::Collision *,
          const components::ParentEntity *parent) {
        auto linkParent = ecm.Component<components::ParentEntity>(parent->Data());
        auto name = ecm.Component<components::Name>(parent->Data());
        if (!linkParent || linkParent->Data() != model || !name ||
            configured.count(entity)) return true;
        const auto &n = name->Data();
        uint16_t category = 8, mask = 1;
        if (n == "delta_end_platform") { category = 4; mask = 3; }
        else if (n.rfind("uav_", 0) == 0 || n == "base_link")
          { category = 2; mask = 5; }
        else if (n.rfind("c1_", 0) == 0 || n.rfind("c2_", 0) == 0 ||
                 n.rfind("c3_", 0) == 0 || n.rfind("c4_", 0) == 0 ||
                 n.rfind("c5_", 0) == 0 || n.find("helper") != std::string::npos ||
                 n.find("dummy") != std::string::npos)
          { category = 0; mask = 0; }
        ecm.CreateComponent(entity, components::CategoryBitmaskCmd(category));
        ecm.CreateComponent(entity, components::CollideBitmaskCmd(mask));
        configured.insert(entity);
        return true;
      });
    if (!announced && !configured.empty()) {
      int u=0,p=0,r=0,h=0,helperLinks=0,dummyLinks=0;
      ecm.Each<components::Link, components::Name, components::ParentEntity>(
        [&](const Entity &, const components::Link *, const components::Name *ne,
            const components::ParentEntity *pe) {
          if (pe->Data()!=model) return true;
          const auto &n=ne->Data();
          if (n.find("dummy")!=std::string::npos) ++dummyLinks;
          else if (n.size()>3 && n[0]=='c' && n[1]>='1' && n[1]<='5' && n[2]=='_') ++helperLinks;
          return true; });
      ecm.Each<components::Collision, components::ParentEntity>(
        [&](const Entity &, const components::Collision *, const components::ParentEntity *pe) {
          auto ne=ecm.Component<components::Name>(pe->Data());
          if (!ne) {
            return true;
          }
          const auto &n = ne->Data();
          if (n=="delta_end_platform") ++p;
          else if (n=="base_link" || n.rfind("uav_",0)==0) ++u;
          else if (n.rfind("c",0)==0 || n.find("dummy")!=std::string::npos) ++h;
          else {
            ++r;
          }
          return true;
        });
      gzmsg << "SelectiveDeltaCollisionSystem loaded: UAV collision count=" << u
            << ", end-platform collision count=" << p
            << ", real Delta collision count=" << r
            << ", helper/dummy collision count=" << h << "\n";
      gzmsg << "Selective collision pairs: UAV<->platform=COLLIDE, helper/dummy=NONE, adjacent Delta=NONE\n";
      announced=true;
      std::ofstream state("/tmp/selective_delta_runtime.json");
      state << "{\n\"plugin_loaded\":true,\n\"model_entity\":"<<model<<",\n"
            << "\"runtime_uav_collision_count\":"<<u<<",\n"
            << "\"runtime_platform_collision_count\":"<<p<<",\n"
            << "\"runtime_real_delta_collision_count\":"<<r<<",\n"
            << "\"runtime_helper_collision_count\":"<<h<<",\n"
            << "\"runtime_dummy_collision_count\":0,\n"
            << "\"runtime_helper_link_count\":"<<helperLinks<<",\n"
            << "\"runtime_dummy_link_count\":"<<dummyLinks<<",\n"
            << "\"collision_pair_policy\":{\"UAV_PLATFORM\":\"ENABLED\",\"UAV_HELPER\":\"DISABLED\",\"PLATFORM_HELPER\":\"DISABLED\",\"DUMMY_ANY\":\"DISABLED\"}\n}\n";
    }
    if (upward && !info.paused && links.count("delta_end_platform")) {
      const auto bp=worldPose(links.at("delta_base"),ecm);
      const auto pp=worldPose(links.at("delta_end_platform"),ecm);
      const auto rel=bp.Inverse()*pp;
      if (!guideSet) { guideXY={rel.Pos().X(),rel.Pos().Y()}; guideSet=true; }
      double zForce=Force(std::chrono::duration<double>(info.simTime).count());
      // Stop the test load before the source boundary; the small margin absorbs
      // momentum from the ramp without constraining or teleporting the model.
      if (workspaceStop && rel.Pos().Z() >= -0.065) zForce=0;
      auto forceLocal=gz::math::Vector3d(
        -800*(rel.Pos().X()-guideXY.X()),-800*(rel.Pos().Y()-guideXY.Y()),
        zForce);
      Link(links.at("delta_end_platform")).AddWorldForce(ecm,bp.Rot().RotateVector(forceLocal));
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &info,
      const gz::sim::EntityComponentManager &ecm) override {
    using namespace gz::sim;
    if (info.paused) return;
    ecm.Each<components::ContactSensorData>([&](const Entity &, const auto *data) {
      for (const auto &c : data->Data().contact()) {
        std::ofstream out("/tmp/selective_delta_contacts.jsonl",std::ios::app);
        out << "{\"iteration\":"<<info.iterations<<",\"collision_a\":\""<<c.collision1().name()
            <<"\",\"collision_b\":\""<<c.collision2().name()<<"\",\"depth\":[";
        for(int i=0;i<c.depth_size();++i) out<<(i?",":"")<<c.depth(i);
        out << "]";
        if (c.position_size()) out << ",\"position\":["<<c.position(0).x()<<','<<c.position(0).y()<<','<<c.position(0).z()<<']';
        if (c.normal_size()) out << ",\"normal\":["<<c.normal(0).x()<<','<<c.normal(0).y()<<','<<c.normal(0).z()<<']';
        out << "}\n";
      } return true; });
    if (!probe.is_open()) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    if (t < nextSample) return;
    nextSample = t + 0.01;
    double closure = 0, upper = 0;
    for (int i=1; i<=5; ++i) {
      auto prefix = "c" + std::to_string(i);
      closure = std::max(closure, (worldPose(links.at(prefix+"_z"),ecm).Pos()
        - worldPose(links.at(prefix+"_dummy"),ecm).Pos()).Length());
    }
    for (const auto &[child,parent] : uppers) {
      const auto cp = worldPose(links.at(child),ecm);
      const auto pp = worldPose(links.at(parent),ecm);
      if (!upperAnchors.count(child)) {
        const auto anchorWorld=worldPose(links.at("delta_base"),ecm).CoordPositionAdd(upperAnchorModel.at(child));
        upperAnchors[child]={pp.Inverse().CoordPositionAdd(anchorWorld),cp.Inverse().CoordPositionAdd(anchorWorld)};
      }
      const auto &a=upperAnchors.at(child);
      upper = std::max(upper,(pp.CoordPositionAdd(a.first)-cp.CoordPositionAdd(a.second)).Length());
    }
    probe << t << ',' << (upward ? Force(t) : 0);
    for (int i=1;i<=3;++i) {
      const auto *q = ecm.Component<components::JointPosition>(joints.at("delta_elbow_"+std::to_string(i)+"_hinge"));
      probe << ',' << (q && !q->Data().empty() ? q->Data()[0] : 0);
    }
    for (int i=1;i<=3;++i) {
      const auto *f = ecm.Component<components::JointForceCmd>(joints.at("delta_elbow_"+std::to_string(i)+"_hinge"));
      probe << ',' << (f && !f->Data().empty() ? f->Data()[0] : 0);
    }
    const auto base = worldPose(links.at("delta_base"),ecm);
    const auto platform = worldPose(links.at("delta_end_platform"),ecm);
    const auto rel = base.Inverse() * platform;
    probe << ',' << rel.Pos().X() << ',' << rel.Pos().Y() << ',' << rel.Pos().Z()
          << ',' << closure << ',' << upper << '\n';
    probe.flush();
  }
};
GZ_ADD_PLUGIN(SelectiveDeltaCollisionSystem, gz::sim::System,
    SelectiveDeltaCollisionSystem::ISystemConfigure,
    SelectiveDeltaCollisionSystem::ISystemPreUpdate,
    SelectiveDeltaCollisionSystem::ISystemPostUpdate)
