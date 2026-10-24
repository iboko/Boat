// Gazebo Harmonic only: optional, experimental external loads on BlueBoat.
// Not a calibrated hydrodynamic/current/wind model. No actuator commands.
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <sdf/Element.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/math/Vector3.hh>

namespace boat {
namespace {
using Vec = gz::math::Vector3d;

Vec quadratic(const Vec &velocity) {
  return velocity * velocity.Length();
}

Vec disturbance(const Vec &velocity, const Vec &flow, double coefficient) {
  // A physically passive relative-flow drag term. Crucially, when the
  // hull outruns the flow this must oppose its velocity; subtracting a
  // zero-flow reference instead led to an unstable accelerating force.
  // Existing Wave Sim hull drag also remains active; coefficients here
  // are experimental, not a calibrated decomposition of water resistance.
  return coefficient * quadratic(flow - velocity);
}

double parameter(const std::shared_ptr<const sdf::Element> &sdf,
                 const std::string &name, double bound) {
  if (!sdf->HasElement(name))
    throw std::runtime_error("missing " + name);
  const double number = sdf->Get<double>(name);
  if (!std::isfinite(number) || std::abs(number) > bound)
    throw std::runtime_error("invalid " + name);
  return number;
}
}  // namespace

class EnvironmentForces final : public gz::sim::System,
                                public gz::sim::ISystemConfigure,
                                public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override {
    this->world_ = entity;
    try {
      this->current_.Set(parameter(sdf, "current_x", 1.5),
                         parameter(sdf, "current_y", 1.5), 0.0);
      this->wind_.Set(parameter(sdf, "wind_x", 15.0),
                      parameter(sdf, "wind_y", 15.0), 0.0);
      this->currentCoefficient_ = parameter(sdf, "current_coefficient", 300.0);
      this->windCoefficient_ = parameter(sdf, "wind_coefficient", 10.0);
      if (this->currentCoefficient_ <= 0 || this->windCoefficient_ <= 0 ||
          (this->current_.Length() == 0 && this->wind_.Length() == 0))
        throw std::runtime_error("missing positive force or nonzero flow");
    } catch (const std::exception &error) {
      std::cerr << "BOAT_ENV_FORCES_CONFIG_FAILED " << error.what() << '\n';
      this->invalid_ = true;
      return;
    }
    std::cout << "BOAT_ENV_FORCES_CONFIGURED current=" << this->current_
              << " wind=" << this->wind_ << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (info.paused || this->invalid_)
      return;
    if (this->link_ == gz::sim::kNullEntity) {
      gz::sim::World world(this->world_);
      const auto modelEntity = world.ModelByName(ecm, "blueboat");
      if (modelEntity == gz::sim::kNullEntity)
        return;
      const auto linkEntity = gz::sim::Model(modelEntity).LinkByName(ecm, "base_link");
      if (linkEntity == gz::sim::kNullEntity)
        return;
      this->link_ = linkEntity;
      gz::sim::Link(this->link_).EnableVelocityChecks(ecm);
    }
    gz::sim::Link link(this->link_);
    const auto velocity = link.WorldLinearVelocity(ecm);
    if (!velocity)
      return;
    Vec horizontal(velocity->X(), velocity->Y(), 0.0);
    const Vec force = disturbance(horizontal, this->current_, this->currentCoefficient_) +
                      disturbance(horizontal, this->wind_, this->windCoefficient_);
    if (!std::isfinite(force.X()) || !std::isfinite(force.Y())) {
      if (!this->invalid_)
        std::cerr << "BOAT_ENV_FORCES_INVALID_VELOCITY\n";
      this->invalid_ = true;
      return;
    }
    // Force in the world XY plane at hull COM; no artificial yaw moment.
    link.AddWorldForce(ecm, force);
    if (!this->reported_) {
      this->reported_ = true;
      std::cout << "BOAT_ENV_FORCES_APPLIED force=" << force
                << " velocity=" << horizontal << std::endl;
    }
  }

 private:
  gz::sim::Entity world_{gz::sim::kNullEntity};
  gz::sim::Entity link_{gz::sim::kNullEntity};
  Vec current_{0, 0, 0};
  Vec wind_{0, 0, 0};
  double currentCoefficient_{0};
  double windCoefficient_{0};
  bool reported_{false};
  bool invalid_{false};
};
}  // namespace boat

GZ_ADD_PLUGIN(boat::EnvironmentForces, gz::sim::System,
              boat::EnvironmentForces::ISystemConfigure,
              boat::EnvironmentForces::ISystemPreUpdate)
