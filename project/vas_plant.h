#pragma once

#include <memory>

#include "drake/math/rigid_transform.h"
#include "drake/common/drake_copyable.h"
#include "drake/geometry/geometry_ids.h"
#include "drake/geometry/query_object.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"

// #include "drake/multibody/tree/ball_rpy_joint.h"

// #include "drake/multibody/tree/revolute_joint.h"
#include "drake/multibody/tree/prismatic_joint.h"

// #include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/linear_bushing_roll_pitch_yaw.h"

namespace drake {
namespace project {

using Eigen::MatrixXi;
using Eigen::Vector3d;

class VolumetricActuatorSystem : public systems::LeafSystem<double> {
  public:
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(VolumetricActuatorSystem)

    VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant);
    VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant, const std::vector<Vector3d>& shape_arg);
    VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant, double m_arg, const std::vector<Vector3d>& shape_arg,
      double r_cl_arg, double r_op_arg);

    multibody::ModelInstanceIndex AddVASToPlant();

    void AddConnect(const std::string body0_name, const std::string body1_name,
      const double k_xyz, const double d_xyz, const double k_012, const double d_012);
    void AddCollide(const std::string body_name,
      const double static_friction, const double dynamic_friction);
    void InitPose(systems::Context<double>* context);
    std::vector<double> GetJointTranslations(systems::Context<double>* context);

  private:

    int HasNeighbor(unsigned int i, Vector3d direction);
    std::string Neighboring(unsigned int i_1, unsigned int i_2);
    void ShowAxes(std::string bodyname, double scale);

    multibody::MultibodyPlant<double>* plant_;
    double m_;
    std::vector<Vector3d> shape_;
    double r_cl_;
    double r_op_;
    multibody::ModelInstanceIndex mii_;
};

}  // namespace examples
}  // namespace drake
