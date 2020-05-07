#include "drake/project/vas_plant.h"

namespace drake {
namespace project {

namespace {
std::vector<Vector3d> default_shape_vec() {
  std::vector<Vector3d> s = {
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(0, 1, 0),
    Vector3d(1, 1, 0),
    Vector3d(0, 0, 1),
    Vector3d(1, 0, 1),
    Vector3d(0, 1, 1),
    Vector3d(1, 1, 1)};
  return s;
}
}  // namespace

VolumetricActuatorSystem::VolumetricActuatorSystem()
  : VolumetricActuatorSystem(.008,            // m (kg)
                             default_shape_vec(), // 2x2x2 shape
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                             ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(const std::vector<Vector3d>& shape_arg)
  : VolumetricActuatorSystem(.008,            // m (kg)
                             shape_arg, // 2x2x2 shape
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                           ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(
  double m_arg, const std::vector<Vector3d>& shape_arg, double r_cl_arg, double r_op_arg)
  : m_(m_arg), shape_(shape_arg), r_cl_(r_cl_arg), r_op_(r_op_arg) {

  drake::log()->info(shape_.size());
}

multibody::ModelInstanceIndex VolumetricActuatorSystem::AddVASToPlant(multibody::MultibodyPlant<double>* plant) {

  DRAKE_THROW_UNLESS(plant != nullptr);

  // add model instance for VolumetricActuatorSystem multibody
  mii_ = plant->AddModelInstance("VAS");

  for (unsigned int i=0; i<shape_.size(); i++) {

    // name for the body
    const std::string div_name = "VA_" + std::to_string(i);

    // spatial inertia (later distribute mass, for now take 7/8)
    drake::multibody::SpatialInertia<double> M_Scm(
        m_*7/8,
        Vector3d::Zero(),
        drake::multibody::UnitInertia<double>::SolidSphere(r_cl_));

    // add body to plant
    const multibody::RigidBody<double>& center = plant->AddRigidBody(
      div_name, mii_, M_Scm);

    double r_center = r_cl_/ 5;

    // register visual, collision geometries
    plant->RegisterVisualGeometry(
        center, math::RigidTransformd(),
        geometry::Sphere(r_center),
        div_name + "_visual",
        Eigen::Vector4d(66./255, 197./255, 226./255, 1));

    plant->RegisterCollisionGeometry(
        center, math::RigidTransformd(),
        geometry::Sphere(r_center),
        div_name + "_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    for (unsigned int j=0; j<shape_.size(); j++) {
      if (Neighboring(i, j)) {
        const std::string con_name = std::to_string(i) + "to" + std::to_string(j);
        drake::log()->info("{} is neigboring {}", i, j);
        const multibody::RigidBody<double>& connecting_body = plant->AddRigidBody(con_name, mii_, M_Scm);
        plant->RegisterVisualGeometry(
            connecting_body, math::RigidTransformd(),
            geometry::Sphere(r_center/2),
            div_name + "_visual",
            Eigen::Vector4d(226./255, 197./255, 66./255, 1));
        plant->AddJoint<drake::multibody::PrismaticJoint>(
          con_name,
          /* Shoulder inboard frame Si IS the the world frame W. */
          plant->GetBodyByName("VA_" + std::to_string(i)), std::nullopt,
          /* Shoulder outboard frame So IS frame L1. */
          plant->GetBodyByName(con_name), std::nullopt,
          shape_[j]-shape_[i], r_cl_, r_op_); /* acrobot oscillates in the x-z plane. */
      }
    }
    // for (unsigned int j=0; j<shape_.size(); j++) {
    //   if (Neighboring(i, j)) {
    //     const std::string con_name = std::to_string(i) + "to" + std::to_string(j);
    //     plant->AddJoint<drake::multibody::WeldJoint>(
    //       "W"+std::to_string(i)+std::to_string(j),
    //       /* Shoulder inboard frame Si IS the the world frame W. */
    //       plant->GetBodyByName(std::to_string(i) + "to" + std::to_string(j)), std::nullopt,
    //       /* Shoulder outboard frame So IS frame L1. */
    //       plant->GetBodyByName(std::to_string(j) + "to" + std::to_string(i)), std::nullopt,
    //       math::RigidTransform<double>::Identity()); /* acrobot oscillates in the x-z plane. */
    //   }
    // }
  }

  return mii_;
}

void VolumetricActuatorSystem::InitPose(multibody::MultibodyPlant<double>* plant, systems::Context<double>* context) {
  for (unsigned int i=0; i<shape_.size(); i++) {
    math::RigidTransformd pose(2*r_cl_*shape_[i]);
    plant->SetFreeBodyPose(
        context, plant->GetBodyByName("VA_" + std::to_string(i)),
        pose);
  }
  for (auto joint_index : plant->GetJointIndices(mii_)) {
    std::string name = plant->get_joint(joint_index).name();
    const multibody::PrismaticJoint<double>& joint =
      plant->GetJointByName<multibody::PrismaticJoint>(name);
    joint.set_translation(context, r_cl_);
  }
}

bool VolumetricActuatorSystem::Neighboring(unsigned int i_1, unsigned int i_2) {
  return (shape_[i_1] - shape_[i_2]).norm() == 1;
}

}  // namespace examples
}  // namespace drake
