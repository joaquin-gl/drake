#include "drake/project/vas_plant.h"

namespace drake {
namespace project {

namespace {
std::vector<Vector3d> box_shape() {
  return std::vector<Vector3d>{
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(0, 1, 0),
    Vector3d(1, 1, 0),
    Vector3d(0, 0, 1),
    Vector3d(1, 0, 1),
    Vector3d(0, 1, 1),
    Vector3d(1, 1, 1)};
}
std::vector<Vector3d> direction_vectors() {
  return std::vector<Vector3d>{
    Vector3d::UnitX(),
    Vector3d::UnitY(),
    Vector3d::UnitZ(),
    -Vector3d::UnitX(),
    -Vector3d::UnitY(),
    -Vector3d::UnitZ()};
}
std::vector<std::string> direction_names() {
  return std::vector<std::string>{
    "px",
    "py",
    "pz",
    "nx",
    "ny",
    "nz"};
}
}  // namespace

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant)
  : VolumetricActuatorSystem(plant,
                             box_shape(),     // 2x2x2 shape
                             .008,            // m (kg)
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                             ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant, const std::vector<Vector3d>& shape_arg)
  : VolumetricActuatorSystem(plant,
                             shape_arg,       // given shape
                             .008,            // m (kg)
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                             ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant,
  const std::vector<Vector3d>& shape_arg, double m_arg, double r_cl_arg, double r_op_arg)
  : plant_(plant), shape_(shape_arg), m_(m_arg), r_cl_(r_cl_arg), r_op_(r_op_arg) {

  // log()->info("{} actuators in system.", shape_.size());
}

void VolumetricActuatorSystem::AddVASToPlant() {

  DRAKE_THROW_UNLESS(plant_ != nullptr);

  // add model instance for VolumetricActuatorSystem multibody
  mii_ = plant_->AddModelInstance("VAS");

  // radius, spatial inertia for center bodies
  const double center_radius = r_cl_/ 5;
  drake::multibody::SpatialInertia<double> center_inertia(
    m_*4/5,
    Vector3d::Zero(),
    drake::multibody::UnitInertia<double>::SolidSphere(center_radius));

  // radius, spatial inertia for spoke bodies
  const double spoke_radius = r_cl_/ 10;
  drake::multibody::SpatialInertia<double> spoke_inertia(
    m_*1/(5*6),
    Vector3d::Zero(),
    drake::multibody::UnitInertia<double>::SolidSphere(spoke_radius));

  // loop to add all actuator center bodies
  for (unsigned int i=0; i<shape_.size(); i++) {

    // name for the body
    std::string center_name = "VA_" + std::to_string(i);

    // add body to plant_
    const multibody::RigidBody<double>& center_body = plant_->AddRigidBody(
      center_name, mii_, center_inertia);

    // show axes cause why not
    ShowAxes(center_name, r_cl_/2);

    // register visual, collision geometries
    plant_->RegisterVisualGeometry(
        center_body, math::RigidTransformd(),
        geometry::Sphere(center_radius),
        center_name + "_visual",
        Eigen::Vector4d(0.3, 0.6, 1, 1));

    plant_->RegisterVisualGeometry(
        center_body, math::RigidTransformd(),
        geometry::Sphere(r_cl_),
        center_name + "_visual_closed",
        Eigen::Vector4d(1, 1, 1, .1));
    plant_->RegisterVisualGeometry(
        center_body, math::RigidTransformd(),
        geometry::Sphere(r_op_),
        center_name + "_visual_open",
        Eigen::Vector4d(1, 1, 1, .1));

    plant_->RegisterCollisionGeometry(
        center_body, math::RigidTransformd(),
        geometry::Sphere(center_radius),
        center_name + "_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    // loop to add all actuator spoke bodies
    for (unsigned int j=0; j<direction_vectors().size(); j++) {
      const std::string spoke_name = center_name + "_" + direction_names()[j];
      const Vector3d spoke_direction = direction_vectors()[j];

      // actuator body initialization
      const multibody::RigidBody<double>& spoke_body =
        plant_->AddRigidBody(spoke_name, mii_, spoke_inertia);

      // joint from center body to spoke body
      const multibody::Joint<double>& joint = plant_->AddJoint<drake::multibody::PrismaticJoint>(
        spoke_name,
        center_body, std::nullopt,
        spoke_body, std::nullopt,
        spoke_direction, r_cl_, r_op_); // set direction, and joint limits

      // log()->info("Created Joint {}", joint.name());
      std::string n = joint.name();

      // add joint actuator (5N as force limit)
      plant_->AddJointActuator(spoke_name, joint, 5);

      // get neighbor direction to use for neighbor name
      std::string neighbor_direction;
      if (j+3 < direction_names().size()) neighbor_direction = direction_names()[j+3];
      else neighbor_direction = direction_names()[j+3-direction_names().size()];
      // log()->info("{} to {}", direction_names()[j], neighbor_direction);

      std::string neighbor_name = "VA_" + std::to_string(HasNeighbor(i, spoke_direction)) + "_" + neighbor_direction;
      if (plant_->HasBodyNamed(neighbor_name)) {
        // log()->info("Weld {} to {}", spoke_name, neighbor_name);
        AddConnect(spoke_name, neighbor_name, spoke_params_[0], spoke_params_[1], spoke_params_[2], spoke_params_[3]);

      } else if (HasNeighbor(i, spoke_direction) < 0) {
        // log()->info("Collide {}", spoke_name);
        AddCollide(spoke_name, spoke_params_[4], spoke_params_[5]);

      }
    }
  }
}

void VolumetricActuatorSystem::SetParams(
  const double k_xyz, const double d_xyz, const double k_012, const double d_012,
  const double static_friction, const double dynamic_friction) {

  spoke_params_ = std::vector<double>{k_xyz, d_xyz, k_012, d_012, static_friction, dynamic_friction};
  }

void VolumetricActuatorSystem::AddConnect(
  const std::string body0_name, const std::string body1_name,
  const double k_xyz, const double d_xyz, const double k_012, const double d_012) {

  // visual
  plant_->RegisterVisualGeometry(
    plant_->GetBodyByName(body0_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/9),
    body0_name + "visual",
    Eigen::Vector4d(1, 0.5, 0.1, 1));

  plant_->RegisterVisualGeometry(
    plant_->GetBodyByName(body1_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/9),
    body1_name + "visual",
    Eigen::Vector4d(1, 0.5, 0.1, 1));

  // get body frames
  const multibody::Frame<double>& frame0 = plant_->GetBodyByName(body0_name).body_frame();
  const multibody::Frame<double>& frame1 = plant_->GetBodyByName(body1_name).body_frame();

  // create constants matrix to pass to bushing
  const Vector3d force_stiffness_constants{k_xyz, k_xyz, k_xyz};  // N/m
  const Vector3d force_damping_constants{d_xyz, d_xyz, d_xyz};    // N·s/m
  const Vector3d torque_stiffness_constants{k_012, k_012, k_012};     // N·m/rad
  const Vector3d torque_damping_constants{d_012, d_012, d_012};       // N·m·s/rad

  // Add a bushing force element where the joint between body0 and body1
  plant_->AddForceElement<multibody::LinearBushingRollPitchYaw>(
      frame0, frame1, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);
}

void VolumetricActuatorSystem::AddCollide(
  const std::string body_name, const double static_friction, const double dynamic_friction) {

  // visual
  plant_->RegisterVisualGeometry(
    plant_->GetBodyByName(body_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/10),
    body_name + "visual",
    Eigen::Vector4d(0.5, 1, 0.1, 1));

  // collision
  plant_->RegisterCollisionGeometry(
    plant_->GetBodyByName(body_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/10),
    body_name + "_collision",
    multibody::CoulombFriction<double>(static_friction, dynamic_friction));
}

void VolumetricActuatorSystem::InitPose(systems::Context<double>* context) {
  // set actuator pose if floating (deprecated)
  for (unsigned int i=0; i<shape_.size(); i++) {
    math::RigidTransformd pose(2*r_cl_*shape_[i]);
    if (plant_->GetBodyByName("VA_" + std::to_string(i)).is_floating()) {
      plant_->SetFreeBodyPose(
        context, plant_->GetBodyByName("VA_" + std::to_string(i)),
        pose);
    }
  }

  // set joint state to closed position to start
  for (auto joint_index : plant_->GetJointIndices(mii_)) {
    std::string name = plant_->get_joint(joint_index).name();
    const multibody::PrismaticJoint<double>& joint =
      plant_->GetJointByName<multibody::PrismaticJoint>(name);
    // joint.set_translation(context, (r_cl_+r_op_)/2);
    joint.set_translation(context, r_cl_);
  }
}

std::vector<math::RigidTransformd> VolumetricActuatorSystem::GetCenterPoses(systems::Context<double>* context) {
  std::vector<math::RigidTransformd> poses;
  for (unsigned int i=0; i<shape_.size(); i++) {
    // math::RigidTransformd pose = plant_->GetFreeBodyPose(*context, plant_->GetBodyByName("VA_" + std::to_string(i)));
    math::RigidTransformd pose = plant_->EvalBodyPoseInWorld(*context, plant_->GetBodyByName("VA_" + std::to_string(i)));
    poses.push_back(pose);
  }
  return poses;
}

std::vector<double> VolumetricActuatorSystem::GetJointTranslations(systems::Context<double>* context) {
  std::vector<double> translations;
  for (auto joint_index : plant_->GetJointIndices(mii_)) {
    std::string name = plant_->get_joint(joint_index).name();
    const multibody::PrismaticJoint<double>& joint =
      plant_->GetJointByName<multibody::PrismaticJoint>(name);
    translations.push_back(joint.get_translation(*context));
  }
  return translations;
}

// std::vector<double> VolumetricActuatorSystem::GetCenterStates

int VolumetricActuatorSystem::HasNeighbor(unsigned int i, Vector3d direction) {
  // returns index of neighbor body if it finds one
  Vector3d neighbor_pos = shape_[i] + direction;
  for (unsigned int j=0; j<shape_.size(); j++) {
    if (neighbor_pos == shape_[j]) return j;
  }
  return -1;
}

void VolumetricActuatorSystem::ShowAxes(std::string bodyname, double scale) {
  // show x-axis
  plant_->RegisterVisualGeometry(
      plant_->GetBodyByName(bodyname),
      math::RigidTransformd(math::RollPitchYawd(0, M_PI/2, 0), Vector3d(scale/2, 0, 0)),
      geometry::Cylinder(0.001, scale),
      bodyname + "_ax_x",
      Eigen::Vector4d(1, 0, 0, 0.2));

  // show y-axis
  plant_->RegisterVisualGeometry(
      plant_->GetBodyByName(bodyname),
      math::RigidTransformd(math::RollPitchYawd(M_PI/2, 0, 0), Vector3d(0, scale/2, 0)),
      geometry::Cylinder(0.001, scale),
      bodyname + "_ax_y",
      Eigen::Vector4d(0, 1, 0, 0.2));

  // show z-axis
  plant_->RegisterVisualGeometry(
      plant_->GetBodyByName(bodyname),
      math::RigidTransformd(math::RollPitchYawd(0, 0, 0), Vector3d(0, 0, scale/2)),
      geometry::Cylinder(0.001, scale),
      bodyname + "_ax_z",
      Eigen::Vector4d(0, 0, 1, 0.2));
}

}  // namespace project
}  // namespace drake
