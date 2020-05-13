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
std::vector<Vector3d> unitVlist() {
  std::vector<Vector3d> s = {
    Vector3d::UnitX(),
    Vector3d::UnitY(),
    Vector3d::UnitZ(),
    -Vector3d::UnitX(),
    -Vector3d::UnitY(),
    -Vector3d::UnitZ()};
  return s;
}
}  // namespace

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant)
  : VolumetricActuatorSystem(plant, .008,            // m (kg)
                             default_shape_vec(), // 2x2x2 shape
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                             ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant, const std::vector<Vector3d>& shape_arg)
  : VolumetricActuatorSystem(plant, .008,            // m (kg)
                             shape_arg, // 2x2x2 shape
                             0.03485,         // closed diameter
                             0.04265          // open diameter
                           ) {}

VolumetricActuatorSystem::VolumetricActuatorSystem(multibody::MultibodyPlant<double>* plant,
  double m_arg, const std::vector<Vector3d>& shape_arg, double r_cl_arg, double r_op_arg)
  : plant_(plant), m_(m_arg), shape_(shape_arg), r_cl_(r_cl_arg), r_op_(r_op_arg) {

  drake::log()->info("{} actuators in system.", shape_.size());
  AddVASToPlant();
}

multibody::ModelInstanceIndex VolumetricActuatorSystem::AddVASToPlant() {

  DRAKE_THROW_UNLESS(plant_ != nullptr);

  // add model instance for VolumetricActuatorSystem multibody
  mii_ = plant_->AddModelInstance("VAS");

  // radius, spatial inertia for center bodies
  const double r_center = r_cl_/ 5;
  drake::multibody::SpatialInertia<double> M_Centercm(
    m_*7/8,
    Vector3d::Zero(),
    drake::multibody::UnitInertia<double>::SolidSphere(r_center));

  // radius, spatial inertia for connector bodies
  const double r_connect = r_cl_/ 10;
  drake::multibody::SpatialInertia<double> M_Connectcm(
    m_*1/8,
    Vector3d::Zero(),
    drake::multibody::UnitInertia<double>::SolidSphere(r_connect));

  // loop to add all actuator bodies
  for (unsigned int i=0; i<shape_.size(); i++) {

    // name for the body
    const std::string div_name = "VA_" + std::to_string(i);

    // add body to plant_
    const multibody::RigidBody<double>& center = plant_->AddRigidBody(
      div_name, mii_, M_Centercm);

    // register visual, collision geometries
    ShowAxes(div_name, r_cl_/2);

    plant_->RegisterVisualGeometry(
        center, math::RigidTransformd(),
        geometry::Sphere(r_center),
        div_name + "_visual",
        Eigen::Vector4d(0.3, 0.6, 1, 1));

    plant_->RegisterCollisionGeometry(
        center, math::RigidTransformd(),
        geometry::Sphere(r_center),
        div_name + "_collision",
        drake::multibody::CoulombFriction<double>(0.3, 0.3));

    // /* BUSH implementation
    // using LinearBushingRollPitchYaw to remove topological loops
    for (unsigned int j=0; j<6; j++) {
      const std::string connect_name = "VA_" + std::to_string(i) + "_" + std::to_string(j);

      // connect body initialization
      const multibody::RigidBody<double>& connect_body =
        plant_->AddRigidBody(connect_name, mii_, M_Connectcm);

      // connect body yellow sphere
      plant_->RegisterVisualGeometry(
        connect_body, math::RigidTransformd(),
        geometry::Sphere(r_connect),
        connect_name + "_visual",
        Eigen::Vector4d(1, 0.6, 1, 0.3));

      // joint from VA to connect body
      const multibody::Joint<double>& joint = plant_->AddJoint<drake::multibody::PrismaticJoint>(
        connect_name + "_joint",
        center, std::nullopt,
        connect_body, std::nullopt,
        unitVlist()[j], r_cl_, r_op_); // set direction, and joint limits
      // plant_->AddJointActuator(connect_name + "_ic", joint_ic);
      // drake::log()->info("Created Joint {}", joint.name());
      std::string n = joint.name();
    }


  }

  // end BUSH implementation*/

  /* SNAKE implementation
  // (continous center-connector-center, no duplicates)
  // loop to add joints with connector bodies between all actuators
  for (unsigned int i=0; i<shape_.size(); i++) {
    for (unsigned int j=i; j<shape_.size(); j++) {
      if (Neighboring(i, j)) {
        drake::log()->info("Connecting VA_{} to VA_{}...", i, j);

        // connector body initialization
        const std::string connect_name = "VA_" + std::to_string(i) + "-" + std::to_string(j);
        const multibody::RigidBody<double>& connect_body =
          plant_->AddRigidBody(connect_name, mii_, M_Connectcm);

        // connector body yellow sphere
        plant_->RegisterVisualGeometry(
          connect_body, math::RigidTransformd(),
          geometry::Sphere(r_connect),
          connect_name + "_visual",
          Eigen::Vector4d(226./255, 197./255, 66./255, 1));

        // joint from VA_i to connector body
        const multibody::Joint<double>& joint_ic = plant_->AddJoint<drake::multibody::PrismaticJoint>(
          connect_name + "_ic",
          plant_->GetBodyByName("VA_" + std::to_string(i)), std::nullopt,
          connect_body, std::nullopt,
          shape_[j]-shape_[i], r_cl_, r_op_); // set direction, and joint limits
        // plant_->AddJointActuator(connect_name + "_ic", joint_ic);

        const multibody::Joint<double>& joint_cj = plant_->AddJoint<drake::multibody::PrismaticJoint>(
        // joint from connector body to VA_j
          connect_name + "_cj",
          connect_body, std::nullopt,
          plant_->GetBodyByName("VA_" + std::to_string(j)), std::nullopt,
          shape_[j]-shape_[i], r_cl_, r_op_); // set direction, and joint limits
        // plant_->AddJointActuator(connect_name + "_cj", joint_cj);

        drake::log()->info("Created Joints {} and {}", joint_ic.name(), joint_cj.name());
      }
    }
  }
  end SNAKE implementation */

  return mii_;
}

void VolumetricActuatorSystem::AddBushing(const std::string body0_name, const std::string body1_name, const double k_z, const double d_z) {

  // add visuals to the two bodies to weld together
  plant_->RegisterVisualGeometry(
    plant_->GetBodyByName(body0_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/9),
    body0_name + "visualbush",
    Eigen::Vector4d(1, 0.5, 0.1, 1));
  plant_->RegisterVisualGeometry(
    plant_->GetBodyByName(body1_name), math::RigidTransformd(),
    geometry::Sphere(r_cl_/9),
    body1_name + "visualbush",
    Eigen::Vector4d(1, 0.5, 0.1, 1));

  // get body frames
  const multibody::Frame<double>& frame0 = plant_->GetBodyByName(body0_name).body_frame();
  const multibody::Frame<double>& frame1 = plant_->GetBodyByName(body1_name).body_frame();

  // create constants matrix to pass to bushing
  const Vector3d force_stiffness_constants{30000, 30000, k_z};  // N/m
  const Vector3d force_damping_constants{1500, 1500, d_z};    // N·s/m
  const Vector3d torque_stiffness_constants{30000, 30000, 30000};     // N·m/rad
  const Vector3d torque_damping_constants{1500, 1500, 1500};       // N·m·s/rad

  // Add a bushing force element where the joint between body0 and body1
  plant_->AddForceElement<multibody::LinearBushingRollPitchYaw>(
      frame0, frame1, torque_stiffness_constants,
      torque_damping_constants, force_stiffness_constants,
      force_damping_constants);
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
    joint.set_translation(context, r_cl_);
  }
}

bool VolumetricActuatorSystem::Neighboring(unsigned int i_1, unsigned int i_2) {
  // return true if the distance between the bodies is 1 (neighbors)
  return (shape_[i_1] - shape_[i_2]).norm() == 1;
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

}  // namespace examples
}  // namespace drake
