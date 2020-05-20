#include <memory>

#include <gflags/gflags.h>

#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

#include "drake/project/vas_plant.h"
#include "drake/project/sequence_controller.h"
#include "drake/examples/springboard/convenience.h"
#include "drake/examples/springboard/csv_logger.h"
#include "drake/examples/springboard/easy_shape.h"


namespace drake {

using systems::DiagramBuilder;
using systems::Simulator;
using systems::Context;
using systems::ContinuousState;
using systems::VectorBase;

using multibody::MultibodyPlant;

namespace project {
namespace {

DEFINE_double(target_realtime_rate, 1.0,
            "Desired rate relative to real time.  See documentation for "
            "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(duration, 10.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 3e-4,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_string(sys, "box",
            "System formation of the volumetric actuators. Can be:"
            "box, worm, or pants for now.");

DEFINE_double(k_xyz, 10000, "hi");
DEFINE_double(d_xyz, 10, "hi");
DEFINE_double(k_012, 100, "hi");
DEFINE_double(d_012, .001, "hi");
DEFINE_double(sf, .3, "hi");
DEFINE_double(df, .3, "hi");

DEFINE_double(force, 5,
            "Force of actuator");

DEFINE_double(vel, 5,
            "velocity to log");

DEFINE_string(demo, "backforth", "hi");
DEFINE_string(log, "com", "hi");

int do_main() {
  systems::DiagramBuilder<double> builder;

  std::vector<Vector3d> single = {
    Vector3d(0, 0, 0)};
  std::vector<Vector3d> box = {
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(0, 1, 0),
    Vector3d(1, 1, 0),
    Vector3d(0, 0, 1),
    Vector3d(1, 0, 1),
    Vector3d(0, 1, 1),
    Vector3d(1, 1, 1)};
  std::vector<Vector3d> worm = {
    Vector3d(-1, 0, 0),
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0)};
  std::vector<Vector3d> pants = {
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(0, 0, 1),
    Vector3d(1, 0, 1),
    Vector3d(0, 1, 1),
    Vector3d(1, 1, 1),
    Vector3d(0, 2, 1),
    Vector3d(1, 2, 1),
    Vector3d(0, 2, 0),
    Vector3d(1, 2, 0),
    // Vector3d(2, 0, 0),
    // Vector3d(2, 0, 1),
    // Vector3d(2, 1, 1),
    // Vector3d(2, 2, 1),
    // Vector3d(2, 2, 0),
    };

  std::vector<Vector3d> sys;
  if (FLAGS_sys == "box") {
    sys = box;
  } else if (FLAGS_sys == "worm") {
    sys = worm;
  } else if (FLAGS_sys == "single") {
    sys = single;
  } else if (FLAGS_sys == "pants") {
    sys = pants;
  }

  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto vas = builder.AddSystem(std::make_unique<VolumetricActuatorSystem>(
    &plant, sys, 0.407747197, 0.0381, 0.06985));
  vas->set_name("vas");

  auto controller = builder.AddSystem(std::make_unique<SequenceController>(sys.size(), 6));
  controller->set_name("controller");

  // examples::AddTableToPlant(
  //     4, 4, 0.1,
  //     math::RigidTransformd(math::RollPitchYawd(0.05, 0.05, 0), -0.3*Vector3d::UnitZ()), &plant);
  examples::AddTableToPlant(
      10, 10, 0.1,
      math::RigidTransformd(-0.1*Vector3d::UnitZ()), &plant);

  plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Vector3d::UnitZ());

  vas->SetParams(
    FLAGS_k_xyz,
    FLAGS_d_xyz,
    FLAGS_k_012,
    FLAGS_d_012,
    FLAGS_sf,
    FLAGS_df);
  vas->AddVASToPlant();

  plant.Finalize();
  examples::PrintMBPStats(&plant);
  // examples::PrintBodyIndices(&plant);

  // connect controller and plant
  builder.Connect(controller->get_output_port(), plant.get_actuation_input_port());

  // CALL ALL CONNECTIONS BEFORE THESE TWO LINES:
  auto diagram = builder.Build();

  // stuff I don't really understand too well:
  std::unique_ptr<systems::Context<double>> diagram_context =
      diagram->CreateDefaultContext();
  diagram->SetDefaultContext(diagram_context.get());
  systems::Context<double>& plant_context =
      diagram->GetMutableSubsystemContext(plant, diagram_context.get());
  plant.SetDefaultContext(&plant_context);

  vas->InitPose(&plant_context);

  systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
  auto& simulator_context = simulator.get_mutable_context();

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  // simulator.AdvanceTo(FLAGS_duration);

  examples::CSVLogger logger("drake/project/output.csv",std::vector<std::string>{});
  double current_time = 0.0;
  const double time_step = 0.005;

  while (current_time < FLAGS_duration) {


    // box back & forth
    if (FLAGS_demo == "backforth") {
      if (current_time >= 1 && current_time < 1 + time_step) {
        log()->info("back");
        Eigen::VectorXd state(8);
        state << FLAGS_force, FLAGS_force,
                -FLAGS_force, -FLAGS_force,
                -FLAGS_force, -FLAGS_force,
                -FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 4 && current_time < 4 + time_step) {
        log()->info("and forth");
        Eigen::VectorXd state(8);
        state << -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
    }

    // box full cycle
    if (FLAGS_demo == "fullbox") {
      if (current_time >= 1 && current_time < 1 + time_step) {
        Eigen::VectorXd state(8);
        state << FLAGS_force, FLAGS_force,
                -FLAGS_force, -FLAGS_force,
                -FLAGS_force, -FLAGS_force,
                -FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 4 && current_time < 4 + time_step) {
        Eigen::VectorXd state(8);
        state << -FLAGS_force, -FLAGS_force,
                 FLAGS_force, FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 7 && current_time < 7 + time_step) {
        Eigen::VectorXd state(8);
        state << -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 10 && current_time < 10 + time_step) {
        Eigen::VectorXd state(8);
        state << -FLAGS_force, -FLAGS_force,
                 -FLAGS_force, -FLAGS_force,
                 FLAGS_force, FLAGS_force,
                 -FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
    }

    // worm locomotion
    else if (FLAGS_demo == "worm") {
      if (current_time >= 0 && current_time < 0 + time_step) {
        Eigen::VectorXd state(3);
        state << FLAGS_force, FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 0.5 && current_time < 0.5 + time_step) {
        Eigen::VectorXd state(3);
        state << -FLAGS_force, FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 1 && current_time < 1 + time_step) {
        Eigen::VectorXd state(3);
        state << -FLAGS_force, -FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 1.5 && current_time < 1.5 + time_step) {
        Eigen::VectorXd state(3);
        state << FLAGS_force, -FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 2 && current_time < 2 + time_step) {
        Eigen::VectorXd state(3);
        state << FLAGS_force, -FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 2.5 && current_time < 2.5 + time_step) {
        Eigen::VectorXd state(3);
        state << FLAGS_force, FLAGS_force, -FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
      if (current_time >= 3 && current_time < 3 + time_step) {
        Eigen::VectorXd state(3);
        state << FLAGS_force, FLAGS_force, FLAGS_force;
        controller->SetState(state);
        // controller->PrintState();
      }
    }

    // outputs
    std::vector<double> output;
    output.push_back(current_time);

    // joint translations output
    if (FLAGS_log == "joints"){
      const multibody::PrismaticJoint<double>& joint =
        plant.GetJointByName<multibody::PrismaticJoint>("VA_0_pz");
      std::vector<double> joint_translations = vas->GetJointTranslations(&plant_context);
      for (auto joint_translation : joint_translations) {
        output.push_back(joint_translation);
      }
      output.push_back(joint.position_lower_limit());
      output.push_back(joint.position_upper_limit());

    }

    // body pitch output
    if (FLAGS_log == "poses"){
      std::vector<math::RigidTransformd> poses = vas->GetCenterPoses(&plant_context);
      for (auto pose : poses) {
        math::RollPitchYawd rpy(pose.rotation());
        Vector3d xyz(pose.translation());
        // log()->info(rpy.pitch_angle());
        // output.push_back(rpy.pitch_angle());
        output.push_back(xyz(0));
      }
    }

    // single pose output
    if (FLAGS_log == "pose"){
      std::vector<math::RigidTransformd> poses = vas->GetCenterPoses(&plant_context);
      math::RigidTransformd pose = poses[1];
      math::RollPitchYawd rpy(pose.rotation());
      Vector3d xyz(pose.translation());
      // log()->info(rpy.pitch_angle());
      // output.push_back(rpy.pitch_angle());
      output.push_back(xyz(0));
    }

    // com output;
    if (FLAGS_log == "com"){
      Vector3d com = vas->GetCoMPosition(&plant_context);
      output.push_back(com(0));
      output.push_back(com(1));
      output.push_back(com(2));
    }

    // spatial vel output
    if (FLAGS_log == "vel"){
    std::vector<multibody::SpatialVelocity<double>> spatial_velocities = vas->GetSpatialVelocities(&plant_context);
      for (auto spatial_velocity: spatial_velocities) {
        output.push_back(spatial_velocity[FLAGS_vel]);
      }
    }

    // spatial vel & position ouptut
    if (FLAGS_log == "velpos"){
      std::vector<multibody::SpatialVelocity<double>> spatial_velocities = vas->GetSpatialVelocities(&plant_context);
      for (auto spatial_velocity: spatial_velocities) {
        output.push_back(spatial_velocity[0]);
      }
      std::vector<math::RigidTransformd> poses = vas->GetCenterPoses(&plant_context);
      for (auto pose : poses) {
        math::RollPitchYawd rpy(pose.rotation());
        Vector3d xyz(pose.translation());
        output.push_back(rpy.roll_angle());
      }
    }


    if (current_time>time_step) logger.Log(output);

    simulator.AdvanceTo(current_time + time_step);
    current_time = simulator_context.get_time();
  }

  return 0;
}

}  // namespace
}  // namespace project
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::project::do_main();
}
