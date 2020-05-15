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

DEFINE_double(duration, 5.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 3e-4,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_string(sys, "box",
            "System formation of the volumetric actuators. Can be:"
            "box, worm, or pants for now.");

DEFINE_double(force, .001,
            "Force of actuator");

int do_main() {
  systems::DiagramBuilder<double> builder;

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
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(2, 0, 0),
    Vector3d(3, 0, 0)};
  std::vector<Vector3d> pants = {
    Vector3d(0, 0, 0),
    Vector3d(1, 0, 0),
    Vector3d(2, 0, 0),
    Vector3d(0, 0, 1),
    Vector3d(1, 0, 1),
    Vector3d(2, 0, 1),
    Vector3d(0, 1, 1),
    Vector3d(1, 1, 1),
    Vector3d(2, 1, 1),
    Vector3d(0, 2, 1),
    Vector3d(1, 2, 1),
    Vector3d(2, 2, 1),
    Vector3d(0, 2, 0),
    Vector3d(1, 2, 0),
    Vector3d(2, 2, 0),
    };

  std::vector<Vector3d> sys;
  if (FLAGS_sys == "box") {
    sys = box;
  } else if (FLAGS_sys == "worm") {
    sys = worm;
  } else if (FLAGS_sys == "pants") {
    sys = pants;
  }

  auto [plant, scene_graph] =
      multibody::AddMultibodyPlantSceneGraph(&builder, FLAGS_time_step);
  geometry::ConnectDrakeVisualizer(&builder, scene_graph);

  auto vas = builder.AddSystem(std::make_unique<VolumetricActuatorSystem>(&plant, sys));
  vas->set_name("vas");

  auto controller = builder.AddSystem(std::make_unique<SequenceController>(sys.size(), 6));
  controller->set_name("controller");

  examples::AddTableToPlant(
      0.5, 0.5, 0.1,
      math::RigidTransformd(math::RollPitchYawd(0.05, 0.05, 0), -0.3*Vector3d::UnitZ()), &plant);

  plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Vector3d::UnitZ());

  //add bushing
  // vas->AddConnect("VA_0_pz", "VA_4_nz", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_0_px", "VA_1_nx", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_0_py", "VA_2_ny", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_1_py", "VA_3_ny", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_2_px", "VA_3_nx", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_2_pz", "VA_6_nz", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_1_pz", "VA_5_nz", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_3_pz", "VA_7_nz", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_4_px", "VA_5_nx", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_4_py", "VA_6_ny", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_6_px", "VA_7_nx", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);
  // vas->AddConnect("VA_7_ny", "VA_5_py", FLAGS_k_xyz, FLAGS_d_xyz, FLAGS_k_012, FLAGS_d_012);

  plant.Finalize();
  examples::PrintMBPStats(&plant);
  // examples::PrintBodyIndices(&plant);

  // controller->PrintState();
  Eigen::VectorXd state_send(8);
  state_send << FLAGS_force, FLAGS_force, FLAGS_force, FLAGS_force, FLAGS_force, FLAGS_force, FLAGS_force, FLAGS_force;
  controller->SetState(state_send);
  // controller->PrintState();

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

  const std::string joint_name = "VA_0_pz";
  // examples::CSVLogger logger("drake/project/output.csv",std::vector<std::string>{
  //   "time", joint_name, joint_name + " low", joint_name + " upp"});
  examples::CSVLogger logger("drake/project/output.csv",std::vector<std::string>{});
  double current_time = 0.0;
  const double time_step = 0.005;

  while (current_time < FLAGS_duration) {

    const multibody::PrismaticJoint<double>& joint =
      plant.GetJointByName<multibody::PrismaticJoint>(joint_name);
    // logger.Log(std::vector<double>{
    //   current_time, joint.get_translation(plant_context), joint.position_lower_limit(), joint.position_upper_limit()});

    std::vector<double> output = vas->GetJointTranslations(&plant_context);
    output.push_back(joint.position_lower_limit());
    output.push_back(joint.position_upper_limit());
    output.push_back(current_time);
    logger.Log(output);

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
