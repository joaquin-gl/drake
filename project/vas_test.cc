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
#include "drake/examples/springboard/convenience.h"

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

DEFINE_double(time_step, 3e-3,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_string(sys, "box",
            "hi");

DEFINE_double(k_z, 0,
            "spring stiffness in joint");

DEFINE_double(d_z, 0,
            "spring damping in joint");

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

  examples::AddTableToPlant(
      0.5, 0.5, 0.01,
      math::RigidTransformd(-0.1*Vector3d::UnitZ()), &plant);

  plant.mutable_gravity_field().set_gravity_vector(
      -9.81 * Vector3d::UnitZ());

  //add bushing
  vas->AddBushing("VA_0_2", "VA_4_5", FLAGS_k_z, FLAGS_d_z);
  plant.Finalize();
  // examples::PrintMBPStats(&plant);
  // examples::PrintBodyIndices(&plant);

  // // connect controller and plant
  // auto controller = builder.AddSystem(
  //     SequenceController("vas"));
  // controller->set_name("controller");
  // builder.Connect(plant.get_state_output_port(),
  //                 controller->get_input_port());
  // builder.Connect(controller->get_output_port(),
  //                 plant.get_actuation_input_port());

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
  // auto& simulator_context = simulator.get_mutable_context();

  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
  simulator.Initialize();

  simulator.AdvanceTo(FLAGS_duration);
  // double current_time = simulator_context.get_time();

  return 0;
}

}  // namespace
}  // namespace project
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::project::do_main();
}
