#include <memory>

#include <gflags/gflags.h>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/uniform_gravity_field_element.h"
#include "drake/multibody/tree/prismatic_joint.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/examples/springboard/easy_force.h"
#include "drake/examples/springboard/easy_shape.h"
#include "drake/project/vol_actuator.h"
#include "drake/examples/springboard/convenience.h"

namespace drake {
namespace project {
namespace {

using geometry::SceneGraph;
using lcm::DrakeLcm;
using Eigen::Vector3d;

// "multibody" namespace is ambiguous here without "drake::".
using drake::multibody::MultibodyPlant;

using drake::multibody::Body;
using drake::multibody::SpatialForce;

using drake::examples::AddTableToPlant;
using drake::examples::PrintMBPStats;
using drake::examples::EasyForce;
using drake::examples::PrintBodyStates;

DEFINE_double(target_realtime_rate, 1.0,
            "Desired rate relative to real time.  See documentation for "
            "Simulator::set_target_realtime_rate() for details.");

DEFINE_double(duration, 5.0,
            "Desired duration of the simulation in seconds.");

DEFINE_double(time_step, 3e-3,
            "If greater than zero, the plant is modeled as a system with "
            "discrete updates and period equal to this time_step. "
            "If 0, the plant is modeled as a continuous system.");

DEFINE_double(t0, 0, "Trigger time 0");
DEFINE_double(t1, 1, "Trigger time 1");
DEFINE_double(t2, 2, "Trigger time 2");

DEFINE_double(s, 0.2, "object scale");
DEFINE_double(m, 0.1, "object mass");

DEFINE_double(k, 3, "spring stiffness");
DEFINE_double(b, 0.3, "damping ratio");

DEFINE_double(h, 5, "height of drop");

DEFINE_bool(rand_rot, true, "randomize rotation of object?");

int do_main() {
    systems::DiagramBuilder<double> builder;

    auto pair = AddMultibodyPlantSceneGraph(&builder, std::make_unique<MultibodyPlant<double>>(FLAGS_time_step));
    MultibodyPlant<double>& plant = pair.plant;

    SceneGraph<double>& scene_graph = pair.scene_graph;
    scene_graph.set_name("scene_graph");

    // Add gravity to the model.
    // plant.AddForceElement<UniformGravityFieldElement>();
    const Vector3<double> gravity_vector_W(0, 0, -9.81);
    plant.mutable_gravity_field().set_gravity_vector(gravity_vector_W);

    srand(time(NULL));

    // indices for object to set initial transform
    multibody::ModelInstanceIndex model_instance_index(0);
    multibody::BodyIndex object_body_index(1);

    // add desired object using:
    //     object scale parameter s
    //     object mass parameter m

    model_instance_index = AddVolActuatorToPlant(
        "lsdbox", FLAGS_s/10,
        FLAGS_s, FLAGS_s, FLAGS_s,
        FLAGS_m, FLAGS_k, FLAGS_b, &plant);
    // object_body_index(1);

    // Add table to model (also scaled)
    const math::RigidTransformd X_WT; // Identity transform
    AddTableToPlant(
        FLAGS_s*4, FLAGS_s*4, FLAGS_s/2,
        X_WT, &plant);

    // Now the model is complete.
    plant.Finalize();

    // Print plant stats
    PrintMBPStats(&plant);
    // PrintBodyIndices(&plant);

    // Set tolerances
    plant.set_penetration_allowance(1.0E-5);
    plant.set_stiction_tolerance(1.0E-5);

    // Sanity check on the availability of the optional source id before using it.
    DRAKE_DEMAND(plant.geometry_source_is_registered());

    // add EasyForce
    auto easy_force = builder.AddSystem(std::make_unique<EasyForce>());

    // // connect EasyForce output port to plant input port
    builder.Connect(
        easy_force->get_externally_applied_spatial_force_output_port(),
        plant.get_applied_spatial_force_input_port());

    // CALL ALL CONNECTIONS BEFORE THESE TWO LINES:
    geometry::ConnectDrakeVisualizer(&builder, scene_graph);
    auto diagram = builder.Build();

    // stuff I don't really understand too well:
    std::unique_ptr<systems::Context<double>> diagram_context =
        diagram->CreateDefaultContext();
    diagram->SetDefaultContext(diagram_context.get());
    systems::Context<double>& plant_context =
        diagram->GetMutableSubsystemContext(plant, diagram_context.get());
    plant.SetDefaultContext(&plant_context);

    systems::Simulator<double> simulator(*diagram, std::move(diagram_context));
    auto& simulator_context = simulator.get_mutable_context();

    // initial vector for dropping objects
    math::RigidTransformd X_WO(Vector3d(0, 0, FLAGS_s*FLAGS_h));
    if (FLAGS_rand_rot){
        X_WO.set_rotation(math::RotationMatrixd(math::RollPitchYawd(
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX),
            2.0*M_PI*double(rand()) / double(RAND_MAX))));
        }

    // vector of positions of vertices of lsdbox in lsdbox frame
    std::vector<Vector3d> div_p_vector = {
        Vector3d( 0.5*FLAGS_s, -0.5*FLAGS_s, -0.5*FLAGS_s),
        Vector3d(-0.5*FLAGS_s, -0.5*FLAGS_s, -0.5*FLAGS_s),
        Vector3d(-0.5*FLAGS_s,  0.5*FLAGS_s, -0.5*FLAGS_s),
        Vector3d( 0.5*FLAGS_s,  0.5*FLAGS_s, -0.5*FLAGS_s),
        Vector3d(-0.5*FLAGS_s, -0.5*FLAGS_s,  0.5*FLAGS_s),
        Vector3d( 0.5*FLAGS_s, -0.5*FLAGS_s,  0.5*FLAGS_s),
        Vector3d(-0.5*FLAGS_s,  0.5*FLAGS_s,  0.5*FLAGS_s),
        Vector3d( 0.5*FLAGS_s,  0.5*FLAGS_s,  0.5*FLAGS_s),
    };

    // vector of lsdbox vertices in world frame
    std::vector<math::RigidTransformd> X_WD_vector(div_p_vector.size());
    for (unsigned int i=0; i<div_p_vector.size(); i++) {
        X_WD_vector[i] = X_WO*math::RigidTransformd(div_p_vector[i]);
    }

    // Show that object initial poses are unset and must be set
    std::cout << "UNSET POSES:\n";
    PrintBodyStates(&plant, &plant_context);
    SetVolActuatorPose(&plant_context, model_instance_index, X_WD_vector, &plant);
    std::cout << "SET POSES:\n";
    PrintBodyStates(&plant, &plant_context);

    simulator.set_publish_every_time_step(false);
    simulator.set_target_realtime_rate(FLAGS_target_realtime_rate);
    simulator.Initialize();

    // add actuation force
    int easyforce_index = easy_force->AddForce(object_body_index);

    // Using a while loop to step the sim as opposed to `simulator.StepTo(FLAGS_duration)` so that we
    // can periodically print the sim's context to command line for debugging.
    double current_time = 0.0;
    const double time_delta = 0.008;
    while(current_time < FLAGS_duration) {

        // Suspend object from gravity
        if (current_time >= FLAGS_t0 && current_time < FLAGS_t0 + time_delta) {
            std::cout << "\nFirst trigger\n";
            easy_force->SetForce(easyforce_index,
                SpatialForce<double>(
                    Vector3d::Zero(),
                    Vector3d(0,0,FLAGS_m*9.81)));
        }

        // Stop suspension
        if (current_time >= FLAGS_t1 && current_time < FLAGS_t1 + time_delta) {
            std::cout << "\nSecond trigger\n";
            easy_force->SetForce(easyforce_index,
                SpatialForce<double>(
                    Vector3d::Zero(),
                    Vector3d::Zero()));
        }

        // Push object into table
        if (current_time >= FLAGS_t2 && current_time < FLAGS_t2 + time_delta) {
            std::cout << "\nThird trigger\n";
            easy_force->SetForce(easyforce_index,
                SpatialForce<double>(
                    Vector3d::Zero(),
                    Vector3d(0,0,FLAGS_m*-9.81)));
        }

        simulator.AdvanceTo(current_time + time_delta);
        current_time = simulator_context.get_time();
    }

    return 0;
}

}  // namespace
}  // namespace project
}  // namespace drake

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage(
      "A shape to table collision example"
      "to demonstrate EasyForce and making an LSD shape."
      "Launch drake-visualizer before running this example.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return drake::project::do_main();
}
