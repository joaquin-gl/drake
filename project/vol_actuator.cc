#include "drake/project/vol_actuator.h"

namespace drake {
namespace project {

multibody::ModelInstanceIndex AddVolActuatorToPlant(
    std::string name, double div_radius,
    double width, double depth, double height,
    double mass, double k, double b,
    MultibodyPlant<double>* plant) {

    DRAKE_THROW_UNLESS(plant != nullptr);

    // add model instance for lsd network multibody
    const drake::multibody::ModelInstanceIndex model_instance = plant->AddModelInstance(name);

    // vector of vertices of box: division positions
    std::vector<Vector3d> div_p_vector = {
        Vector3d( 0.5*width, -0.5*depth, -0.5*height),
        Vector3d(-0.5*width, -0.5*depth, -0.5*height),
        Vector3d(-0.5*width,  0.5*depth, -0.5*height),
        Vector3d( 0.5*width,  0.5*depth, -0.5*height),
        Vector3d(-0.5*width, -0.5*depth,  0.5*height),
        Vector3d( 0.5*width, -0.5*depth,  0.5*height),
        Vector3d(-0.5*width,  0.5*depth,  0.5*height),
        Vector3d( 0.5*width,  0.5*depth,  0.5*height),
    };

    // create a spherical division body for each position
    // int i=0; // for naming the Body of each division
    for (unsigned int i=0; i<div_p_vector.size(); i++) {
    // for (Vector3d div_p : div_p_vector) {

        // name for the body
        const std::string div_name = name + "_div_" + std::to_string(i);

        // spatial inertia (mass/8 since 8 total divisions)
        drake::multibody::SpatialInertia<double> M_Scm(
            mass/8,
            Vector3d::Zero(),
            drake::multibody::UnitInertia<double>::SolidSphere(div_radius));

        // add body to plant
        plant->AddRigidBody(div_name, model_instance, M_Scm);

        // register visual, collision geometries
        plant->RegisterVisualGeometry(
            plant->GetBodyByName(div_name), math::RigidTransformd(),
            geometry::Sphere(div_radius),
            div_name + "_visual",
            Eigen::Vector4d(66./255, 197./255, 226./255, 1));

        plant->RegisterCollisionGeometry(
            plant->GetBodyByName(div_name), math::RigidTransformd(),
            geometry::Sphere(div_radius),
            div_name + "_collision",
            drake::multibody::CoulombFriction<double>(0.3, 0.3));

        // i++;
    }

    // create LinearSpringDamper between each division
    for (unsigned int i=0; i<div_p_vector.size(); i++) {
        for (unsigned int j=i+1; j<div_p_vector.size(); j++) {

            // get names from iterators
            const std::string div_i_name = name + "_div_" + std::to_string(i);
            const std::string div_j_name = name + "_div_" + std::to_string(j);

            // calculate free length from position difference norm
            const double free_length = (div_p_vector[j] - div_p_vector[i]).norm();

            // output for debugging to see free lengths
            std::cout << div_i_name << " to " << div_j_name << "\n  free_length: " << free_length << "\n\n";

            // add LinearSpringDamper element between divisions
            plant->AddForceElement<LinearSpringDamper>(
                plant->GetBodyByName(div_i_name), Vector3d::Zero(),
                plant->GetBodyByName(div_j_name), Vector3d::Zero(),
                free_length, k, b);
        }
    }

    // // register visual box to first division to show the underlying shape
    // plant->RegisterVisualGeometry(
    //     plant->GetBodyByName(name + "_div_0"), X_WB,
    //     geometry::Box(width, depth, height),
    //     "box_visual",
    //     Eigen::Vector4d(0.5, 0.5, 1, 0.5));

    return model_instance;
}

void SetVolActuatorPose(
    systems::Context<double>* context,
    multibody::ModelInstanceIndex model_instance_index,
    std::vector<math::RigidTransformd> X_WD_vector,
    MultibodyPlant<double>* plant) {

    int i=0;
    for(auto body_index : plant->GetBodyIndices(model_instance_index)) {
        plant->SetFreeBodyPose(
            context, plant->get_body(body_index),
            X_WD_vector[i++]);

    };
}

}  // namespace project
}  // namespace drake
