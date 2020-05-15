#include "drake/project/sequence_controller.h"

namespace drake{
namespace project{

SequenceController::SequenceController(int size, int num_spokes)
  : SingleOutputVectorSource(size*num_spokes), actuation_state_(Eigen::VectorXd(size)), num_spokes_(num_spokes) {
  // : SingleOutputVectorSource(2), actuation_state_(Eigen::VectorXd(size)), num_spokes_(num_spokes) {
}

void SequenceController::DoCalcVectorOutput(const systems::Context<double>& context,
  Eigen::VectorBlock<VectorX<double>>* output) const {

  Eigen::VectorXd output_vector(actuation_state_.size()*num_spokes_);
  for (unsigned int i=0; i<actuation_state_.size(); i++) {
    // log()->info("a value: {}", actuation_state_(i));
    for (unsigned int j=i*6; j<(i+1)*6; j++) {
      // log()->info("{}", j);
      output_vector(j) = actuation_state_(i);
    }
  }

  *output = output_vector;
  // *output = Eigen::Vector2d(.1, .1);
  double t = context.get_time();
  t += 1;
  // log()->info("Time is {}", context.get_time());
}



void SequenceController::SetState(const Eigen::VectorXd& state) {
  // log()->info("Sizes {}, {}", state.size(), actuation_state_.size());

  actuation_state_ = state;
}

void SequenceController::PrintState() {
  log()->info("State is\n{}", actuation_state_);
}

} // namespace project
} // namespace drake
