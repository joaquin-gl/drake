#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"
#include <drake/systems/framework/single_output_vector_source.h>

namespace drake {
namespace project {

class SequenceController : public systems::SingleOutputVectorSource<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SequenceController)

  explicit SequenceController(int size, int num_spokes);
  void SetState(const Eigen::VectorXd& state);
  void PrintState();

private:
  void DoCalcVectorOutput(
      const systems::Context<double>& context,
      Eigen::VectorBlock<VectorX<double>>* output) const final;
      // Eigen::VectorX<double>* output) const final;

  // Eigen::BasicVector<double> actuation_state_;
  Eigen::VectorXd actuation_state_;
  int num_spokes_;

};

}  // namespace examples
}  // namespace drake
