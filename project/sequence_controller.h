#pragma once

#include <memory>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace project {

class SequenceController : public systems::LeafSystem<double> {
public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(SequenceController)

  explicit SequenceController();

}

}  // namespace examples
}  // namespace drake
