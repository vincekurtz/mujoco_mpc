#ifndef MJPC_TASKS_SPINNER_SPINNER_H_
#define MJPC_TASKS_SPINNER_SPINNER_H_

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"

namespace mjpc {
class Spinner : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public BaseResidualFn {
   public:
    explicit ResidualFn(const Spinner* task) : BaseResidualFn(task) {}
    // ------- Residuals for spinner task ------
    //   Number of residuals: 8
    //     Residual (0:3): positions
    //     Residual (3:6): velocities
    //     Residual (7:8): control torques
    // ------------------------------------------
    void Residual(const mjModel* model, const mjData* data,
                  double* residual) const override;
  };

  Spinner() : residual_(this) {}

 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this);
  }
  ResidualFn* InternalResidual() override { return &residual_; }

 private:
  ResidualFn residual_;
};
}  // namespace mjpc

#endif  // MJPC_TASKS_SPIINNER_SPINNER_H_
