#include "mjpc/tasks/spinner/spinner.h"

#include <cmath>
#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
std::string Spinner::XmlPath() const {
  return GetModelPath("spinner/task.xml");
}
std::string Spinner::Name() const { return "Spinner"; }

// ------- Residuals for spinner task ------
//     Position: joint angles should match targets
//     Velocities: joint velocities should be small
//     Control:  Control should be small
// ------------------------------------------
void Spinner::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                    double* residual) const {
  // ---------- Finger joint positions ----------
  residual[0] = data->qpos[0] - 0.3;
  residual[1] = data->qpos[1] - 1.5;

  // ---------- Spinner angle ----------
  residual[2] = data->qpos[2] - 2.0;

  // ---------- Velocities ----------
  residual[3] = data->qvel[0];
  residual[4] = data->qvel[1];
  residual[5] = data->qvel[2];

  // ---------- Controls ----------
  residual[6] = data->ctrl[0];
  residual[7] = data->ctrl[1];
}

}  // namespace mjpc
