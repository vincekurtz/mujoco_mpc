#include "mjpc/tasks/allegro_cube/allegro_cube.h"
#include <iostream>

#include <cmath>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {
std::string AllegroCube::XmlPath() const {
  return GetModelPath("allegro_cube/task.xml");
}
std::string AllegroCube::Name() const { return "AllegroCube"; }

// ------- Residuals for cube manipulation task ------
//     Cube position: (3)
//     Cube orientation: (3)
//     Cube linear velocity: (3)
//     Control:  u
// ------------------------------------------
void AllegroCube::ResidualFn::Residual(const mjModel* model, const mjData* data,
                        double* residual) const {
  int counter = 0;

  // ---------- Cube position ----------
  // TODO(vincekurtz): specify goal position in a better way
  //double* cube_position = SensorByName(model, data, "cube_position");
  //std::vector<double> goal_cube_position = {0.325, 0.0, 0.025};

  //mju_sub3(residual + counter, cube_position, goal_cube_position.data());
  //counter += 3;

  //// ---------- Cube orientation ----------
  //double* cube_orientation = SensorByName(model, data, "cube_orientation");
  //double* goal_cube_orientation = SensorByName(model, data, "cube_goal_orientation");
  //mju_normalize4(goal_cube_orientation);

  //mju_subQuat(residual + counter, goal_cube_orientation, cube_orientation);
  //counter += 3;

  //// ---------- Cube linear velocity ----------
  //double* cube_linear_velocity =
  //    SensorByName(model, data, "cube_linear_velocity");

  //mju_copy(residual + counter, cube_linear_velocity, 3);
  //counter += 3;

  // ---------- Position ----------
  std::vector<double> q_nom = {0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 0.0, 0.5, 0.5, 0.5, 1.0, 0.8, 0.5, 0.5};
  mju_sub(residual + counter, data->qpos, q_nom.data(), model->nq);
  counter += model->nq;

  // ---------- Velocity ----------
  mju_copy(residual + counter, data->qvel, model->nv);
  counter += model->nv;

  // ---------- Control ----------
  mju_copy(residual + counter, data->actuator_force, model->nu);
  counter += model->nu;

  // Sanity check
  CheckSensorDim(model, counter);
}


}  // namespace mjpc
