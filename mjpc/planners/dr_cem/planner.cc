// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/planners/dr_cem/planner.h"

#include <absl/random/random.h>
#include <mujoco/mujoco.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <shared_mutex>

#include "mjpc/array_safety.h"
#include "mjpc/planners/planner.h"
#include "mjpc/planners/sampling/planner.h"
#include "mjpc/states/state.h"
#include "mjpc/task.h"
#include "mjpc/threadpool.h"
#include "mjpc/trajectory.h"
#include "mjpc/utilities.h"

namespace mjpc {

namespace mju = ::mujoco::util_mjpc;

// initialize data and settings
void DRCEMPlanner::Initialize(mjModel* model, const Task& task) {
  // delete mjData instances since model might have changed.
  data_.clear();

  // allocate one mjData for nominal.
  ResizeMjData(model, 1);

  // model
  this->model = model;

  // task
  this->task = &task;

  // sampling noise
  std_initial_ =
      GetNumberOrDefault(0.1, model,
                         "sampling_exploration");        // initial variance
  std_min_ = GetNumberOrDefault(0.1, model, "std_min");  // minimum variance

  // set number of trajectories to rollout
  num_rollouts_ = GetNumberOrDefault(10, model, "sampling_trajectories");

  if (num_rollouts_ > kMaxRollouts) {
    mju_error_i("Too many rollouts, %d is the maximum allowed.", kMaxRollouts);
  }

  // set number of randomized models
  num_randomized_models_ =
      GetNumberOrDefault(4, model, "num_randomized_models");

  if (num_randomized_models_ > kMaxRandomizedModels) {
    mju_error_i("Too many randomized models, %d is the maximum allowed.",
                kMaxRandomizedModels);
  }

  // set number of elite samples max(best 10%, 2)
  n_elite_ =
      GetNumberOrDefault(std::max(num_rollouts_ / 10, 2), model, "n_elite");

  // set the discount factor
  gamma_ = GetNumberOrDefault(1.0, model, "gamma");

  // add extra models for domain randomization
  randomized_models.resize(kMaxRandomizedModels);
  for (int i = 0; i < kMaxRandomizedModels; i++) {
    randomized_models[i] = mj_copyModel(nullptr, model);
  }
  task.DomainRandomize(randomized_models);
}

// allocate memory
void DRCEMPlanner::Allocate() {
  // initial state
  int num_state = model->nq + model->nv + model->na;

  // state
  state.resize(num_state);
  mocap.resize(7 * model->nmocap);
  userdata.resize(model->nuserdata);

  // policy
  int num_max_parameter = model->nu * kMaxTrajectoryHorizon;
  policy.Allocate(model, *task, kMaxTrajectoryHorizon);
  resampled_policy.Allocate(model, *task, kMaxTrajectoryHorizon);
  previous_policy.Allocate(model, *task, kMaxTrajectoryHorizon);

  // scratch
  parameters_scratch.resize(num_max_parameter);
  times_scratch.resize(kMaxTrajectoryHorizon);

  // noise
  noise.resize(kMaxRollouts * (model->nu * kMaxTrajectoryHorizon));

  // variance
  variance.resize(model->nu * kMaxTrajectoryHorizon);  // (nu * horizon)

  // need to initialize an arbitrary order of the trajectories
  trajectory_order.reserve(num_rollouts_);
  for (int i = 0; i < num_rollouts_; i++) {
    trajectory_order.push_back(i);
  }

  // trajectories and parameters
  for (int i = 0; i < kMaxRollouts * kMaxRandomizedModels; i++) {
    trajectory[i].Initialize(num_state, model->nu, task->num_residual,
                             task->num_trace, kMaxTrajectoryHorizon, gamma_);
    trajectory[i].Allocate(kMaxTrajectoryHorizon);
    candidate_policy[i].Allocate(model, *task, kMaxTrajectoryHorizon);
  }

  // elite average trajectory
  elite_avg.Initialize(num_state, model->nu, task->num_residual,
                       task->num_trace, kMaxTrajectoryHorizon, gamma_);
  elite_avg.Allocate(kMaxTrajectoryHorizon);
}

// reset memory to zeros
void DRCEMPlanner::Reset(int horizon, const double* initial_repeated_action) {
  // state
  std::fill(state.begin(), state.end(), 0.0);
  std::fill(mocap.begin(), mocap.end(), 0.0);
  std::fill(userdata.begin(), userdata.end(), 0.0);
  time = 0.0;

  // policy parameters
  policy.Reset(horizon, initial_repeated_action);
  resampled_policy.Reset(horizon, initial_repeated_action);
  previous_policy.Reset(horizon, initial_repeated_action);

  // scratch
  std::fill(parameters_scratch.begin(), parameters_scratch.end(), 0.0);
  std::fill(times_scratch.begin(), times_scratch.end(), 0.0);

  // noise
  std::fill(noise.begin(), noise.end(), 0.0);

  // variance
  double var = std_initial_ * std_initial_;
  std::fill(variance.begin(), variance.end(), var);

  // trajectory samples
  for (int i = 0; i < kMaxRollouts * kMaxRandomizedModels; i++) {
    trajectory[i].Reset(kMaxTrajectoryHorizon);
    candidate_policy[i].Reset(horizon);
  }
  elite_avg.Reset(kMaxTrajectoryHorizon);

  for (const auto& d : data_) {
    mju_zero(d->ctrl, model->nu);
  }

  // improvement
  improvement = 0.0;
}

// set state
void DRCEMPlanner::SetState(const State& state) {
  state.CopyTo(this->state.data(), this->mocap.data(), this->userdata.data(),
               &this->time);
}

// optimize nominal policy using random sampling
void DRCEMPlanner::OptimizePolicy(int horizon, ThreadPool& pool) {
  // check horizon
  if (horizon != elite_avg.horizon) {
    NominalTrajectory(horizon, pool);
  }

  // if num_rollouts_ has changed, use it in this new iteration.
  // num_rollouts_ might change while this function runs. Keep it constant
  // for the duration of this function. Same thing for num_randomized_models_.
  int num_rollouts = num_rollouts_;
  int num_randomized_models = num_randomized_models_;

  // n_elite_ might change in the GUI - keep constant for in this function
  int n_elite = std::min(n_elite_, num_rollouts);

  // resize number of mjData
  ResizeMjData(model, pool.NumThreads());

  // copy nominal policy
  policy.num_parameters = model->nu * policy.num_spline_points;
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    resampled_policy.CopyFrom(policy, policy.num_spline_points);
  }

  // resample nominal policy to current time
  this->ResamplePolicy(horizon);

  // Get new domain randomization parameters
  task->DomainRandomize(randomized_models);

  // ----- rollout noisy policies ----- //
  // start timer
  auto rollouts_start = std::chrono::steady_clock::now();

  // simulate noisy policies
  this->Rollouts(num_rollouts, num_randomized_models, horizon, pool);

  // sort candidate policies and trajectories by score
  for (int i = 0; i < num_rollouts; i++) {
    trajectory_order[i] = i;
  }

  // sort so that the first ncandidates elements are the best candidates, and
  // the rest are in an unspecified order
  std::partial_sort(
      trajectory_order.begin(),
      trajectory_order.begin() + num_rollouts,  // TODO: should be n_elite??
      trajectory_order.begin() + num_rollouts,
      [&trajectory = trajectory](int a, int b) {
        return trajectory[a].total_return < trajectory[b].total_return;
      });

  // stop timer
  rollouts_compute_time = GetDuration(rollouts_start);

  // ----- update policy ----- //
  // start timer
  auto policy_update_start = std::chrono::steady_clock::now();

  // dimensions
  int num_spline_points = resampled_policy.num_spline_points;
  int num_parameters = resampled_policy.num_parameters;

  // reset parameters scratch to zero
  std::fill(parameters_scratch.begin(), parameters_scratch.end(), 0.0);

  // reset elite average
  elite_avg.Reset(horizon);

  // set elite average trajectory times
  for (int tt = 0; tt <= horizon; tt++) {
    elite_avg.times[tt] = time + tt * model->opt.timestep;
  }

  // best elite
  int idx = trajectory_order[0];

  // add parameters
  mju_copy(parameters_scratch.data(), candidate_policy[idx].parameters.data(),
           num_parameters);

  // copy first elite trajectory
  mju_copy(elite_avg.actions.data(), trajectory[idx].actions.data(),
           model->nu * (horizon - 1));
  mju_copy(elite_avg.trace.data(), trajectory[idx].trace.data(),
           trajectory[idx].dim_trace * horizon);
  mju_copy(elite_avg.residual.data(), trajectory[idx].residual.data(),
           elite_avg.dim_residual * horizon);
  mju_copy(elite_avg.costs.data(), trajectory[idx].costs.data(), horizon);
  elite_avg.total_return = trajectory[idx].total_return;

  // loop over remaining elites to compute average
  for (int i = 1; i < n_elite; i++) {
    // ordered trajectory index
    int idx = trajectory_order[i];

    // add parameters
    mju_addTo(parameters_scratch.data(),
              candidate_policy[idx].parameters.data(), num_parameters);

    // add elite trajectory
    mju_addTo(elite_avg.actions.data(), trajectory[idx].actions.data(),
              model->nu * (horizon - 1));
    mju_addTo(elite_avg.trace.data(), trajectory[idx].trace.data(),
              trajectory[idx].dim_trace * horizon);
    mju_addTo(elite_avg.residual.data(), trajectory[idx].residual.data(),
              elite_avg.dim_residual * horizon);
    mju_addTo(elite_avg.costs.data(), trajectory[idx].costs.data(), horizon);
    elite_avg.total_return += trajectory[idx].total_return;
  }

  // normalize
  mju_scl(parameters_scratch.data(), parameters_scratch.data(), 1.0 / n_elite,
          num_parameters);
  mju_scl(elite_avg.actions.data(), elite_avg.actions.data(), 1.0 / n_elite,
          model->nu * (horizon - 1));
  mju_scl(elite_avg.trace.data(), elite_avg.trace.data(), 1.0 / n_elite,
          elite_avg.dim_trace * horizon);
  mju_scl(elite_avg.residual.data(), elite_avg.residual.data(), 1.0 / n_elite,
          elite_avg.dim_residual * horizon);
  mju_scl(elite_avg.costs.data(), elite_avg.costs.data(), 1.0 / n_elite,
          horizon);
  elite_avg.total_return /= n_elite;

  // loop over elites to compute variance
  std::fill(variance.begin(), variance.end(), 0.0);  // reset variance to zero
  for (int t = 0; t < num_spline_points; t++) {
    for (int j = 0; j < model->nu; j++) {
      // average
      double p_avg = parameters_scratch[t * model->nu + j];
      for (int i = 0; i < n_elite; i++) {
        // candidate parameter
        double pi =
            candidate_policy[trajectory_order[i]].parameters[t * model->nu + j];
        double diff = pi - p_avg;
        variance[t * model->nu + j] += diff * diff / (n_elite - 1);
      }
    }
  }

  // update
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    policy.CopyParametersFrom(parameters_scratch, times_scratch);
  }

  // improvement: compare nominal to elite average
  improvement = mju_max(
      elite_avg.total_return - trajectory[trajectory_order[0]].total_return,
      0.0);

  // stop timer
  policy_update_compute_time = GetDuration(policy_update_start);
}

// compute trajectory using nominal policy
void DRCEMPlanner::NominalTrajectory(int horizon, ThreadPool& pool) {
  // set policy
  auto nominal_policy = [&cp = resampled_policy](
                            double* action, const double* state, double time) {
    cp.Action(action, state, time);
  };

  // rollout nominal policy
  elite_avg.Rollout(nominal_policy, task, model, data_[0].get(), state.data(),
                    time, mocap.data(), userdata.data(), horizon);
}

// set action from policy
void DRCEMPlanner::ActionFromPolicy(double* action, const double* state,
                                    double time, bool use_previous) {
  const std::shared_lock<std::shared_mutex> lock(mtx_);
  if (use_previous) {
    previous_policy.Action(action, state, time);
  } else {
    policy.Action(action, state, time);
  }
}

// update policy via resampling
void DRCEMPlanner::ResamplePolicy(int horizon) {
  // dimensions
  int num_parameters = resampled_policy.num_parameters;
  int num_spline_points = resampled_policy.num_spline_points;

  // time
  double nominal_time = time;
  double time_shift = mju_max(
      (horizon - 1) * model->opt.timestep / (num_spline_points - 1), 1.0e-5);

  // get spline points
  for (int t = 0; t < num_spline_points; t++) {
    times_scratch[t] = nominal_time;
    resampled_policy.Action(DataAt(parameters_scratch, t * model->nu), nullptr,
                            nominal_time);
    nominal_time += time_shift;
  }

  // copy resampled policy parameters
  mju_copy(resampled_policy.parameters.data(), parameters_scratch.data(),
           num_parameters);
  mju_copy(resampled_policy.times.data(), times_scratch.data(),
           num_spline_points);

  LinearRange(resampled_policy.times.data(), time_shift,
              resampled_policy.times[0], num_spline_points);
}

// add random noise to nominal policy
void DRCEMPlanner::AddNoiseToPolicy(int i, double std_min) {
  // start timer
  auto noise_start = std::chrono::steady_clock::now();

  // dimensions
  int num_spline_points = candidate_policy[i].num_spline_points;
  int num_parameters = candidate_policy[i].num_parameters;

  // sampling token
  absl::BitGen gen_;

  // shift index
  int shift = i * (model->nu * kMaxTrajectoryHorizon);

  // sample noise
  // variance[k] is the standard deviation for the k^th control parameter over
  // the elite samples we draw a bunch of control actions from this distribution
  // (which i indexes) - the noise is stored in `noise`.
  for (int k = 0; k < num_parameters; k++) {
    noise[k + shift] = absl::Gaussian<double>(
        gen_, 0.0, std::max(std::sqrt(variance[k]), std_min));
  }

  // add noise
  mju_addTo(candidate_policy[i].parameters.data(), DataAt(noise, shift),
            num_parameters);

  // clamp parameters
  for (int t = 0; t < num_spline_points; t++) {
    Clamp(DataAt(candidate_policy[i].parameters, t * model->nu),
          model->actuator_ctrlrange, model->nu);
  }

  // end timer
  IncrementAtomic(noise_compute_time, GetDuration(noise_start));
}

// compute candidate trajectories
void DRCEMPlanner::Rollouts(int num_rollouts, int num_randomized_models,
                            int horizon, ThreadPool& pool) {
  // reset noise compute time
  noise_compute_time = 0.0;

  // lock std_min
  double std_min = std_min_;

  policy.num_parameters = model->nu * policy.num_spline_points;

  // compute num_rollouts random control tapes, storing each one in
  // this->candidate_policy[i]. Additional copies of each tape are stored in
  // this->candidate_policy[i + j*num_rollouts] for
  // j=1,...,num_randomized_models
  int count_before = pool.GetCount();
  for (int i = 0; i < num_rollouts; i++) {
    pool.Schedule([&s = *this, i, num_rollouts, num_randomized_models,
                   std_min]() {
      // copy nominal policy
      {
        const std::shared_lock<std::shared_mutex> lock(s.mtx_);
        s.candidate_policy[i].CopyFrom(s.policy, s.policy.num_spline_points);
        s.candidate_policy[i].representation = s.policy.representation;
      }

      // add random noise to the policy
      if (i != 0) s.AddNoiseToPolicy(i, std_min);

      // make copies of the candidate policy for each randomized model
      for (int j = 1; j < num_randomized_models; j++) {
        int k = i + j * num_rollouts;
        s.candidate_policy[k].CopyFrom(s.candidate_policy[i],
                                       s.candidate_policy[i].num_spline_points);
        s.candidate_policy[k].representation =
            s.candidate_policy[i].representation;
      }
    });
  }
  pool.WaitCount(count_before + num_rollouts);
  pool.ResetCount();

  // Roll out the control tapes across the randomized models. Tape i for model j
  // is stored in this->trajectory[i + j*num_rollouts].
  count_before = pool.GetCount();
  for (int i = 0; i < num_rollouts; i++) {
    for (int j = 0; j < num_randomized_models; j++) {
      pool.Schedule(
          [&s = *this, &model = this->randomized_models[j], &task = this->task,
           &state = this->state, &time = this->time, &mocap = this->mocap,
           &userdata = this->userdata, horizon, i, j, num_rollouts]() {
            // policy helper function
            int k = i + j * num_rollouts;
            auto sample_policy = [&candidate_policy = s.candidate_policy, &k](
                                     double* action, const double* state,
                                     double time) {
              candidate_policy[k].Action(action, state, time);
            };

            // policy rollout
            s.trajectory[k].Rollout(sample_policy, task, model,
                                    s.data_[ThreadPool::WorkerId()].get(),
                                    state.data(), time, mocap.data(),
                                    userdata.data(), horizon);
          });
    }
  }
  pool.WaitCount(count_before + num_rollouts * num_randomized_models);
  pool.ResetCount();

  // compute average trajectory costs across the randomized models, storing
  // them in this->trajectory[i].total_return (the first time rollout i is
  // used). Thus the first num_rollouts elements of this->trajectory are scored
  // by average performance across the randomized models.
  for (int i = 0; i < num_rollouts; ++i) {
    for (int j = 1; j < num_randomized_models; ++j) {
      std::cout << trajectory[i + j * num_rollouts].total_return << " ";
      trajectory[i].total_return +=
          trajectory[i + j * num_rollouts].total_return;
    }
    std::cout << std::endl;
    trajectory[i].total_return /= num_randomized_models;
  }
}

// returns the nominal trajectory (this is the purple trace)
const Trajectory* DRCEMPlanner::BestTrajectory() { return &elite_avg; }

// visualize planner-specific traces
void DRCEMPlanner::Traces(mjvScene* scn) {
  // sample color
  float color[4];
  color[0] = 1.0;
  color[1] = 1.0;
  color[2] = 1.0;
  color[3] = 1.0;

  // width of a sample trace, in pixels
  double width = GetNumberOrDefault(3, model, "agent_sample_width");

  // scratch
  double zero3[3] = {0};
  double zero9[9] = {0};

  // best
  auto best = this->BestTrajectory();

  // sample traces
  for (int k = 0; k < num_rollouts_ * num_randomized_models_; k++) {
    // plot sample
    for (int i = 0; i < best->horizon - 1; i++) {
      if (scn->ngeom + task->num_trace > scn->maxgeom) break;
      for (int j = 0; j < task->num_trace; j++) {
        // initialize geometry
        mjv_initGeom(&scn->geoms[scn->ngeom], mjGEOM_LINE, zero3, zero3, zero9,
                     color);

        // make geometry
        mjv_makeConnector(
            &scn->geoms[scn->ngeom], mjGEOM_LINE, width,
            trajectory[k].trace[3 * task->num_trace * i + 3 * j],
            trajectory[k].trace[3 * task->num_trace * i + 1 + 3 * j],
            trajectory[k].trace[3 * task->num_trace * i + 2 + 3 * j],
            trajectory[k].trace[3 * task->num_trace * (i + 1) + 3 * j],
            trajectory[k].trace[3 * task->num_trace * (i + 1) + 1 + 3 * j],
            trajectory[k].trace[3 * task->num_trace * (i + 1) + 2 + 3 * j]);

        // increment number of geometries
        scn->ngeom += 1;
      }
    }
  }
}

// planner-specific GUI elements
void DRCEMPlanner::GUI(mjUI& ui) {
  mjuiDef defCrossEntropy[] = {
      {mjITEM_SLIDERINT, "Models", 2, &num_randomized_models_, "0 1"},
      {mjITEM_SLIDERINT, "Rollouts", 2, &num_rollouts_, "0 1"},
      {mjITEM_SELECT, "Spline", 2, &policy.representation,
       "Zero\nLinear\nCubic"},
      {mjITEM_SLIDERINT, "Spline Pts", 2, &policy.num_spline_points, "0 1"},
      {mjITEM_SLIDERNUM, "Init. Std", 2, &std_initial_, "0 1"},
      {mjITEM_SLIDERNUM, "Min. Std", 2, &std_min_, "0.01 1.0"},
      {mjITEM_SLIDERINT, "Elite", 2, &n_elite_, "2 128"},
      {mjITEM_END}};

  // set number of models slider limits
  mju::sprintf_arr(defCrossEntropy[0].other, "%i %i", 1, kMaxRandomizedModels);

  // set number of trajectory slider limits
  mju::sprintf_arr(defCrossEntropy[1].other, "%i %i", 1, kMaxRollouts);

  // set spline point limits
  mju::sprintf_arr(defCrossEntropy[3].other, "%i %i", MinSamplingSplinePoints,
                   MaxSamplingSplinePoints);

  // set noise standard deviation limits
  mju::sprintf_arr(defCrossEntropy[4].other, "%f %f", MinNoiseStdDev,
                   MaxNoiseStdDev);

  // add cross entropy planner
  mjui_add(&ui, defCrossEntropy);
}

// planner-specific plots
void DRCEMPlanner::Plots(mjvFigure* fig_planner, mjvFigure* fig_timer,
                         int planner_shift, int timer_shift, int planning,
                         int* shift) {
  // ----- planner ----- //
  double planner_bounds[2] = {-6.0, 6.0};

  // improvement
  mjpc::PlotUpdateData(fig_planner, planner_bounds,
                       fig_planner->linedata[0 + planner_shift][0] + 1,
                       mju_log10(mju_max(improvement, 1.0e-6)), 100,
                       0 + planner_shift, 0, 1, -100);

  // legend
  mju::strcpy_arr(fig_planner->linename[0 + planner_shift], "Avg - Best");

  fig_planner->range[1][0] = planner_bounds[0];
  fig_planner->range[1][1] = planner_bounds[1];

  // bounds
  double timer_bounds[2] = {0.0, 1.0};

  // ----- timer ----- //

  PlotUpdateData(
      fig_timer, timer_bounds, fig_timer->linedata[0 + timer_shift][0] + 1,
      1.0e-3 * noise_compute_time * planning, 100, 0 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[1 + timer_shift][0] + 1,
                 1.0e-3 * rollouts_compute_time * planning, 100,
                 1 + timer_shift, 0, 1, -100);

  PlotUpdateData(fig_timer, timer_bounds,
                 fig_timer->linedata[2 + timer_shift][0] + 1,
                 1.0e-3 * policy_update_compute_time * planning, 100,
                 2 + timer_shift, 0, 1, -100);

  // legend
  mju::strcpy_arr(fig_timer->linename[0 + timer_shift], "Noise");
  mju::strcpy_arr(fig_timer->linename[1 + timer_shift], "Rollout");
  mju::strcpy_arr(fig_timer->linename[2 + timer_shift], "Policy Update");

  // planner shift
  shift[0] += 1;

  // timer shift
  shift[1] += 3;
}

}  // namespace mjpc
