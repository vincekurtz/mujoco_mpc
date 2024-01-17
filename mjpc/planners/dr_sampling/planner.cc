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

#include "mjpc/planners/dr_sampling/planner.h"

#include <algorithm>
#include <chrono>
#include <mutex>
#include <shared_mutex>

#include <absl/random/random.h>
#include <mujoco/mujoco.h>
#include "mjpc/array_safety.h"
#include "mjpc/planners/policy.h"
#include "mjpc/states/state.h"
#include "mjpc/trajectory.h"
#include "mjpc/utilities.h"

namespace mjpc {

namespace mju = ::mujoco::util_mjpc;

// initialize data and settings
void DRSamplingPlanner::Initialize(mjModel* model, const Task& task) {
  // delete mjData instances since model might have changed.
  data_.clear();
  // allocate one mjData for nominal.
  ResizeMjData(model, 1);

  // model
  this->model = model;

  // task
  this->task = &task;

  // rollout parameters
  timestep_power = 1.0;

  // sampling noise
  noise_exploration = GetNumberOrDefault(0.1, model, "sampling_exploration");

  // set number of trajectories to rollout for each model
  num_rollouts_ = GetNumberOrDefault(10, model, "sampling_trajectories");
  
  if (num_rollouts_ > kMaxRollouts) {
    mju_error_i("Too many trajectories, %d is the maximum allowed.",
                kMaxRollouts);
  }

  // set number of randomized models
  num_randomized_models_ = GetNumberOrDefault(3, model, "num_randomized_models");

  if (num_randomized_models_ > kMaxRandomizedModels) {
    mju_error_i("Too many randomized models, %d is the maximum allowed.",
                kMaxRandomizedModels);
  }
  
  // add extra models for domain randomization
  randomized_models.resize(kMaxRandomizedModels);
  for (int i = 0; i < kMaxRandomizedModels; i++) {
    randomized_models[i] = mj_copyModel(nullptr, model);
  }
  task.DomainRandomize(randomized_models);

  winner = 0;
}

// allocate memory
void DRSamplingPlanner::Allocate() {
  // initial state
  int num_state = model->nq + model->nv + model->na;

  // state
  state.resize(num_state);
  mocap.resize(7 * model->nmocap);
  userdata.resize(model->nuserdata);

  // policy
  int num_max_parameter = model->nu * kMaxTrajectoryHorizon;
  policy.Allocate(model, *task, kMaxTrajectoryHorizon);
  previous_policy.Allocate(model, *task, kMaxTrajectoryHorizon);

  // scratch
  parameters_scratch.resize(num_max_parameter);
  times_scratch.resize(kMaxTrajectoryHorizon);

  // noise
  noise.resize(kMaxRollouts * (model->nu * kMaxTrajectoryHorizon));

  // trajectory and parameters
  winner = -1;
  for (int i = 0; i < kMaxRollouts * kMaxRandomizedModels; i++) {
    trajectory[i].Initialize(num_state, model->nu, task->num_residual,
                             task->num_trace, kMaxTrajectoryHorizon);
    trajectory[i].Allocate(kMaxTrajectoryHorizon);
    candidate_policy[i].Allocate(model, *task, kMaxTrajectoryHorizon);
  }
}

// reset memory to zeros
void DRSamplingPlanner::Reset(int horizon) {
  // state
  std::fill(state.begin(), state.end(), 0.0);
  std::fill(mocap.begin(), mocap.end(), 0.0);
  std::fill(userdata.begin(), userdata.end(), 0.0);
  time = 0.0;

  // policy parameters
  policy.Reset(horizon);
  previous_policy.Reset(horizon);

  // scratch
  std::fill(parameters_scratch.begin(), parameters_scratch.end(), 0.0);
  std::fill(times_scratch.begin(), times_scratch.end(), 0.0);

  // noise
  std::fill(noise.begin(), noise.end(), 0.0);

  // trajectory samples
  for (int i = 0; i < kMaxRollouts * kMaxRandomizedModels; i++) {
    trajectory[i].Reset(kMaxTrajectoryHorizon);
    candidate_policy[i].Reset(horizon);
  }

  for (const auto& d : data_) {
    mju_zero(d->ctrl, model->nu);
  }

  // improvement
  improvement = 0.0;

  // winner
  winner = 0;
}

// set state
void DRSamplingPlanner::SetState(const State& state) {
  state.CopyTo(this->state.data(), this->mocap.data(), this->userdata.data(),
               &this->time);
}

int DRSamplingPlanner::OptimizePolicyCandidates(int ncandidates, int horizon,
                                              ThreadPool& pool) {
  // if num_rollouts_ has changed, use it in this new iteration.
  // num_rollouts_ might change while this function runs. Keep it constant
  // for the duration of this function. Same thing for num_randomized_models_.
  int num_rollouts = num_rollouts_;
  int num_randomized_models = num_randomized_models_;
  ncandidates = std::min(ncandidates, num_rollouts);
  ResizeMjData(model, pool.NumThreads());

  // ----- rollout noisy policies ----- //
  // start timer
  auto rollouts_start = std::chrono::steady_clock::now();

  // simulate noisy policies
  this->Rollouts(num_rollouts, num_randomized_models, horizon, pool);

  // sort candidate policies and trajectories by score
  trajectory_order.clear();
  trajectory_order.reserve(num_rollouts);
  for (int i = 0; i < num_rollouts; i++) {
    trajectory_order.push_back(i);
  }

  // sort so that the first ncandidates elements are the best candidates, and
  // the rest are in an unspecified order
  std::partial_sort(
      trajectory_order.begin(), trajectory_order.begin() + ncandidates,
      trajectory_order.end(), [trajectory = trajectory](int a, int b) {
        return trajectory[a].total_return < trajectory[b].total_return;
      });

  // stop timer
  rollouts_compute_time = GetDuration(rollouts_start);

  return ncandidates;
}

// optimize nominal policy using random sampling
void DRSamplingPlanner::OptimizePolicy(int horizon, ThreadPool& pool) {
  // resample nominal policy to current time
  this->UpdateNominalPolicy(horizon);

  OptimizePolicyCandidates(1, horizon, pool);

  // ----- update policy ----- //
  // start timer
  auto policy_update_start = std::chrono::steady_clock::now();

  CopyCandidateToPolicy(0);

  // improvement: compare nominal to winner
  double best_return = trajectory[0].total_return;
  improvement = mju_max(best_return - trajectory[winner].total_return, 0.0);

  // stop timer
  policy_update_compute_time = GetDuration(policy_update_start);
}

// compute trajectory using nominal policy
void DRSamplingPlanner::NominalTrajectory(int horizon, ThreadPool& pool) {
  // set policy
  auto nominal_policy = [&cp = candidate_policy[0]](
                            double* action, const double* state, double time) {
    cp.Action(action, state, time);
  };

  // rollout nominal policy
  trajectory[0].Rollout(nominal_policy, task, model, data_[0].get(),
                        state.data(), time, mocap.data(), userdata.data(),
                        horizon);
}

// set action from policy
void DRSamplingPlanner::ActionFromPolicy(double* action, const double* state,
                                       double time, bool use_previous) {
  const std::shared_lock<std::shared_mutex> lock(mtx_);
  if (use_previous) {
    previous_policy.Action(action, state, time);
  } else {
    policy.Action(action, state, time);
  }
}

// update policy via resampling
void DRSamplingPlanner::UpdateNominalPolicy(int horizon) {
  // dimensions
  int num_spline_points = candidate_policy[winner].num_spline_points;

  // set time
  double nominal_time = time;
  double time_shift = mju_max(
      (horizon - 1) * model->opt.timestep / (num_spline_points - 1), 1.0e-5);

  // get spline points
  for (int t = 0; t < num_spline_points; t++) {
    times_scratch[t] = nominal_time;
    candidate_policy[winner].Action(DataAt(parameters_scratch, t * model->nu),
                               nullptr, nominal_time);
    nominal_time += time_shift;
  }

  // update
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    // parameters
    policy.CopyParametersFrom(parameters_scratch, times_scratch);

    // time power transformation
    PowerSequence(policy.times.data(), time_shift,
                  policy.times[0],
                  policy.times[num_spline_points - 1],
                  timestep_power, num_spline_points);
  }
}

// add random noise to nominal policy
void DRSamplingPlanner::AddNoiseToPolicy(int i) {
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
  for (int k = 0; k < num_parameters; k++) {
    noise[k + shift] = absl::Gaussian<double>(gen_, 0.0, noise_exploration);
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
void DRSamplingPlanner::Rollouts(int num_rollouts, int num_randomized_models, 
                                 int horizon, ThreadPool& pool) {
  // reset noise compute time
  noise_compute_time = 0.0;

  policy.num_parameters = model->nu * policy.num_spline_points;

  // compute num_rollouts random control tapes, storing each one in
  // this->candidate_policy[i]. Additional copies of each tape are stored in
  // this->candidate_policy[i + j*num_rollouts] for j=1,...,num_randomized_models
  int count_before = pool.GetCount();
  for (int i = 0; i < num_rollouts; i++) {
    pool.Schedule([&s = *this, i, num_rollouts, num_randomized_models]() {
      // copy nominal policy
      {
        const std::shared_lock<std::shared_mutex> lock(s.mtx_);
        s.candidate_policy[i].CopyFrom(s.policy, s.policy.num_spline_points);
        s.candidate_policy[i].representation = s.policy.representation;
      }

      // add random noise to the policy
      if (i != 0) s.AddNoiseToPolicy(i);

      // make copies of the candidate policy for each randomized model
      for (int j = 1; j < num_randomized_models; j++) {
        int k = i + j * num_rollouts;
        s.candidate_policy[k].CopyFrom(
            s.candidate_policy[i], s.candidate_policy[i].num_spline_points);
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
  for (int i=0; i < num_rollouts; i++) {
    for (int j=0; j < num_randomized_models; j++) {
      pool.Schedule([&s = *this, &model = this->model, &task = this->task,
                    &state = this->state, &time = this->time,
                    &mocap = this->mocap, &userdata = this->userdata, horizon,
                    i, j, num_rollouts]()
                    {
        // policy helper function
        int k = i + j * num_rollouts;
        auto sample_policy = [&candidate_policy = s.candidate_policy, &k](
                                  double* action, const double* state,
                                  double time) {
          candidate_policy[k].Action(action, state, time);
        };

        // policy rollout
        s.trajectory[k].Rollout(
            sample_policy, task, model, s.data_[ThreadPool::WorkerId()].get(),
            state.data(), time, mocap.data(), userdata.data(), horizon); 
      });
    }
  }
  pool.WaitCount(count_before + num_rollouts*num_randomized_models);
  pool.ResetCount();

  // compute average trajectory costs across the randomized models, storing
  // them in this->trajectory[i].total_return (the first time rollout i is used).
  // Thus the first num_rollouts elements of this->trajectory are scored by
  // average performance across the randomized models.
  for (int i=0; i<num_rollouts; ++i) {
    for (int j=1; j<num_randomized_models; ++j) {
      trajectory[i].total_return += trajectory[i + j*num_rollouts].total_return;
    }
    trajectory[i].total_return /= num_randomized_models;
  }
}

// return trajectory with best total return
const Trajectory* DRSamplingPlanner::BestTrajectory() {
  return winner >= 0 ? &trajectory[winner] : nullptr;
}

// visualize planner-specific traces
void DRSamplingPlanner::Traces(mjvScene* scn) {
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
  for (int k = 0; k < num_rollouts_; k++) {
    // skip winner
    if (k == winner) continue;

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
void DRSamplingPlanner::GUI(mjUI& ui) {
  mjuiDef defSampling[] = {
      {mjITEM_SLIDERINT, "Models", 2, &num_randomized_models_, "0 1"},
      {mjITEM_SLIDERINT, "Rollouts", 2, &num_rollouts_, "0 1"},
      {mjITEM_SELECT, "Spline", 2, &policy.representation,
       "Zero\nLinear\nCubic"},
      {mjITEM_SLIDERINT, "Spline Pts", 2, &policy.num_spline_points, "0 1"},
      // {mjITEM_SLIDERNUM, "Spline Pow. ", 2, &timestep_power, "0 10"},
      // {mjITEM_SELECT, "Noise type", 2, &noise_type, "Gaussian\nUniform"},
      {mjITEM_SLIDERNUM, "Noise Std", 2, &noise_exploration, "0 1"},
      {mjITEM_END}};

  // set number of randomized models slider limits
  mju::sprintf_arr(defSampling[0].other, "%i %i", 1, kMaxRandomizedModels);
  
  // set number of trajectory slider limits
  mju::sprintf_arr(defSampling[1].other, "%i %i", 1, kMaxRollouts);

  // set spline point limits
  mju::sprintf_arr(defSampling[3].other, "%i %i", MinSamplingSplinePoints,
                   MaxSamplingSplinePoints);

  // set noise standard deviation limits
  mju::sprintf_arr(defSampling[4].other, "%f %f", MinNoiseStdDev,
                   MaxNoiseStdDev);

  // add sampling planner
  mjui_add(&ui, defSampling);
}

// planner-specific plots
void DRSamplingPlanner::Plots(mjvFigure* fig_planner, mjvFigure* fig_timer,
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
  mju::strcpy_arr(fig_planner->linename[0 + planner_shift], "Improvement");

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

double DRSamplingPlanner::CandidateScore(int candidate) const {
  return trajectory[trajectory_order[candidate]].total_return;
}

// set action from candidate policy
void DRSamplingPlanner::ActionFromCandidatePolicy(double* action, int candidate,
                                                const double* state,
                                                double time) {
  candidate_policy[trajectory_order[candidate]].Action(action, state, time);
}

void DRSamplingPlanner::CopyCandidateToPolicy(int candidate) {
  // set winner
  winner = trajectory_order[candidate];

  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    previous_policy = policy;
    policy = candidate_policy[winner];
  }
}
}  // namespace mjpc
