/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2016 Henner Zeller <h.zeller@acm.org>
 *
 * This file is part of BeagleG. http://github.com/hzeller/beagleg
 *
 * BeagleG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * BeagleG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BeagleG.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <math.h>
#include <stdlib.h>

#include "planner.h"
#include "hardware-mapping.h"
#include "gcode-machine-control.h"
#include "motor-operations.h"
#include "logging.h"
#include "container.h"

// The target position vector is essentially a position in the
// GCODE_NUM_AXES-dimensional space.
//
// An AxisTarget has a position vector, in absolute machine coordinates, and a
// speed when arriving at that position.
//
// The speed is initially the aimed goal; if it cannot be reached, the
// AxisTarget will be modified to contain the actually reachable value. That is
// used in planning along the path.
struct AxisTarget {
  AxesRegister target;   // Target coordinate
  float speed;           // Speed in euclidian space
  float dx, dy, dz;      // 3D distance to previous position.
  float len;             // 3D length
  unsigned short aux_bits;             // Auxillary bits in this segment; set with M42
};

class Planner::Impl {
public:
  Impl(const MachineControlConfig *config,
       HardwareMapping *hardware_mapping,
       MotorOperations *motor_backend);
  ~Impl();

  void move_machine_steps(const struct AxisTarget *last_pos,
                          struct AxisTarget *target_pos,
                          const struct AxisTarget *upcoming);

  void assign_steps_to_motors(struct LinearSegmentSteps *command,
                              enum GCodeParserAxis axis,
                              int steps);

  void issue_motor_move_if_possible();
  void machine_move(const AxesRegister &axis, float feedrate);
  void bring_path_to_halt();

  void GetCurrentPosition(AxesRegister *pos);
  int DirectDrive(GCodeParserAxis axis, float distance, float v0, float v1);
  void SetExternalPosition(GCodeParserAxis axis, float pos);

private:
  const struct MachineControlConfig *const cfg_;
  HardwareMapping *const hardware_mapping_;
  MotorOperations *const motor_ops_;

  // Latest enqueued motor position after the last motor segment sent out.
  int motor_position_[GCODE_NUM_AXES];

  // Next buffered positions. Written by incoming gcode, read by outgoing
  // motor movements.
  RingDeque<AxisTarget, 4> planning_buffer_;

  // Pre-calculated per axis limits in steps, steps/s, steps/s^2
  // All arrays are indexed by axis.
  AxesRegister max_axis_speed_;   // max travel speed hz
  AxesRegister max_axis_step_accel_;   // acceleration hz/s
  float highest_accel_;           // hightest accel of all axes.

  HardwareMapping::AuxBitmap last_aux_bits_;  // last enqueued aux bits.

  bool path_halted_;
  bool position_known_;
};

static inline int round2int(float x) { return (int) roundf(x); }

// Given that we want to travel "s" steps, start with speed "v0",
// accelerate peak speed v1 and slow down to "v2" with acceleration "a",
// what is v1 ?
// Note, it could be that we never reach v2
static float get_peak_speed(float s, float v0, float v2, float a) {
  return sqrtf(v2*v2 + v0*v0 + 2 * a * s) / sqrtf(2);
}

static float euclid_distance(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}

// Number of steps to accelerate or decelerate (negative "a") from speed
// v0 to speed v1. Modifies v1 if we can't reach the speed with the allocated
// number of steps.
static float steps_for_speed_change(float a, float v0, float *v1, int max_steps) {
  // s = v0 * t + a/2 * t^2
  // v1 = v0 + a*t
  const float t = (*v1 - v0) / a;
  // TODO:
  if (t < 0) Log_error("Error condition: t=%.1f INSUFFICIENT LOOKAHEAD\n", t);
  float steps = a/2 * t*t + v0 * t;
  if (steps <= max_steps) return steps;
  // Ok, we would need more steps than we have available. We correct the speed to what
  // we actually can reach.
  *v1 = sqrtf(v0*v0 + 2 * a * max_steps);
  return max_steps;
}

// Returns true, if all results in zero movement
static bool subtract_steps(struct LinearSegmentSteps *value,
                           const struct LinearSegmentSteps &subtract) {
  bool has_nonzero = false;
  for (int i = 0; i < BEAGLEG_NUM_MOTORS; ++i) {
    value->steps[i] -= subtract.steps[i];
    has_nonzero |= (value->steps[i] != 0);
  }
  return has_nonzero;
}

// Determine what speed we should have at the border between now and next
// segment.
static bool determine_joining_speed(const struct AxisTarget *from,
                                    const struct AxisTarget *to,
                                    const float threshold) {
  // the dot product of the vectors
  const float dot = from->dx*to->dx + from->dy*to->dy + from->dz*to->dz;
  const float mag = from->len * to->len;
  if (dot == 0) return 0.0f;     // orthogonal 90 degree, full stop
  if (dot == mag) return to->speed;  // codirectional 0 degree, keep accelerating

  // The angle between the vectors
  const float rad2deg = 180.0 / M_PI;
  const float angle = fabsf(acosf(dot / mag) * rad2deg);
  if (angle <= threshold)
    return to->speed;               // in tolerance, keep accelerating

  return 0.0f;
}

Planner::Impl::Impl(const MachineControlConfig *config,
                    HardwareMapping *hardware_mapping,
                    MotorOperations *motor_backend)
  : cfg_(config), hardware_mapping_(hardware_mapping),
    motor_ops_(motor_backend),
    highest_accel_(-1), path_halted_(true), position_known_(true) {
  // Initial machine position. We assume the homed position here, which is
  // wherever the endswitch is for each axis.
  struct AxisTarget *init_axis = planning_buffer_.append();
  bzero(init_axis, sizeof(*init_axis));
  // TODO(planner-hz): get homing position and assign to init_axis->target.
  // also, then convert them to the motor position.
  bzero(motor_position_, sizeof(motor_position_));

  position_known_ = true;

  float lowest_accel = cfg_->max_feedrate[AXIS_X] * cfg_->steps_per_mm[AXIS_X];
  for (const GCodeParserAxis i : AllAxes()) {
    max_axis_speed_[i] = cfg_->max_feedrate[i] * cfg_->steps_per_mm[i];
    const float accel = cfg_->acceleration[i] * cfg_->steps_per_mm[i];
    max_axis_step_accel_[i] = accel;
    if (accel > highest_accel_)
      highest_accel_ = accel;
    if (accel < lowest_accel)
      lowest_accel = accel;
  }
}

Planner::Impl::~Impl() {
  bring_path_to_halt();
}

// Assign steps to all the motors responsible for given axis.
void Planner::Impl::assign_steps_to_motors(struct LinearSegmentSteps *command,
                                           enum GCodeParserAxis axis,
                                           int steps) {
  hardware_mapping_->AssignMotorSteps(axis, steps, command);
}

// Move the given number of machine steps for each axis.
//
// This will be up to three segments: accelerating from last_pos speed to
// target speed, regular travel, and decelerating to the speed that the
// next segment is never forced to decelerate, but stays at speed or accelerate.
//
// The segments are sent to the motor operations backend.
//
// Since we calculate the deceleration, this modifies the speed of target_pos
// to reflect what the last speed was at the end of the move.
void Planner::Impl::move_machine_steps(const struct AxisTarget *last_pos,
                                       struct AxisTarget *target_pos,
                                       const struct AxisTarget *upcoming) {
  // TODO(planner-hz): this only works if there is any euclidian axis right now.
  int delta_steps[GCODE_NUM_AXES];
  int abs_defining_axis_steps = 0;
  enum GCodeParserAxis defining_axis = AXIS_X;
  for (GCodeParserAxis a : AllAxes()) {
    int new_machine_pos = target_pos->target[a] * cfg_->steps_per_mm[a];
    int delta = new_machine_pos - motor_position_[a];
    delta_steps[a] = delta;
    if (abs(delta) > abs_defining_axis_steps) {
      abs_defining_axis_steps = abs(delta);
      defining_axis = a;
    }
    motor_position_[a] = new_machine_pos;
  }

  if (abs_defining_axis_steps == 0) {
    if (last_aux_bits_ != target_pos->aux_bits) {
      // Special treatment: bits changed since last time, let's push them through.
      struct LinearSegmentSteps bit_set_command = {};
      bit_set_command.aux_bits = target_pos->aux_bits;
      motor_ops_->Enqueue(bit_set_command);
      last_aux_bits_ = target_pos->aux_bits;
    }
    return;
  }

  struct LinearSegmentSteps accel_command = {};
  struct LinearSegmentSteps move_command = {};
  struct LinearSegmentSteps decel_command = {};

  // Aux bits are set synchronously with what we need.
  move_command.aux_bits = target_pos->aux_bits;

  // Copy over common settings (currently: only aux_bits). Maybe simplify.
  memcpy(&accel_command, &move_command, sizeof(accel_command));
  memcpy(&decel_command, &move_command, sizeof(decel_command));

  // We need to arrive at a speed that the upcoming move does not have
  // to decelerate further (after all, it has a fixed feed-rate it should not
  // go over).
  const float last_end_speed = last_pos->speed;
  // End speed is either 0, when there is a corner, or the speed of the
  // upcoming segment.
  const float end_speed = determine_joining_speed(target_pos, upcoming,
                                                  cfg_->threshold_angle);
  const float a = cfg_->acceleration[defining_axis];  // TODO(planner-hz): scale
  const float peak_speed = get_peak_speed(target_pos->len,
                                          last_end_speed, end_speed, a);
  assert(peak_speed > 0);
  if (peak_speed < target_pos->speed) {
    // Didn't manage to accelerate to desired v. Let the next segment know
    // to take it from here.
    target_pos->speed = peak_speed;
  }

  // Determine accel fraction. Possibly modify our current target speed
  // if we can't reach the speed. In that case, leave the next segment to
  // go from here.
  const float accel_fraction =
    (last_end_speed < target_pos->speed)
    ? steps_for_speed_change(a, last_end_speed, &target_pos->speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  // We only decelerate if the upcoming speed is _slower_
  float dummy_next_speed = end_speed;  // Don't care to modify; we don't have
  const float decel_fraction =
    (end_speed < target_pos->speed)
    ? steps_for_speed_change(-a, target_pos->speed, &dummy_next_speed,
                             abs_defining_axis_steps) / abs_defining_axis_steps
    : 0;

  assert(accel_fraction + decel_fraction <= 1.0 + 1e-3);
  bool has_accel = false;
  bool has_move = false;
  bool has_decel = false;

  const float defining_axis_fraction =
    (target_pos->target[defining_axis] - last_pos->target[defining_axis]) / target_pos->len;
  const float to_defining_axis_speed = defining_axis_fraction * cfg_->steps_per_mm[defining_axis];

  // TODO(planner-hz): last defining axis fraction.
  if (accel_fraction * abs_defining_axis_steps > 0) {
    has_accel = true;
    accel_command.v0 = last_pos->speed * to_defining_axis_speed;
    accel_command.v1 = target_pos->speed * to_defining_axis_speed;

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis axis : AllAxes()) {
      const int accel_steps = round2int(accel_fraction * delta_steps[axis]);
      assign_steps_to_motors(&accel_command, axis, accel_steps);
    }
  } else {
    if (last_end_speed) target_pos->speed = last_end_speed; // No accel so use the last speed
  }

  move_command.v0 = target_pos->speed * to_defining_axis_speed;
  move_command.v1 = target_pos->speed * to_defining_axis_speed;

  if (decel_fraction * abs_defining_axis_steps > 0) {
    has_decel = true;
    decel_command.v0 = target_pos->speed * to_defining_axis_speed;
    decel_command.v1 = end_speed * to_defining_axis_speed;
    target_pos->speed = end_speed;

    // Now map axis steps to actual motor driver
    for (const GCodeParserAxis axis : AllAxes()) {
      const int decel_steps = round2int(decel_fraction * delta_steps[axis]);
      assign_steps_to_motors(&decel_command, axis, decel_steps);
    }
  }

  // Move is everything that hasn't been covered in speed changes.
  // So we start with all steps and subtract steps done in acceleration and
  // deceleration.
  for (const GCodeParserAxis a : AllAxes()) {
    assign_steps_to_motors(&move_command, a, delta_steps[a]);
  }
  subtract_steps(&move_command, accel_command);
  has_move = subtract_steps(&move_command, decel_command);

  if (cfg_->synchronous) motor_ops_->WaitQueueEmpty();

  if (has_accel) motor_ops_->Enqueue(accel_command);
  if (has_move) motor_ops_->Enqueue(move_command);
  if (has_decel) motor_ops_->Enqueue(decel_command);

  last_aux_bits_ = target_pos->aux_bits;
}

// If we have enough data in the queue, issue motor move.
void Planner::Impl::issue_motor_move_if_possible() {
  if (planning_buffer_.size() >= 3) {
    move_machine_steps(planning_buffer_[0],  // Current established position.
                       planning_buffer_[1],  // Position we want to move to.
                       planning_buffer_[2]); // Next upcoming.
    planning_buffer_.pop_front();
  }
}

void Planner::Impl::machine_move(const AxesRegister &target, float feedrate) {
  assert(position_known_);   // call SetExternalPosition() after DirectDrive()
  // We always have a previous position.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  new_pos->target = target;

  // Real world -> machine coordinates. Here, we are rounding to the next full
  // step, but we never accumulate the error, as we always use the absolute
  // position as reference.
  bool any_movement = false;
  for (const GCodeParserAxis a : AllAxes()) {
    const int steps = round2int((new_pos->target[a] - previous->target[a])
                                * cfg_->steps_per_mm[a]);
    if (steps != 0) {
      any_movement = true;   // At least one axis does more than zero steps
      break;
    }
  }

  if (!any_movement) {
    // Nothing to do, ignore this move. Since the next move will be an absolute
    // coordinate, we don't loose it.
    planning_buffer_.pop_back();
    return;
  }

  new_pos->aux_bits = hardware_mapping_->GetAuxBits();

  // Work out the real units values for the euclidian axes now to avoid
  // having to replicate the calcs later.
  new_pos->dx = new_pos->target[AXIS_X] - previous->target[AXIS_X];
  new_pos->dy = new_pos->target[AXIS_Y] - previous->target[AXIS_Y];
  new_pos->dz = new_pos->target[AXIS_Z] - previous->target[AXIS_Z];
  new_pos->len = euclid_distance(new_pos->dx, new_pos->dy, new_pos->dz);

  // Work out the desired euclidian travel speed in steps/s on the defining axis.
  new_pos->speed = feedrate;
  // TODO(planner-hz): limit feedrate here depending on the limits per axis. This
  // is essentially what clamp_to_limits() was.

  issue_motor_move_if_possible();
  path_halted_ = false;
}

void Planner::Impl::bring_path_to_halt() {
  if (path_halted_) return;
  // Enqueue a new position that is the same position as the last
  // one seen, but zero speed. That will allow the previous segment to
  // slow down. Enqueue.
  struct AxisTarget *previous = planning_buffer_.back();
  struct AxisTarget *new_pos = planning_buffer_.append();
  new_pos->target = previous->target;
  new_pos->speed = 0;
  new_pos->aux_bits = hardware_mapping_->GetAuxBits();
  new_pos->dx = new_pos->dy = new_pos->dz = new_pos->len = 0.0;
  issue_motor_move_if_possible();
  path_halted_ = true;
}

void Planner::Impl::GetCurrentPosition(AxesRegister *pos) {
  assert(planning_buffer_.size() > 0);  // we always should have a current pos
  *pos = planning_buffer_[0]->target;
}

int Planner::Impl::DirectDrive(GCodeParserAxis axis, float distance,
                               float v0, float v1) {
  bring_path_to_halt();     // Precondition. Let's just do it for good measure.
  position_known_ = false;

  const float steps_per_mm = cfg_->steps_per_mm[axis];

  struct LinearSegmentSteps move_command = {};

  move_command.v0 = v0 * steps_per_mm;
  if (move_command.v0 > max_axis_speed_[axis])
    move_command.v0 = max_axis_speed_[axis];
  move_command.v1 = v1 * steps_per_mm;
  if (move_command.v1 > max_axis_speed_[axis])
    move_command.v1 = max_axis_speed_[axis];

  move_command.aux_bits = hardware_mapping_->GetAuxBits();

  const int segment_move_steps = distance * steps_per_mm;
  assign_steps_to_motors(&move_command, axis, segment_move_steps);

  motor_ops_->Enqueue(move_command);
  motor_ops_->WaitQueueEmpty();

  return segment_move_steps;
}

void Planner::Impl::SetExternalPosition(GCodeParserAxis axis, float pos) {
  assert(path_halted_);   // Precondition.
  position_known_ = true;

  const int motor_axis_pos = pos * cfg_->steps_per_mm[axis];
  planning_buffer_.back()->target[axis] = pos;
  planning_buffer_[0]->target[axis] = pos;
  motor_position_[axis] = motor_axis_pos;
}

// -- public interface

Planner::Planner(const MachineControlConfig *config,
                 HardwareMapping *hardware_mapping,
                 MotorOperations *motor_backend)
  : impl_(new Impl(config, hardware_mapping, motor_backend)) {
}

Planner::~Planner() { delete impl_; }

void Planner::Enqueue(const AxesRegister &target_pos, float speed) {
  impl_->machine_move(target_pos, speed);
}

void Planner::BringPathToHalt() {
  impl_->bring_path_to_halt();
}

void Planner::GetCurrentPosition(AxesRegister *pos) {
  impl_->GetCurrentPosition(pos);
}

int Planner::DirectDrive(GCodeParserAxis axis, float distance,
                          float v0, float v1) {
  return impl_->DirectDrive(axis, distance, v0, v1);
}

void Planner::SetExternalPosition(GCodeParserAxis axis, float pos) {
  impl_->SetExternalPosition(axis, pos);
}
