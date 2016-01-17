/* -*- mode: c++; c-basic-offset: 2; indent-tabs-mode: nil; -*-
 * (c) 2013, 2014 Henner Zeller <h.zeller@acm.org>
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
#ifndef _BEAGLEG_GCODE_MACHINE_CONTROL_H_
#define _BEAGLEG_GCODE_MACHINE_CONTROL_H_

#include "gcode-parser.h"
#include "container.h"

#include <string>

class MotorOperations;
class HardwareMapping;
class ConfigParser;
typedef FixedArray<float, GCODE_NUM_AXES> FloatAxisConfig;

// (TODO: the enums need to go somewhere else)
enum AuxMap {
  AUX_NOT_MAPPED,
  AUX_MIST,
  AUX_FLOOD,
  AUX_VACUUM,
  AUX_SPINDLE_ON,
  AUX_SPINDLE_DIR,
  AUX_COOLER,
  AUX_CASE_LIGHTS,
  AUX_FAN,
};

enum {
  BEAGLEG_NUM_SWITCHES = 6,  // Number of supported input switches.
};

// Compact representation of an enstop configuration.
struct EndstopConfig {
  bool homing_use : 1;      // 0: no 1: yes.
  unsigned char endstop_switch : 6;  // 0: no mapping; or (1..NUM_ENDSTOPS+1)
};

/* Configuration constants for the controller.
 * Parameters in the arrays are always indexed by logical axes, e.g. AXIS_X.
 * The output mapping to the physical driver is controlled by output_mapping
 */
struct MachineControlConfig {
  MachineControlConfig();

  // Read values from configuration file.
  bool ConfigureFromFile(ConfigParser *parser);

  // Arrays with values for each axis
  FloatAxisConfig steps_per_mm;   // Steps per mm for each logical axis.
  FloatAxisConfig move_range_mm;  // Range of axes in mm (0..range[axis]). -1: no limit

  FloatAxisConfig max_feedrate;   // Max feedrate for axis (mm/s)
  FloatAxisConfig acceleration;   // Max acceleration for axis (mm/s^2)

  float speed_factor;         // Multiply feed with. Should be 1.0 by default.
  float threshold_angle;      // Threshold angle to ignore speed changes

  // The follwing parameter determines which logical axis ends up
  // on which physical plug location.
  //
  // The 'axis_mapping' determines how to map a logical axis (e.g. 'X') to
  // a connector position. The string position represents the
  // position on the board (sequence of connectors), while the character at
  // that position describes the logical axis. Typicaly, this is just
  // "XZYEABC"; for reasons such as using a double-connector, one might
  // have a different mapping, e.g. "XZE_Y". Underscores represent axis that
  // are not mapped.
  std::string axis_mapping;     // Mapping of axis-name (character in string)
                                // to physical location (position in string).
                                // Assumed "XYZEABC" if empty.
                                // Axis name '_' for skipped placeholder.
                                // Not mentioned axes are not handled.

  // Mapping of Axis to which endstop it affects.
  FixedArray<EndstopConfig, GCODE_NUM_AXES> min_endstop_;
  FixedArray<EndstopConfig, GCODE_NUM_AXES> max_endstop_;
  FixedArray<bool, BEAGLEG_NUM_SWITCHES> trigger_level_;

  std::string home_order;        // Order in which axes are homed.

  bool acknowledge_lines;       // Respond w/ 'ok' on each command on msg_stream.
  bool require_homing;          // Require homing before any moves.
  bool range_check;             // Do machine limit checks. Default 1.
  bool debug_print;             // Print step-tuples to output_fd if 1.
  bool synchronous;             // Don't queue, wait for command to finish if 1.
};

// A class that controls a machine via gcode.
class GCodeMachineControl {
 public:
  // Factor to create a GCodeMachineControl.
  // The MotorOperations provide the low-level motor control ops.
  // msg_stream, if non-NULL, sends back return messages on the GCode channel.
  // Returns NULL on failure.
  static GCodeMachineControl *Create(const MachineControlConfig &config,
                                     MotorOperations *motor_backend,
                                     HardwareMapping *hardware_mapping,
                                     FILE *msg_stream);

  ~GCodeMachineControl();

  // Set where messages should go.
  void SetMsgOut(FILE *msg_stream);

  // Get the physical home position of this machine which depend
  // on the position of the endstops configured for homing.
  // return in *pos register.
  void GetHomePos(AxesRegister *pos);

  // Return the receiver for parse events. The caller must not assume ownership
  // of the returned pointer.
  GCodeParser::EventReceiver *ParseEventReceiver();

 private:
  class Impl;

  // The MotorOperations struct provide the low-level motor control ops.
  // msg_stream, if non-NULL, sends back return messages on the GCode channel.
  GCodeMachineControl(Impl *Impl);

  Impl *const impl_;  // opaque state.
};

#endif //  _BEAGLEG_GCODE_MACHINE_CONTROL_H_
