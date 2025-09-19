// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once

#include "roboclaw_cmd.h"

class CmdSetEncoderValue : public Cmd {
 public:
  CmdSetEncoderValue(RoboClaw &roboclaw, RoboClaw::kMotor motor, long value)
      : Cmd(roboclaw, "SetEncoderValue", motor), value_(value) {}

  void send() override {
    roboclaw_.appendToWriteLog("SetEncoderValue: motor: %d (%s) value: %ld, WROTE: ", motor_,
                               RoboClaw::motorNames_[motor_], value_);
    try {
      roboclaw_.writeN2(6, roboclaw_.portAddress_,
                        motor_ == RoboClaw::kM1 ? RoboClaw::SETM1ENCCOUNT : RoboClaw::SETM2ENCCOUNT,
                        SetDWORDval(value_));
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdSetEncoderValue] Uncaught exception !!!");
    }
  }

 private:
  long value_;
};
