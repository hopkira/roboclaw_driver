// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once
#include "RoboClaw.h"
#include "roboclaw_cmd.h"

class CmdSetSerialTimeout : public Cmd {
 public:
  CmdSetSerialTimeout(RoboClaw &roboclaw, uint8_t timeout_deciseconds)
      : Cmd(roboclaw, "SetSerialTimeout", RoboClaw::kNone), timeout_(timeout_deciseconds) {}

  void send() override {
    roboclaw_.appendToWriteLog("SetSerialTimeout: timeout (deciseconds): %u, WROTE: ", timeout_);
    try {
      roboclaw_.writeN2(3, roboclaw_.portAddress_, RoboClaw::SETSERIALTIMEOUT, timeout_);
      return;
    } catch (...) {
      RCUTILS_LOG_ERROR("[RoboClaw::CmdSetSerialTimeout] Uncaught exception !!!");
    }
  }

 private:
  long timeout_;
};
