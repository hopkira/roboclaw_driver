#pragma once

#include "roboclaw_cmd.h"

class CmdReadBufferLength : public Cmd {
 public:
  CmdReadBufferLength(RoboClaw &roboclaw, uint8_t &m1_buffer_length,
                      uint8_t &m2_buffer_length)
      : Cmd(roboclaw, "ReadBufferLength", RoboClaw::kNone),
        m1_buffer_length_(m1_buffer_length),
        m2_buffer_length_(m2_buffer_length) {}
  void send() override {
    try {
      roboclaw_.appendToWriteLog("ReadBufferLength: WROTE: ");
      uint16_t crc = 0;
      roboclaw_.updateCrc(crc, roboclaw_.portAddress_);
      roboclaw_.updateCrc(crc, RoboClaw::GETBUFFERS);
      roboclaw_.writeByte2(roboclaw_.portAddress_);
      roboclaw_.writeByte2(RoboClaw::GETBUFFERS);
      m1_buffer_length_ = 0;
      m2_buffer_length_ = 0;
      uint8_t datum = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, datum);
      m1_buffer_length_ = datum;

      datum = roboclaw_.readByteWithTimeout2();
      roboclaw_.updateCrc(crc, datum);
      m2_buffer_length_ = datum;

      uint16_t responseCrc = 0;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc = datum << 8;
      datum = roboclaw_.readByteWithTimeout2();
      responseCrc |= datum;
      if (responseCrc != crc) {
        RCUTILS_LOG_ERROR(
            "[RoboClaw::CmdReadBufferLength] invalid CRC expected: 0x%2X, "
            "got: 0x%2X",
            crc, responseCrc);
        m1_buffer_length_ = 0;
        m2_buffer_length_ = 0;
      }
    } catch (...) {
      RCUTILS_LOG_ERROR(
          "[RoboClaw::CmdReadBufferLength] Uncaught exception !!!");
    }

    // roboclaw_.appendToReadLog(", RESULT: %u, %u", m1_buffer_length_,
    //                           m2_buffer_length_);
  }

 private:
  uint8_t &m1_buffer_length_;
  uint8_t &m2_buffer_length_;
};
