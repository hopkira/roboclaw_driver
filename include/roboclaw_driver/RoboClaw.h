// SPDX-License-Identifier: Apache-2.0
// Copyright 2025 WimbleRobotics
// https://github.com/wimblerobotics/Sigyn

#pragma once

#include <rcutils/logging_macros.h>
#include <sys/types.h>

#include <chrono>
#include <cstdarg>  // Include cstdarg for va_list
#include <cstdint>
#include <rclcpp/logger.hpp>
#include <sstream>
#include <string>

#define SetDWORDval(arg) \
  (uint8_t)(arg >> 24), (uint8_t)(arg >> 16), (uint8_t)(arg >> 8), (uint8_t)arg

class RoboClaw {
 public:
  enum kMotor { kM1 = 0, kM2 = 1, kNone = 2 };
  // uint16_t crc;
  // uint32_t timeout;

  // Bit positions used to build alarms.
  enum {
    kM1_OVER_CURRENT = 0x01,        // Motor 1 current sense is too high.
    kM2_OVER_CURRENT = 0x02,        // Motor 2 current sense is too high.
    kM1_OVER_CURRENT_ALARM = 0x04,  // Motor 1 controller over current alarm.
    kM2_OVER_CURRENT_ALARM = 0x08,  // Motor 2 controller over current alarm.
  };

  // Temperatures.
  typedef enum { kTemperature1, kTemperature2 } TTemperature;

  // A convenience struction to pass around configuration information.
  typedef struct {
    double p;
    double i;
    double d;
    uint32_t qpps;
    double max_current;
  } TPIDQ;

  // For a custom exception message.
  struct TRoboClawException : public std::exception {
    std::string s;
    TRoboClawException(const char *format, ...) {
      char buffer[256];
      va_list args;
      va_start(args, format);
      vsnprintf(buffer, sizeof(buffer), format, args);
      va_end(args);
      s = std::string(buffer);
    }
    ~TRoboClawException() throw() {}
    const char *what() const throw() { return s.c_str(); }
  };

  // Holds RoboClaw encoder result.
  typedef struct {
    int32_t value;
    uint8_t status;
  } EncodeResult;

  // Convenience structure to  hold a pair of current values.
  typedef struct {
    float m1Current;
    float m2Current;
  } TMotorCurrents;

  enum {
    M1FORWARD = 0,
    M1BACKWARD = 1,
    SETMINMB = 2,
    SETMAXMB = 3,
    M2FORWARD = 4,
    M2BACKWARD = 5,
    M17BIT = 6,
    M27BIT = 7,
    MIXEDFORWARD = 8,
    MIXEDBACKWARD = 9,
    MIXEDRIGHT = 10,
    MIXEDLEFT = 11,
    MIXEDFB = 12,
    MIXEDLR = 13,
    SETSERIALTIMEOUT = 14,
    GETSERIALTIMEOUT = 15,
    GETM1ENC = 16,
    GETM2ENC = 17,
    GETM1SPEED = 18,
    GETM2SPEED = 19,
    RESETENC = 20,
    GETVERSION = 21,
    SETM1ENCCOUNT = 22,
    SETM2ENCCOUNT = 23,
    GETMBATT = 24,
    GETLBATT = 25,
    SETMINLB = 26,
    SETMAXLB = 27,
    SETM1PID = 28,
    SETM2PID = 29,
    GETM1ISPEED = 30,
    GETM2ISPEED = 31,
    M1DUTY = 32,
    M2DUTY = 33,
    MIXEDDUTY = 34,
    M1SPEED = 35,
    M2SPEED = 36,
    MIXEDSPEED = 37,
    M1SPEEDACCEL = 38,
    M2SPEEDACCEL = 39,
    MIXEDSPEEDACCEL = 40,
    M1SPEEDDIST = 41,
    M2SPEEDDIST = 42,
    MIXEDSPEEDDIST = 43,
    M1SPEEDACCELDIST = 44,
    M2SPEEDACCELDIST = 45,
    MIXEDSPEEDACCELDIST = 46,
    GETBUFFERS = 47,
    GETPWMS = 48,
    GETCURRENTS = 49,
    MIXEDSPEED2ACCEL = 50,
    MIXEDSPEED2ACCELDIST = 51,
    M1DUTYACCEL = 52,
    M2DUTYACCEL = 53,
    MIXEDDUTYACCEL = 54,
    READM1PID = 55,
    READM2PID = 56,
    SETMAINVOLTAGES = 57,
    SETLOGICVOLTAGES = 58,
    GETMINMAXMAINVOLTAGES = 59,
    GETMINMAXLOGICVOLTAGES = 60,
    SETM1POSPID = 61,
    SETM2POSPID = 62,
    READM1POSPID = 63,
    READM2POSPID = 64,
    M1SPEEDACCELDECCELPOS = 65,
    M2SPEEDACCELDECCELPOS = 66,
    MIXEDSPEEDACCELDECCELPOS = 67,
    SETM1DEFAULTACCEL = 68,
    SETM2DEFAULTACCEL = 69,
    SETPINFUNCTIONS = 74,
    GETPINFUNCTIONS = 75,
    SETDEADBAND = 76,
    GETDEADBAND = 77,
    GETENCODERS = 78,
    GETISPEEDS = 79,
    RESTOREDEFAULTS = 80,
    GETTEMP = 82,
    GETTEMP2 = 83,  // Only valid on some models
    GETERROR = 90,
    GETENCODERMODE = 91,
    SETM1ENCODERMODE = 92,
    SETM2ENCODERMODE = 93,
    WRITENVM = 94,
    READNVM = 95,  // Reloads values from Flash into Ram
    SETCONFIG = 98,
    GETCONFIG = 99,
    SETM1MAXCURRENT = 133,
    SETM2MAXCURRENT = 134,
    GETM1MAXCURRENT = 135,
    GETM2MAXCURRENT = 136,
    SETPWMMODE = 148,
    GETPWMMODE = 149,
    FLAGBOOTLOADER = 255
  };  // Only available via USB communications

  // RoboClaw error status bit definitions (complete specification)
  enum class RoboClawError : uint32_t {
    // Error flags (bits 0-15)
    ERROR_ESTOP = 0x00000001,      ///< Emergency stop triggered
    ERROR_TEMP = 0x00000002,       ///< Temperature fault
    ERROR_TEMP2 = 0x00000004,      ///< Secondary temperature fault
    ERROR_LBATHIGH = 0x00000010,   ///< Logic battery voltage too high
    ERROR_LBATLOW = 0x00000020,    ///< Logic battery voltage too low
    ERROR_FAULTM1 = 0x00000040,    ///< Motor 1 driver fault
    ERROR_FAULTM2 = 0x00000080,    ///< Motor 2 driver fault
    ERROR_SPEED1 = 0x00000100,     ///< Motor 1 speed error
    ERROR_SPEED2 = 0x00000200,     ///< Motor 2 speed error
    ERROR_POS1 = 0x00000400,       ///< Motor 1 position error
    ERROR_POS2 = 0x00000800,       ///< Motor 2 position error
    ERROR_CURRENTM1 = 0x00001000,  ///< Motor 1 current error
    ERROR_CURRENTM2 = 0x00002000,  ///< Motor 2 current error

    // Warning flags (bits 16-31)
    WARN_OVERCURRENTM1 = 0x00010000,  ///< Motor 1 overcurrent warning
    WARN_OVERCURRENTM2 = 0x00020000,  ///< Motor 2 overcurrent warning
    WARN_MBATHIGH = 0x00040000,       ///< Main battery voltage too high warning
    WARN_MBATLOW = 0x00080000,        ///< Main battery voltage too low warning
    WARN_TEMP = 0x00100000,           ///< Temperature warning
    WARN_TEMP2 = 0x00200000,          ///< Secondary temperature warning
    WARN_S4 = 0x00400000,             ///< S4 signal warning
    WARN_S5 = 0x00800000,             ///< S5 signal warning
    WARN_CAN = 0x10000000,            ///< CAN communication warning (MCP models only)
    WARN_BOOT = 0x20000000,           ///< Boot mode warning
    WARN_OVERREGENM1 = 0x40000000,    ///< Motor 1 over-regeneration warning
    WARN_OVERREGENM2 = 0x80000000     ///< Motor 2 over-regeneration warning
  };

  // public methods
  RoboClaw(float m1MaxCurrent, float m2MaxCurrent, std::string device_name, uint8_t device_port,
           uint32_t baud_rate, bool do_debug = false, bool do_low_level_debug = false);

  ~RoboClaw();

  void appendToReadLog(const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug_log_.appendToReadLog(format, args);
    va_end(args);
  }

  void appendToWriteLog(const char *format, ...) {
    va_list args;
    va_start(args, format);
    debug_log_.appendToWriteLog(format, args);
    va_end(args);
  }

  uint16_t get2ByteCommandResult2(uint8_t command);
  uint32_t getUlongCommandResult2(uint8_t command);
  uint32_t getULongCont2(uint16_t &crc);
  uint8_t getByteCommandResult2(uint8_t command);
  int32_t getVelocityResult(uint8_t command);

  // Open the RoboClaw USB port.
  void openPort();

  uint8_t readByteWithTimeout2();
  void restartPort();
  void stop();
  void updateCrc(uint16_t &crc, uint8_t data);
  void writeByte2(uint8_t byte);
  void writeN2(uint8_t cnt, ...);

  // bool ForwardM1(uint8_t address, uint8_t speed);
  // bool BackwardM1(uint8_t address, uint8_t speed);
  // bool SetMinVoltageMainBattery(uint8_t address, uint8_t voltage);
  // bool SetMaxVoltageMainBattery(uint8_t address, uint8_t voltage);
  // bool ForwardM2(uint8_t address, uint8_t speed);
  // bool BackwardM2(uint8_t address, uint8_t speed);
  // bool ForwardBackwardM1(uint8_t address, uint8_t speed);
  // bool ForwardBackwardM2(uint8_t address, uint8_t speed);
  // bool ForwardMixed(uint8_t address, uint8_t speed);
  // bool BackwardMixed(uint8_t address, uint8_t speed);
  // bool TurnRightMixed(uint8_t address, uint8_t speed);
  // bool TurnLeftMixed(uint8_t address, uint8_t speed);
  // bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
  // bool LeftRightMixed(uint8_t address, uint8_t speed);
  // uint32_t ReadEncM1(uint8_t address, uint8_t *status = NULL,
  //                    bool *valid = NULL);
  // uint32_t ReadEncM2(uint8_t address, uint8_t *status = NULL,
  //                    bool *valid = NULL);
  // bool SetEncM1(uint8_t address, int32_t val);
  // bool SetEncM2(uint8_t address, int32_t val);
  // uint32_t ReadSpeedM1(uint8_t address, uint8_t *status = NULL,
  //                      bool *valid = NULL);
  // uint32_t ReadSpeedM2(uint8_t address, uint8_t *status = NULL,
  //                      bool *valid = NULL);
  // bool ResetEncoders(uint8_t address);
  // bool ReadVersion(uint8_t address, char *version);
  // uint16_t ReadMainBatteryVoltage(uint8_t address, bool *valid = NULL);
  // uint16_t ReadLogicBatteryVoltage(uint8_t address, bool *valid = NULL);
  // bool SetMinVoltageLogicBattery(uint8_t address, uint8_t voltage);
  // bool SetMaxVoltageLogicBattery(uint8_t address, uint8_t voltage);
  // bool SetM1VelocityPID(uint8_t address, float Kp, float Ki, float Kd,
  //                       uint32_t qpps);
  // bool SetM2VelocityPID(uint8_t address, float Kp, float Ki, float Kd,
  //                       uint32_t qpps);
  // uint32_t ReadISpeedM1(uint8_t address, uint8_t *status = NULL,
  //                       bool *valid = NULL);
  // uint32_t ReadISpeedM2(uint8_t address, uint8_t *status = NULL,
  //                       bool *valid = NULL);
  // bool DutyM1(uint8_t address, uint16_t duty);
  // bool DutyM2(uint8_t address, uint16_t duty);
  // bool DutyM1M2(uint8_t address, uint16_t duty1, uint16_t duty2);
  // bool SpeedM1(uint8_t address, uint32_t speed);
  // bool SpeedM2(uint8_t address, uint32_t speed);
  // bool SpeedM1M2(uint8_t address, uint32_t speed1, uint32_t speed2);
  // bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
  // bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);
  // bool SpeedAccelM1M2(uint8_t address, uint32_t accel, uint32_t speed1,
  //                     uint32_t speed2);
  // bool SpeedDistanceM1(uint8_t address, uint32_t speed, uint32_t distance,
  //                      uint8_t flag = 0);
  // bool SpeedDistanceM2(uint8_t address, uint32_t speed, uint32_t distance,
  //                      uint8_t flag = 0);
  // bool SpeedDistanceM1M2(uint8_t address, uint32_t speed1, uint32_t
  // distance1,
  //                        uint32_t speed2, uint32_t distance2, uint8_t flag =
  //                        0);
  // bool SpeedAccelDistanceM1(uint8_t address, uint32_t accel, uint32_t speed,
  //                           uint32_t distance, uint8_t flag = 0);
  // bool SpeedAccelDistanceM2(uint8_t address, uint32_t accel, uint32_t speed,
  //                           uint32_t distance, uint8_t flag = 0);
  // bool SpeedAccelDistanceM1M2(uint8_t address, uint32_t accel, uint32_t
  // speed1,
  //                             uint32_t distance1, uint32_t speed2,
  //                             uint32_t distance2, uint8_t flag = 0);
  // bool ReadBuffers(uint8_t address, uint8_t &depth1, uint8_t &depth2);
  // bool ReadPWMs(uint8_t address, int16_t &pwm1, int16_t &pwm2);
  // bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);
  // bool SpeedAccelM1M2_2(uint8_t address, uint32_t accel1, uint32_t speed1,
  //                       uint32_t accel2, uint32_t speed2);
  // bool SpeedAccelDistanceM1M2_2(uint8_t address, uint32_t accel1,
  //                               uint32_t speed1, uint32_t distance1,
  //                               uint32_t accel2, uint32_t speed2,
  //                               uint32_t distance2, uint8_t flag = 0);
  // bool DutyAccelM1(uint8_t address, uint16_t duty, uint32_t accel);
  // bool DutyAccelM2(uint8_t address, uint16_t duty, uint32_t accel);
  // bool DutyAccelM1M2(uint8_t address, uint16_t duty1, uint32_t accel1,
  //                    uint16_t duty2, uint32_t accel2);
  // bool ReadM1VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp,
  //                        float &Kd_fp, uint32_t &qpps);
  // bool ReadM2VelocityPID(uint8_t address, float &Kp_fp, float &Ki_fp,
  //                        float &Kd_fp, uint32_t &qpps);
  // bool SetMainVoltages(uint8_t address, uint16_t min, uint16_t max);
  // bool SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max);
  // bool ReadMinMaxMainVoltages(uint8_t address, uint16_t &min, uint16_t &max);
  // bool ReadMinMaxLogicVoltages(uint8_t address, uint16_t &min, uint16_t
  // &max); bool SetM1PositionPID(uint8_t address, float kp, float ki, float kd,
  //                       uint32_t kiMax, uint32_t deadzone, uint32_t min,
  //                       uint32_t max);
  // bool SetM2PositionPID(uint8_t address, float kp, float ki, float kd,
  //                       uint32_t kiMax, uint32_t deadzone, uint32_t min,
  //                       uint32_t max);
  // bool ReadM1PositionPID(uint8_t address, float &Kp, float &Ki, float &Kd,
  //                        uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min,
  //                        uint32_t &Max);
  // bool ReadM2PositionPID(uint8_t address, float &Kp, float &Ki, float &Kd,
  //                        uint32_t &KiMax, uint32_t &DeadZone, uint32_t &Min,
  //                        uint32_t &Max);
  // bool SpeedAccelDeccelPositionM1(uint8_t address, uint32_t accel,
  //                                 uint32_t speed, uint32_t deccel,
  //                                 uint32_t position, uint8_t flag);
  // bool SpeedAccelDeccelPositionM2(uint8_t address, uint32_t accel,
  //                                 uint32_t speed, uint32_t deccel,
  //                                 uint32_t position, uint8_t flag);
  // bool SpeedAccelDeccelPositionM1M2(uint8_t address, uint32_t accel1,
  //                                   uint32_t speed1, uint32_t deccel1,
  //                                   uint32_t position1, uint32_t accel2,
  //                                   uint32_t speed2, uint32_t deccel2,
  //                                   uint32_t position2, uint8_t flag);
  // bool SetM1DefaultAccel(uint8_t address, uint32_t accel);
  // bool SetM2DefaultAccel(uint8_t address, uint32_t accel);
  // bool SetPinFunctions(uint8_t address, uint8_t S3mode, uint8_t S4mode,
  //                      uint8_t S5mode);
  // bool GetPinFunctions(uint8_t address, uint8_t &S3mode, uint8_t &S4mode,
  //                      uint8_t &S5mode);
  // bool SetDeadBand(uint8_t address, uint8_t Min, uint8_t Max);
  // bool GetDeadBand(uint8_t address, uint8_t &Min, uint8_t &Max);
  // bool ReadEncoders(uint8_t address, uint32_t &enc1, uint32_t &enc2);
  // bool ReadISpeeds(uint8_t address, uint32_t &ispeed1, uint32_t &ispeed2);
  // bool RestoreDefaults(uint8_t address);
  // bool ReadTemp(uint8_t address, uint16_t &temp);
  // bool ReadTemp2(uint8_t address, uint16_t &temp);
  // uint32_t ReadError(uint8_t address, bool *valid = NULL);
  // bool ReadEncoderModes(uint8_t address, uint8_t &M1mode, uint8_t &M2mode);
  // bool SetM1EncoderMode(uint8_t address, uint8_t mode);
  // bool SetM2EncoderMode(uint8_t address, uint8_t mode);
  // bool WriteNVM(uint8_t address);
  // bool ReadNVM(uint8_t address);
  // bool SetConfig(uint8_t address, uint16_t config);
  // bool GetConfig(uint8_t address, uint16_t &config);
  // bool SetM1MaxCurrent(uint8_t address, uint32_t max);
  // bool SetM2MaxCurrent(uint8_t address, uint32_t max);
  // bool ReadM1MaxCurrent(uint8_t address, uint32_t &max);
  // bool ReadM2MaxCurrent(uint8_t address, uint32_t &max);
  // bool SetPWMMode(uint8_t address, uint8_t mode);
  // bool GetPWMMode(uint8_t address, uint8_t &mode);

  // virtual int available();
  // void begin(long speed);
  // bool isListening();
  // bool overflow();
  // int peek();
  // virtual int read();
  // int read(uint32_t timeout);
  // bool listen();
  // virtual size_t write(uint8_t byte);
  // virtual void flush();
  // void clear();

 private:
  int baud_rate_;            // Baud rate for RoboClaw connection.
  std::string device_name_;  // Device name of RoboClaw device.
  int device_port_;          // Unix file descriptor for RoboClaw connection.
  bool do_debug_;            // True => print debug messages.
  bool do_low_level_debug_;  // True => print byte values as they are read and
                             // written.
  int maxCommandRetries_;    // Maximum number of times to retry a RoboClaw
  float maxM1Current_;       // Maximum allowed M1 current.
  float maxM2Current_;       // Maximum allowed M2 current.
  int motorAlarms_;          // Motors alarms. Bit-wise OR of contributors.
  int portAddress_;          // Port number of RoboClaw device under control command.

  // void crc_clear();
  // void crc_update(uint8_t data);
  // uint16_t crc_get();
  // bool write_n(uint8_t byte, ...);
  // bool read_n(uint8_t byte, uint8_t address, uint8_t cmd, ...);
  // uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool
  // *valid); uint32_t Read4(uint8_t address, uint8_t cmd, bool *valid);
  // uint16_t Read2(uint8_t address, uint8_t cmd, bool *valid);
  // uint8_t Read1(uint8_t address, uint8_t cmd, bool *valid);

  class DebugLog {
   public:
    DebugLog(RoboClaw *roboclaw)
        : roboclaw_(roboclaw), next_read_log_index_(0), next_write_log_index_(0) {}
    ~DebugLog() {}

    void appendToReadLog(const char *format, va_list args) {
      if (roboclaw_->do_debug_) {
        int written = vsnprintf(&read_log_[next_read_log_index_],
                                sizeof(read_log_) - next_read_log_index_, format, args);
        if (written > 0) {
          next_read_log_index_ += written;
        }
      }
    }

    void appendToWriteLog(const char *format, va_list args) {
      if (roboclaw_->do_debug_) {
        int written = vsnprintf(&write_log_[next_write_log_index_],
                                sizeof(write_log_) - next_write_log_index_, format, args);
        if (written > 0) {
          next_write_log_index_ += written;
        }
      }
    }

    void showLog() {
      if (roboclaw_->do_debug_) {
        RCUTILS_LOG_INFO("[RoboClaw::DebugLog] %s, READ: %s", write_log_, read_log_);
      }

      read_log_[0] = '\0';
      next_read_log_index_ = 0;
      write_log_[0] = '\0';
      next_write_log_index_ = 0;
    }

    // private:
    RoboClaw *roboclaw_;  // Pointer to the RoboClaw instance (if needed)
    char read_log_[256];
    char write_log_[256];
    uint16_t next_read_log_index_;
    uint16_t next_write_log_index_;
  };

  friend class Cmd;
  friend class CmdDoBufferedM1M2DriveSpeedAccelDistance;
  friend class CmdReadEncoderSpeed;
  friend class CmdReadEncoder;
  friend class CmdReadFirmwareVersion;
  friend class CmdReadLogicBatteryVoltage;
  friend class CmdReadMainBatteryVoltage;
  friend class CmdReadMotorCurrents;
  friend class CmdReadStatus;
  friend class CmdReadTemperature;
  friend class CmdSetEncoderValue;
  friend class CmdSetSerialTimeout;
  friend class CmdSetPid;

 public:  // ###
  DebugLog debug_log_;

 protected:
  static const char *motorNames_[];
};
