#pragma once
#include "declarations.h"
class Motors {
  void Velocity(); // update velocity reading of these motors every 10 ms.
                   // DOES NOT END
  void
  NormalizeMultipliers(float p_leftmultiplier,
                       float p_rightmultiplier); // maxes out motor multipliers
  /* IN: left motor multiplier, right motor multiplier
   * EG: NormalizeMultipliers(0.1,0.2) sets lmm & rmm to 0.5 & 1 and tmm to 2.
   * maintains distances regardless of multipliers. */

public:
  BKND::pass m_Pass;               // bunch of data that functions need
  float m_LeftSpeed, m_RightSpeed; // the speed that the left and right motors
                                   // are going. read, dont write.

  ~Motors();
  Motors(int p_leftport, int p_rightport, float p_leftmultiplier,
         float p_rightmultiplier, float p_wheelradius, float p_wheelbase);
  void Clear(); // clear motor position counter.
  void
  Speed(float p_leftgoalpercent, float p_rightgoalpercent,
        float p_timeinseconds); // drive at left speed, right speed for seconds
  void Rotation(float p_leftgoaldegrees, float p_rightgoaldegrees,
                float p_timeinseconds); // rotate the wheels this many more
                                        // degrees in this many seconds
  void Distance(float p_leftgoalinches, float p_rightgoalinches,
                float p_timeinseconds); // drive this many more inches in this
                                        // many seconds
  void Accelerate(float p_leftgoalpercent, float p_rightgoalpercent,
                  float p_timeinseconds); // accelerate from current speed to
                                          // goal speed in this many seconds
  void Brake();
};
class Servos {
  static void MotorSet(int p_port,
                       int ticks); // set, but only if using a motor as a servo
  bool m_IsMotor;

public:
  int m_Port;
  BKND::pointpair m_Slope; // unit conversion
  Servos(int p_port, BKND::P2D p_min, BKND::P2D p_max, bool p_ismotor = false);
  void Set(float p_angle);                // go to angle
  void Change(float p_angle);             // add p_angle degrees to angle
  void GoTo(float p_angle, float p_time); // go to angle slowly
  float Angle();                          // get angle
};

template <BKND::sensors::type> class Sensors {
public:
  int m_Port;

  Sensors(int p_port); // Sensors<sensors::type::__> name(port);
};

template <> class Sensors<BKND::sensors::type::Analog> {
public:
  int m_Port;

  Sensors(int p_port);
  float Value(); // get value of sensor from 0-1
};
template <> class Sensors<BKND::sensors::type::Digital> {
public:
  int m_Port;

  Sensors(int p_port);
  bool Value(); // get value of sensor 0/1
};

class PathFind {
public:
  BKND::pass m_Read;
  PathFind(BKND::pass &p_motorstoread);      // pass m_Pass from Motors
  void GoTo(BKND::P2D p_goal, float p_time); // go to point in p_time
  void Face(float p_goal,
            float p_time); // face angle (in degrees) in p_time
  void FollowPath(BKND::pathFind::pathfunc p_path, float p_time,
                  float p_start = 0, float p_end = 1);
};
template <typename DATA> class Connection {
  std::string Serialize(DATA p_data) {
    DBUG;
    return BKND::IRoC::Serialize<DATA>(p_data);
  }
  DATA Deserialize(std::string p_serialdata) {
    DBUG;
    return BKND::IRoC::Deserialize<DATA>(p_serialdata);
  }

public:
  std::string m_TargetIP;
  int m_Socket;
  bool m_IsHost;
  Connection(std::string p_targetip, bool p_ishost) {
    DBUG;
    m_TargetIP = p_targetip;
    m_IsHost = p_ishost;
  }
  int Connect() {
    DBUG;
    return BKND::IRoC::Connect(m_IsHost, m_TargetIP);
  }
  void Send(DATA p_data) {
    DBUG;
    return BKND::IRoC::Send(m_Socket, Serialize(p_data));
  }
  DATA Recieve() {
    DBUG;
    return Deserialize(BKND::IRoC::Recieve(m_Socket, sizeof(DATA)));
  }
};
