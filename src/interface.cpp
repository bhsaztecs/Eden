#include "../include/interface.h"
#include "../include/declarations.h"
Motors::~Motors() {
  Brake();
  off(m_Pass.leftmotor);
  off(m_Pass.rightmotor);
  m_Alive = false;
}
void Motors::NormalizeMultipliers(float p_leftmultiplier,
                                  float p_rightmultiplier) {
  DBUG;
  float maximizer = 1;
  if (m_Pass.lmm > m_Pass.rmm) {
    maximizer = 1 / m_Pass.lmm;
  } else {
    maximizer = 1 / m_Pass.rmm;
  }

  m_Pass.rmm = m_Pass.rmm * maximizer;
  m_Pass.lmm = m_Pass.lmm * maximizer;
  m_Pass.tmm = maximizer;
}

Motors::Motors(int p_leftport, int p_rightport, float p_leftmultiplier,
               float p_rightmultiplier, float p_wheelradius, float p_wheelbase)
    : m_Pass(p_leftport, p_rightport, p_leftmultiplier, p_rightmultiplier,
             p_wheelradius, p_wheelbase, m_LeftSpeed, m_RightSpeed),
      m_VelThread([this]() {
        while (m_Alive && BKND::G_ProgramRunning) {
          Velocity();
        }
      }) {
  DBUG;
  NormalizeMultipliers(p_leftmultiplier, p_rightmultiplier);
  Clear();
}
void Motors::Clear() {
  DBUG;
  BKND::motors::ClearMotorRotations(m_Pass);
}
void Motors::Velocity() {
  DBUG;
  BKND::motors::Velocity(m_Pass);
}
void Motors::Speed(float p_leftgoalpercent, float p_rightgoalpercent,
                   float p_timeinseconds) {
  DBUG;
  BKND::motors::Speed(p_leftgoalpercent, p_rightgoalpercent, p_timeinseconds,
                      m_Pass);
}
void Motors::Rotation(float p_leftgoaldegrees, float p_rightgoaldegrees,
                      float p_timeinseconds) {
  DBUG;
  BKND::motors::Rotation(p_leftgoaldegrees, p_rightgoaldegrees, p_timeinseconds,
                         m_Pass);
}
void Motors::Distance(float p_leftgoalinches, float p_rightgoalinches,
                      float p_timeinseconds) {
  DBUG;
  BKND::motors::Distance(p_leftgoalinches, p_rightgoalinches, p_timeinseconds,
                         m_Pass);
}
void Motors::Accelerate(float p_leftgoalpercent, float p_rightgoalpercent,
                        float p_timeinseconds) {
  DBUG;
  BKND::motors::Accelerate(p_leftgoalpercent, p_rightgoalpercent,
                           p_timeinseconds, m_Pass);
}
void Motors::Brake() {
  DBUG;
  BKND::motors::Brake(m_Pass);
}

Servos::Servos(int p_port, BKND::P2D p_min, BKND::P2D p_max, bool p_ismotor) {
  DBUG;
  m_Port = p_port;
  m_Slope = BKND::pointpair(p_min, p_max);
  m_IsMotor = p_ismotor;
  if (!m_IsMotor) {
    set_servo_enabled(m_Port, 1);
  }
}
void Servos::Set(float p_angle) {
  DBUG;
  if (m_IsMotor) {
    Servos::MotorSet(m_Port, UnitConvert(m_Slope, p_angle));
  } else {
    BKND::servos::Set(m_Port, p_angle, m_Slope);
  }
}
void Servos::Change(float p_angle) {
  DBUG;
  BKND::servos::Change(m_Port, p_angle, m_Slope);
}
void Servos::GoTo(float p_angle, float p_time) {
  DBUG;
  BKND::servos::Move(m_Port, p_angle, p_time, m_Slope);
}
float Servos::Angle() {
  DBUG;
  if (m_IsMotor) {
    return BKND::UnitConvert(Inverse(m_Slope), gmpc(m_Port));
  } else {
    return BKND::UnitConvert(Inverse(m_Slope), get_servo_position(m_Port));
  }
}
void Servos::MotorSet(int p_port, int p_ticks) {
  DBUG;
  int delta = p_ticks - gmpc(p_port);
  if (delta == 0) {
    return;
  }
  mtp(p_port, 1500, p_ticks);
  bmd(p_port);
  msleep(100);
  bmd(p_port);
  off(p_port);
  msleep(1000);
}

Sensors<BKND::sensors::type::Analog>::Sensors(int p_port) : m_Port(p_port) {
  DBUG;
}
float Sensors<BKND::sensors::type::Analog>::Value() {
  DBUG;
  return BKND::sensors::analog::Value(m_Port);
}

Sensors<BKND::sensors::type::Digital>::Sensors(int p_port) : m_Port(p_port) {
  DBUG;
}
bool Sensors<BKND::sensors::type::Digital>::Value() {
  DBUG;
  return BKND::sensors::digital::Value(m_Port);
}

PathFind::PathFind(BKND::pass &motorstoread) : m_Read(motorstoread) { DBUG; }
void PathFind::GoTo(BKND::P2D p_goal, float p_time) {
  DBUG;
  BKND::path::GoTo(p_goal, p_time, m_Read);
}
void PathFind::Face(float p_goal, float p_time) {
  DBUG;
  BKND::path::Face(p_goal, p_time, m_Read);
}
void PathFind::FollowPath(BKND::path::pathfunc p_path, float p_time,
                          float p_start, float p_end) {
  DBUG;
  BKND::path::FollowPath(p_path, p_time, p_start, p_end, m_Read);
}
