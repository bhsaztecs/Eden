#include "../include/interface.h"
#include "../include/declarations.h"
#include <kipr/servo/servo.h>

void Motors::NormalizeMultipliers(float p_leftmultiplier,
                                  float p_rightmultiplier) {
  DBUG;
  float maximizer = 1;
  if (m_pass.lmm > m_pass.rmm) {
    maximizer = 1 / m_pass.lmm;
  } else {
    maximizer = 1 / m_pass.rmm;
  }

  m_pass.rmm = m_pass.rmm * maximizer;
  m_pass.lmm = m_pass.lmm * maximizer;
  m_pass.tmm = maximizer;
}

Motors::Motors(int p_leftport, int p_rightport, float p_leftmultiplier,
               float p_rightmultiplier, float p_wheelradius, float p_wheelbase)
    : m_pass(p_leftport, p_rightport, p_leftmultiplier, p_rightmultiplier, 1.0F,
             p_wheelradius, p_wheelbase, m_LeftSpeed, m_RightSpeed) {
  DBUG;
  NormalizeMultipliers(p_leftmultiplier, p_rightmultiplier);
  Clear();
}
void Motors::Clear() const {
  DBUG;
  BKND::motors::ClearMotorRotations(m_pass);
}
void Motors::Velocity() const {
  DBUG;
  BKND::motors::Velocity(m_pass);
}
void Motors::Speed(float p_leftgoalpercent, float p_rightgoalpercent,
                   float p_timeinseconds) const {
  DBUG;
  BKND::motors::Speed(p_leftgoalpercent, p_rightgoalpercent, p_timeinseconds,
                      m_pass);
}
void Motors::Rotation(float p_leftgoaldegrees, float p_rightgoaldegrees,
                      float p_timeinseconds) const {
  DBUG;
  BKND::motors::Rotation(p_leftgoaldegrees, p_rightgoaldegrees, p_timeinseconds,
                         m_pass);
}
void Motors::Distance(float p_leftgoalinches, float p_rightgoalinches,
                      float p_timeinseconds) const {
  DBUG;
  BKND::motors::Distance(p_leftgoalinches, p_rightgoalinches, p_timeinseconds,
                         m_pass);
}
void Motors::Accelerate(float p_leftgoalpercent, float p_rightgoalpercent,
                        float p_timeinseconds) const {
  DBUG;
  BKND::motors::Accelerate(p_leftgoalpercent, p_rightgoalpercent,
                           p_timeinseconds, m_pass);
}
void Motors::Brake() const {
  DBUG;
  BKND::motors::Brake(m_pass);
}

Servos::Servos(int p_port, BKND::P2D p_min, BKND::P2D p_max) {
  DBUG;
  m_Port = p_port;
  m_Slope = BKND::pointpair(p_min, p_max);
  set_servo_enabled(m_Port, 1);
}
void Servos::Set(float p_angle) const {
  DBUG;
  BKND::servos::Set(m_Port, p_angle, m_Slope);
}
void Servos::Change(float p_angle) const {
  DBUG;
  BKND::servos::Change(m_Port, p_angle, m_Slope);
}
void Servos::GoTo(float p_angle, float p_time) const {
  DBUG;
  BKND::servos::Move(m_Port, p_angle, p_time, m_Slope);
}
float Servos::Angle() const {
  DBUG;
  return BKND::lerp(m_Slope, get_servo_position(m_Port));
}

Sensors<BKND::sensors::type::Analog>::Sensors(int p_port) : m_Port(p_port) {
  DBUG;
}
float Sensors<BKND::sensors::type::Analog>::Value() const {
  DBUG;
  return BKND::sensors::analog::Value(m_Port);
}

Sensors<BKND::sensors::type::Digital>::Sensors(int p_port) : m_Port(p_port) {
  DBUG;
}
bool Sensors<BKND::sensors::type::Digital>::Value() const {
  DBUG;
  return BKND::sensors::digital::Value(m_Port);
}

PathFind::PathFind(BKND::pass &motorstoread) : m_Read(motorstoread) { DBUG; }
void PathFind::GoTo(BKND::P2D p_goal, float p_time) const {
  DBUG;
  BKND::pathFind::GoTo(p_goal, p_time, m_Read);
}
void PathFind::Face(float p_goal, float p_time) const {
  DBUG;
  BKND::pathFind::Face(p_goal, p_time, m_Read);
}
