#include "../include/interface.h"
#include "../include/declarations.h"
Motors::~Motors() {
  Brake();
  off(m_Pass.leftmotor);
  off(m_Pass.rightmotor);
  m_Alive = false;
}
void Motors::NormalizeMultipliers() {
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
             p_wheelradius, p_wheelbase, BKND::worldSpace(), nullptr,
             m_LeftSpeed, m_RightSpeed),
      m_VelThread([this]() {
        while (m_Alive && BKND::G_ProgramRunning) {
          Velocity();
        }
      }) {
  NormalizeMultipliers();
  Clear();
}

Motors &Motors::operator=(const Motors &other) {
  if (this != &other) {
    m_Pass = other.m_Pass;
    m_LeftSpeed = other.m_LeftSpeed;
    m_RightSpeed = other.m_RightSpeed;
    m_Alive = other.m_Alive.load();
  }
  return *this;
}

Motors::Motors(const Motors &other)
    : m_Pass(other.m_Pass), m_VelThread([this]() {
        while (m_Alive && BKND::G_ProgramRunning) {
          Velocity();
        }
      }),
      m_LeftSpeed(other.m_LeftSpeed), m_RightSpeed(other.m_RightSpeed),
      m_Alive(other.m_Alive.load()) {}

void Motors::Clear() { BKND::motors::ClearMotorRotations(m_Pass); }
void Motors::Velocity() { BKND::motors::Velocity(m_Pass); }
void Motors::Speed(float p_leftgoalpercent, float p_rightgoalpercent,
                   float p_timeinseconds) {
  BKND::motors::Speed(p_leftgoalpercent, p_rightgoalpercent, p_timeinseconds,
                      m_Pass);
}
void Motors::Rotation(float p_leftgoaldegrees, float p_rightgoaldegrees,
                      float p_timeinseconds) {
  BKND::motors::Rotation(p_leftgoaldegrees, p_rightgoaldegrees, p_timeinseconds,
                         m_Pass);
}
void Motors::Distance(float p_leftgoalinches, float p_rightgoalinches,
                      float p_timeinseconds) {
  BKND::motors::Distance(p_leftgoalinches, p_rightgoalinches, p_timeinseconds,
                         m_Pass);
}
void Motors::Accelerate(float p_leftgoalpercent, float p_rightgoalpercent,
                        float p_timeinseconds) {
  BKND::motors::Accelerate(p_leftgoalpercent, p_rightgoalpercent,
                           p_timeinseconds, m_Pass);
}
void Motors::Brake() { BKND::motors::Brake(m_Pass); }

Servos::Servos(int p_port, BKND::P2D p_min, BKND::P2D p_max, bool p_ismotor) {
  m_Port = p_port;
  m_Slope = BKND::pointpair(p_min, p_max);
  m_IsMotor = p_ismotor;
  if (!m_IsMotor) {
    set_servo_enabled(m_Port, 1);
  }
}
void Servos::Set(float p_angle) {
  if (m_IsMotor) {
    Servos::MotorSet(m_Port, UnitConvert(m_Slope, p_angle));
  } else {
    BKND::servos::Set(m_Port, p_angle, m_Slope);
  }
}
void Servos::Change(float p_angle) {
  BKND::servos::Change(m_Port, p_angle, m_Slope);
}
void Servos::GoTo(float p_angle, float p_time) {
  BKND::servos::Move(m_Port, p_angle, p_time, m_Slope);
}
float Servos::Angle() {
  if (m_IsMotor) {
    return BKND::UnitConvert(Inverse(m_Slope), gmpc(m_Port));
  } else {
    return BKND::UnitConvert(Inverse(m_Slope), get_servo_position(m_Port));
  }
}
void Servos::MotorSet(int p_port, int p_ticks) {
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

Sensors<BKND::sensors::type::Analog>::Sensors(int p_port) : m_Port(p_port) {}
float Sensors<BKND::sensors::type::Analog>::Value() {
  return BKND::sensors::analog::Value(m_Port);
}

Sensors<BKND::sensors::type::Digital>::Sensors(int p_port) : m_Port(p_port) {}
bool Sensors<BKND::sensors::type::Digital>::Value() {
  return BKND::sensors::digital::Value(m_Port);
}

PathFind::PathFind(BKND::pass &motorstoread) : m_Motors(motorstoread) {}
void PathFind::GoTo(BKND::P2D p_goal, float p_time) {
  BKND::path::GoTo(p_goal, p_time, m_Motors);
}
void PathFind::Face(float p_goal, float p_time) {
  BKND::path::Face(p_goal, p_time, m_Motors);
}
void PathFind::FollowPath(BKND::path::pathfunc p_path, float p_time,
                          float p_start, float p_end) {
  BKND::path::FollowPath(p_path, p_time, p_start, p_end, m_Motors);
}
