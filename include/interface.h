#pragma once
#include "declarations.h"
class Motors {
  void Velocity() const;
  void NormalizeMultipliers(float p_leftmultiplier, float p_rightmultiplier);

public:
  BKND::pass m_pass;
  float m_LeftSpeed, m_RightSpeed;

  Motors(int p_leftport, int p_rightport, float p_leftmultiplier,
         float p_rightmultiplier, float p_wheelradius, float p_wheelbase);
  void Clear() const;
  void Speed(float p_leftgoalpercent, float p_rightgoalpercent,
             float p_timeinseconds) const;
  void Rotation(float p_leftgoaldegrees, float p_rightgoaldegrees,
                float p_timeinseconds) const;
  void Distance(float p_leftgoalinches, float p_rightgoalinches,
                float p_timeinseconds) const;
  void Accelerate(float p_leftgoalpercent, float p_rightgoalpercent,
                  float p_timeinseconds) const;
  void Brake() const;
};
class Servos {
public:
  int m_Port;
  BKND::pointpair m_Slope;
  Servos(int p_port, BKND::P2D p_min, BKND::P2D p_max);
  void Set(float p_angle) const;
  void Change(float p_angle) const;
  void GoTo(float p_angle, float p_time) const;
  float Angle() const;
};

template <BKND::sensors::type> class Sensors {
public:
  int m_Port;

  Sensors(int p_port);
};

template <> class Sensors<BKND::sensors::type::Analog> {
public:
  int m_Port;

  Sensors(int p_port);
  float Value() const;
};
template <> class Sensors<BKND::sensors::type::Digital> {
public:
  int m_Port;

  Sensors(int p_port);
  bool Value() const;
};

class PathFind {
public:
  BKND::pass m_Read;
  PathFind(BKND::pass &p_motorstoread);
  void GoTo(BKND::P2D p_goal, float p_time) const;
  void Face(float p_goal, float p_time) const;
};
