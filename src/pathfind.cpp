#include "../include/declarations.h"
namespace BKND {
namespace pathFind {
void Pathfind(float p_deltal, float p_deltar, pass p_vals) {
  DBUG;
  float theta = (p_deltar - p_deltal) / (2 * p_vals.wheelbase);
  float distance = (p_deltar + p_deltal) / 2;
  BKND::worldSpace temp = BKND::worldSpace(
      distance * cos(theta + Rad(G_Position.m_Orientation)),
      distance * sin(theta + Rad(G_Position.m_Orientation)), Deg(theta));
  G_Position.m_X += temp.m_X;
  G_Position.m_Y += temp.m_Y;
  G_Position.m_Orientation += temp.m_Orientation;
}
void Face(float p_deg, float p_time, pass p_vals) {
  DBUG;
  float wheelangle = (p_deg - BKND::G_Position.m_Orientation) *
                     (p_vals.wheelbase / p_vals.wheelradius);
  BKND::motors::Rotation(-wheelangle, wheelangle, p_time, p_vals);
}
void GoTo(BKND::P2D p_goal, float p_time, pass p_vals) {
  DBUG;
  worldSpace delta = (worldSpace(p_goal.m_X, p_goal.m_Y) - BKND::G_Position);
  float bias = 10;
  float ftime = (fabs(delta.Angle() / bias) /
                 (fabs(delta.Angle() / bias) + delta.Magnitude())) *
                p_time;
  float dtime =
      (delta.Magnitude() / (fabs(delta.Angle() / bias) + delta.Magnitude())) *
      p_time;

  BKND::pathFind::Face(delta.Angle(), ftime, p_vals);
  BKND::motors::Distance(delta.Magnitude(), delta.Magnitude(), dtime, p_vals);
}
} // namespace pathFind
} // namespace BKND
