#include "../include/declarations.h"
namespace BKND {
namespace pathFind {
void Pathfind(float p_deltal, float p_deltar, pass p_vals) {
  if (BKND::MarginOfError(p_deltal, p_deltar, p_vals.margin)) {
    BKND::pathFind::LinearPathfind(p_deltal, p_deltar, p_vals);
    return;
  }
  if (BKND::MarginOfError(fabs(p_deltal), fabs(p_deltar), p_vals.margin)) {
    BKND::pathFind::AngularPathfind(p_deltal, p_deltar, p_vals);
    return;
  }
  BKND::pathFind::DynamicPathfind(p_deltal, p_deltar);
}
void AngularPathfind(float p_deltal, float p_deltar, pass p_vals) {
  float wheelangle = (fabs(p_deltal) + fabs(p_deltar)) / 2;
  float orientationoffset = wheelangle / (p_vals.turnrate * 1.32); // WHYYYYYYY

  BKND::G_Position.m_Orientation += BKND::Deg(orientationoffset);
}

void LinearPathfind(float p_deltal, float p_deltar, pass p_vals) {
  float distance = (p_deltal + p_deltar) / 2;
  BKND::P2D delta(distance * cos(BKND::Rad(BKND::G_Position.m_Orientation)),
                  distance * sin(BKND::Rad(BKND::G_Position.m_Orientation)));

  BKND::G_Position += delta;
}

void DynamicPathfind(float p_deltal, float p_deltar) {
  float curvature = (p_deltal + p_deltar) / (p_deltal - p_deltar);
  float distancetraveled = (p_deltal + p_deltar) / 2;
  float theta = distancetraveled / fabs(curvature);
  BKND::P2D changeinposition(fabs(curvature) * cos(theta),
                             fabs(curvature) * sin(theta));

  BKND::G_Position += changeinposition;
  BKND::G_Position.m_Orientation += BKND::Deg(theta);
}
void Face(float p_deg, float p_time, pass p_vals) {
  DBUG;
  float wheelangle =
      (p_deg - BKND::G_Position.m_Orientation) * (p_vals.turnrate);
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
  Face(delta.Angle(), ftime, p_vals);
  BKND::motors::Brake(p_vals);
  msleep(100);
  BKND::motors::Distance(delta.Magnitude(), delta.Magnitude(), dtime, p_vals);
}
} // namespace pathFind
} // namespace BKND
