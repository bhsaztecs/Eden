#include "../include/declarations.h"
#include <initializer_list>
#include <math.h>
namespace BKND {
namespace pathFind {
void Pathfind(float p_deltal, float p_deltar, pass p_vals) {
  DBUG;
  float theta = (p_deltar - p_deltal) / (2 * p_vals.wheelbase);
  float distance = (p_deltar + p_deltal) / 2;
  BKND::worldSpace temp = BKND::worldSpace(
      distance * cos(theta + Rad(G_Odometry.m_Orientation)),
      distance * sin(theta + Rad(G_Odometry.m_Orientation)), Deg(theta));
  G_Odometry.m_X += temp.m_X;
  G_Odometry.m_Y += temp.m_Y;
  G_Odometry.m_Orientation += temp.m_Orientation;
}
void Face(float p_deg, float p_time, pass p_vals) {
  DBUG;
  float wheelangle = (p_deg - BKND::G_Odometry.m_Orientation) *
                     (p_vals.wheelbase / p_vals.wheelradius);
  BKND::motors::Rotation(-wheelangle, wheelangle, p_time, p_vals);
}
void GoTo(BKND::P2D p_goal, float p_time, pass p_vals) {
  DBUG;
  worldSpace delta = (worldSpace(p_goal.m_X, p_goal.m_Y) - BKND::G_Odometry);
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
void FollowPath(pathfunc p_path, float p_time, pass p_vals) {
  float dt = 0.001;
  GoTo(p_path(0),
       PathLength(
           MakePath({BKND::P2D(G_Odometry.m_X, G_Odometry.m_Y), p_path(0)})),
       p_vals);
  GoTo(p_path(dt),
       PathLength(
           MakePath({BKND::P2D(G_Odometry.m_X, G_Odometry.m_Y), p_path(dt)})),
       p_vals);
  for (float t = 2 * dt; t < 1; t += dt) {
    GoTo(p_path(t), p_time * dt, p_vals);
  }
}
float PathLength(pathfunc p_path, float p_start, float p_end) {
  float length = 0;
  float dt = 0.001;
  for (float t = p_start; t < p_end; t += dt) {
    length += (p_path(t + dt) - p_path(t)).Magnitude() * dt;
  }
  return length;
}
pathfunc MakePath(std::initializer_list<BKND::P2D> p_points) {
  std::vector<BKND::P2D> points(p_points);
  return [points](float t) -> BKND::P2D {
    int n = points.size() - 1;
    BKND::P2D result(0, 0);
    for (int i = 0; i <= n; i++) {
      float coeff = 1.0;
      for (int j = 0; j < i; j++) {
        coeff *= (n - j);
      }
      for (int j = 0; j < i; j++) {
        coeff /= (i - j);
      }
      coeff *= pow(t, i) * pow(1 - t, n - i);
      result = result + BKND::P2D(points[i].m_X * coeff, points[i].m_Y * coeff);
    }
    return result;
  };
}
pathfunc MakePath(std::initializer_list<pathfunc> p_funcs) {
  std::vector<pathfunc> funcs(p_funcs);
  return [funcs](float t) -> BKND::P2D {
    // scale t down by funcs.len(), run func(t % funcs.len())
    return funcs[roundf(t * funcs.size())](
        fmod(t * funcs.size(), funcs.size()));
  };
}
pathfunc
MakePath(std::initializer_list<std::initializer_list<BKND::P2D>> p_points) {
  std::vector<std::initializer_list<BKND::P2D>> points(p_points);
  std::vector<pathfunc> funcs;
  for (auto pointset : points) {
    funcs.push_back(MakePath(pointset));
  }
  return [funcs](float t) -> BKND::P2D {
    // scale t down by funcs.len(), run func(t % funcs.len())
    return funcs[roundf(t * funcs.size())](
        fmod(t * funcs.size(), funcs.size()));
  };
}
} // namespace pathFind
} // namespace BKND
