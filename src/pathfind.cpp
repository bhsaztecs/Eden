#include "../include/declarations.h"
#include <initializer_list>
#include <limits>
#include <math.h>
namespace BKND {
namespace path {
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

  BKND::path::Face(delta.Angle(), ftime, p_vals);
  BKND::motors::Distance(delta.Magnitude(), delta.Magnitude(), dtime, p_vals);
}

void FollowPath(pathfunc p_path, float p_time, float p_start, float p_end,
                pass p_vals) {
  float delta = 0.1;

  for (int i = p_start; i < p_end; i += delta * 3) {
    P2D start = p_path(i);
    P2D middle = p_path(i + delta);
    P2D end = p_path(i + delta * 2);

    std::array<float, 3> circle = PointsToCircle({start, middle, end});

    std::array<float, 2> info = GetArcInfo(circle);
    FollowCircle(info[1], info[0], delta * p_time, p_vals);
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
pathfunc MakePath(initlist<BKND::P2D> p_points) {
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
pathfunc MakePath(initlist<pathfunc> p_funcs) {
  std::vector<pathfunc> funcs(p_funcs);
  return [funcs](float t) -> BKND::P2D {
    int index = 0;
    if (t == 1) {
      index = 1 - std::numeric_limits<float>::epsilon();
    } else {
      index = floor(t * funcs.size());
    }
    float value = 0;
    if (t == 1) {
      value = 1;
    } else {
      value = fmod(t * funcs.size(), 1);
    }
    return funcs[index](value);
  };
}
pathfunc MakePath(initlist<initlist<BKND::P2D>> p_points) {
  std::vector<initlist<BKND::P2D>> points(p_points);
  std::vector<pathfunc> funcs;
  for (auto pointset : points) {
    funcs.push_back(MakePath(pointset));
  }
  return [funcs](float t) -> BKND::P2D {
    int index = 0;
    if (t == 1) {
      index = 1 - std::numeric_limits<float>::epsilon();
    } else {
      index = floor(t * funcs.size());
    }
    float value = 0;
    if (t == 1) {
      value = 1;
    } else {
      value = fmod(t * funcs.size(), 1);
    }
    return funcs[index](value);
  };
}
std::array<float, 3> PointsToCircle(std::array<BKND::P2D, 3> p_points) {
  float x = ((p_points[1].m_Y - p_points[0].m_Y) *
                 (p_points[0].m_Y - p_points[2].m_Y) *
                 (p_points[2].m_Y - p_points[1].m_Y) -
             (p_points[2].m_X - p_points[0].m_X) *
                 (p_points[2].m_X + p_points[0].m_X) *
                 (p_points[1].m_Y - p_points[0].m_Y) +
             (p_points[0].m_X - p_points[1].m_X) *
                 (p_points[0].m_X + p_points[1].m_X) *
                 (p_points[0].m_Y - p_points[2].m_Y)) /
            (2 * ((p_points[0].m_X - p_points[1].m_X) *
                      (p_points[0].m_Y - p_points[2].m_Y) -
                  (p_points[2].m_X - p_points[0].m_X) *
                      (p_points[1].m_Y - p_points[0].m_Y)));
  float y = ((p_points[2].m_Y + p_points[0].m_Y) *
                 (p_points[0].m_Y - p_points[2].m_Y) *
                 (p_points[1].m_X - p_points[0].m_X) +
             (p_points[2].m_X - p_points[1].m_X) *
                 (p_points[0].m_X - p_points[2].m_X) *
                 (p_points[1].m_X - p_points[0].m_X) -
             (p_points[0].m_Y + p_points[1].m_Y) *
                 (p_points[1].m_Y - p_points[0].m_Y) *
                 (p_points[0].m_X - p_points[2].m_X)) /
            (2 * ((p_points[1].m_X - p_points[0].m_X) *
                      (p_points[0].m_Y - p_points[2].m_Y) -
                  (p_points[0].m_X - p_points[2].m_X) *
                      (p_points[1].m_Y - p_points[0].m_Y)));
  float r = sqrtf(powf(x - p_points[0].m_X, 2) + powf(y - p_points[0].m_Y, 2));
  std::array<float, 3> result = {x, y, r};
  return result;
}
std::array<float, 2> GetArcInfo(std::array<P2D, 3> p_points) {
  auto circle = PointsToCircle(p_points);
  P2D center(circle[0], circle[1]);
  float radius = circle[2];

  float angle0 = Deg(
      std::atan2(p_points[0].m_Y - center.m_Y, p_points[0].m_X - center.m_X));
  float angle1 = Deg(
      std::atan2(p_points[1].m_Y - center.m_Y, p_points[1].m_X - center.m_X));
  float angle2 = Deg(
      std::atan2(p_points[2].m_Y - center.m_Y, p_points[2].m_X - center.m_X));

  if (angle0 < 0)
    angle0 += 360;
  if (angle1 < 0)
    angle1 += 360;
  if (angle2 < 0)
    angle2 += 360;

  float arcAngle = 0;
  int direction = 0; // 1 for counterclockwise, -1 for clockwise

  bool isCCW = false;

  if ((angle0 <= angle1 && angle1 <= angle2) ||
      (angle2 <= angle0 && angle0 <= angle1) ||
      (angle1 <= angle2 && angle2 <= angle0)) {
    isCCW = true;
  }

  if (isCCW) {
    direction = 1; // counterclockwise

    if (angle2 >= angle0)
      arcAngle = angle2 - angle0;
    else
      arcAngle = (360 - angle0) + angle2;
  } else {
    direction = -1; // clockwise

    if (angle0 >= angle2)
      arcAngle = angle0 - angle2;
    else
      arcAngle = (360 - angle2) + angle0;
  }

  float arcLength = radius * Rad(arcAngle);

  return {arcLength, static_cast<float>(direction)};
}
void FollowCircle(float p_direction, float p_length, float p_time,
                  pass p_vals) {
  float l = p_length + (p_direction * p_vals.wheelbase);
  float r = p_length - (p_direction * p_vals.wheelbase);
  motors::Distance(l, r, p_time, p_vals);
}
} // namespace path
} // namespace BKND
