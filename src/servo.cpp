#include "../include/declarations.h"
namespace BKND {
namespace servos {
void Set(int p_port, float p_angle, pointpair p_slope) {
  DBUG;
  set_servo_position(p_port, BKND::lerp(p_slope, p_angle));
}
void Change(int p_port, float p_changeinangle, pointpair p_slope) {
  DBUG;

  int currentangle = BKND::lerp(p_slope, get_servo_position(p_port));
  int newangle = currentangle + p_changeinangle;

  Set(p_port, newangle, p_slope);
}
void Move(int p_port, float p_angle, float p_timeinseconds, pointpair p_slope) {
  DBUG;

  float start = BKND::lerp(p_slope, get_servo_position(p_port));
  float delta = (p_angle - start);
  float delay = 1000 * (p_timeinseconds / 100);
  if (-5 <= delta && delta <= 5) {
    Set(p_port, p_angle, p_slope);
    msleep(p_timeinseconds * 1000);
    return;
  }
  if (delta > 0) {
    for (float i = start; i <= p_angle; i += delta / 100) {
      Set(p_port, i, p_slope);
      msleep(delay);
    }
  } else {
    for (float i = start; i >= p_angle; i += delta / 100) {
      Set(p_port, i, p_slope);
      msleep(delay);
    }
  }
  Set(p_port, p_angle, p_slope);
}
} // namespace servos
} // namespace BKND
