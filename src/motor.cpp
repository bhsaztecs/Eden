#include "../include/declarations.h"
#include <cmath>
namespace BKND {
namespace motors {
float GetLoad(pass p_vals) { return NAN; }
void ClearMotorRotations(pass p_vals) {
  DBUG;
  cmpc(p_vals.leftmotor);
  cmpc(p_vals.rightmotor);
}
void Velocity(pass p_vals) {
  DBUG;
  while (true) {
    float leftposition1 = gmpc(p_vals.leftmotor);
    float rightposition1 = gmpc(p_vals.rightmotor);
    msleep(100);
    float leftposition2 = gmpc(p_vals.leftmotor);
    float rightposition2 = gmpc(p_vals.rightmotor);

    float leftvelocity = (leftposition2 - leftposition1) * 10; // tics per
                                                               // second
    float rightvelocity = (rightposition2 - rightposition1) * 10;
    p_vals.leftspeed = leftvelocity * 0.06; // percent speed
    p_vals.rightspeed = rightvelocity * 0.06;
  }
}
void Speed(float p_leftpercent, float p_rightpercent, float p_timeinseconds,
           pass p_vals) {

  float leftposition1 = BKND::UnitConvert(TTI, gmpc(p_vals.leftmotor));
  float rightposition1 = BKND::UnitConvert(TTI, gmpc(p_vals.rightmotor));
  motor(p_vals.leftmotor, (p_leftpercent * p_vals.lmm));
  motor(p_vals.rightmotor, (p_rightpercent * p_vals.rmm));
  msleep((p_timeinseconds * 1000) * p_vals.tmm);
  float leftposition2 = BKND::UnitConvert(TTI, gmpc(p_vals.leftmotor));
  float rightposition2 = BKND::UnitConvert(TTI, gmpc(p_vals.rightmotor));
  float leftdelta = leftposition2 - leftposition1;
  float rightdelta = rightposition2 - rightposition1;
  BKND::pathFind::Pathfind(leftdelta, rightdelta, p_vals);
}
void Distance(float p_leftinches, float p_rightinches, float p_timeinseconds,
              pass p_vals) {
  DBUG;
  float leftticksps = BKND::UnitConvert(ITT, p_leftinches) / p_timeinseconds;
  float leftspeed = BKND::UnitConvert(TPSTP, leftticksps);
  float rightticksps = BKND::UnitConvert(ITT, p_rightinches) / p_timeinseconds;
  float rightspeed = BKND::UnitConvert(TPSTP, rightticksps);
  Speed(leftspeed, rightspeed, p_timeinseconds, p_vals);
}
void Rotation(float p_leftdegrees, float p_rightdegrees, float p_timeinseconds,
              pass p_vals) {
  DBUG;
  float leftdistanec = BKND::UnitConvert(DTI, p_leftdegrees);
  float rightdistance = BKND::UnitConvert(DTI, p_rightdegrees);
  BKND::motors::Distance(leftdistanec, rightdistance, p_timeinseconds, p_vals);
}
void Accelerate(float p_leftmaxpercent, float p_rightmaxpercent,
                float p_timeinseconds, pass p_vals) {
  DBUG;
  float leftinitialspeed = p_vals.leftspeed;
  float leftdeltaspeed = p_leftmaxpercent - leftinitialspeed;

  float rightinitialspeed = p_vals.rightspeed;
  float rightdeltaspeed = p_rightmaxpercent - rightinitialspeed;

  for (float i = 0; i < 1; i += 0.01) {
    Speed((BKND::Interpolate(i) * leftdeltaspeed) + leftinitialspeed,
          (BKND::Interpolate(i) * rightdeltaspeed) + rightinitialspeed,
          p_timeinseconds / 100, p_vals);
  }
}
void Brake(pass p_vals) {
  DBUG;
  Speed(0, 0, 0.1, p_vals);
}
} // namespace motors
} // namespace BKND
