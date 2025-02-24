#include "../include/declarations.h"
#include <kipr/accel/accel.h>

namespace BKND {
namespace sensors {
namespace digital {
bool Value(int p_port) { return ::digital(p_port) != 0; }
} // namespace digital
namespace analog {
float Value(int p_port) { return (float)::analog(p_port) / 4095.0; }
int Raw(int p_port) { return ::analog(p_port); }
} // namespace analog
namespace accel {
BKND::P3D Value;
void DetectCollision() {
  float colisionthreshold = 0.1;
  auto one = G_Odometry;
  msleep(100);
  auto delta = G_Odometry - one;
  if (delta.Magnitude() - Raw().Magnitude() > colisionthreshold) {
    G_Colided = true;
  }
}
BKND::P3D Raw() {
  Update();
  return Value;
}
void Calibrate() {
  accel_calibrate();
  BKND::sensors::accel::Update();
}
float Magnitude() {
  BKND::sensors::accel::Update();
  return Value.Magnitude();
}
float Pitch() {
  BKND::sensors::accel::Update();
  return Value.Pitch();
}
float Yaw() {
  BKND::sensors::accel::Update();
  return Value.Yaw();
}
void Update() {
  Value.m_X = accel_x();
  Value.m_Y = accel_y();
  Value.m_Z = accel_z();
  Value = (Value / 1000) * 386.08858;
}
} // namespace accel
namespace gyro {
BKND::P3D Value;
BKND::P3D Raw() {
  Update();
  return Value;
}
void Calibrate() { gyro_calibrate(); }
float Magnitude() {
  BKND::sensors::gyro::Update();
  return Value.Magnitude();
}
float Pitch() {
  BKND::sensors::gyro::Update();
  return Value.Pitch();
}
float Yaw() {
  BKND::sensors::gyro::Update();
  return Value.Yaw();
}
void Update() {
  Value.m_X = gyro_x();
  Value.m_Y = gyro_y();
  Value.m_Z = gyro_z();
  Value *= -0.00142135186;
}
} // namespace gyro
namespace battery {
int Power() { return power_level() * 100; }
bool Critical() { return Power() < 33; }
} // namespace battery
} // namespace sensors
} // namespace BKND
