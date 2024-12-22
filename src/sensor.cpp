#include "../include/declarations.h"
#include <kipr/analog/analog.h>

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
}
} // namespace accel
namespace gyro {
BKND::P3D Value;
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
}
} // namespace gyro
namespace mag {
BKND::P3D Value;
void Calibrate() {
  // magneto_calibrate();
}
float Magnitude() {
  BKND::sensors::mag::Update();
  return Value.Magnitude();
}
float Pitch() {
  BKND::sensors::mag::Update();
  return Value.Pitch();
}
float Yaw() {
  BKND::sensors::mag::Update();
  return Value.Yaw();
}
void Update() {
  Value.m_X = magneto_x();
  Value.m_Y = magneto_y();
  Value.m_Z = magneto_z();
}
} // namespace mag
namespace battery {
int Power() { return power_level() * 100; }
bool Critical() { return Power() < 33; }
} // namespace battery
} // namespace sensors
} // namespace BKND
