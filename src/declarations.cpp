#include "../include/declarations.h"
namespace BKND {
Thread TIMER([]() { misc::Timer(); });
pointpair TTD(P2D(0, 0), P2D(1900, 360));
pointpair DTT(Inverse(TTD));

pointpair ITD(P2D(0, 0), P2D(1, 41.379));
pointpair DTI(Inverse(ITD));

pointpair TTI(P2D(0, 0), P2D(436.782, 2));
pointpair ITT(Inverse(TTI));

pointpair PTTPS(P2D(0, 0), P2D(100, 1386));
pointpair TPSTP(Inverse(PTTPS));

std::atomic<bool> G_ProgramRunning = {true};
float G_ColisionLimit = .1;
std::atomic<bool> G_Colided = {false};
void (*G_CollisionHandler)(pass) = nullptr;
long int G_CurrentMS = 0;
std::ofstream G_File("data/log.txt");
std::vector<worldSpace *> G_Obstacles;
worldSpace G_Odometry(0, 0, 0, 0);
IMU G_IMU(0, 0, 0, 0, 0);
void HandleColision(pass p_vals) {
  G_Colided = true;
  if (G_CollisionHandler != nullptr) {
    G_CollisionHandler(p_vals);
  } else {
    ao();
  }
}
std::string PrettyTime(int p_ms) {
  int min;
  int sec;
  int totalms = p_ms;
  min = totalms / (60 * 1000);
  totalms %= 60 * 1000;
  sec = totalms / 1000;
  totalms %= 1000;

  // Convert totalms to float and then to string to handle decimals
  std::string msStr = std::to_string(totalms / 1000.0F);

  // Remove trailing zeros
  size_t found = msStr.find_last_not_of('0');
  if ((found != std::string::npos) && (found != msStr.size() - 1)) {
    msStr = msStr.substr(0, found + 1);
  }
  msStr.erase(0, 1);
  msStr += "0";
  return std::to_string(min) + ":" + std::to_string(sec) + "." +
         msStr.substr(1);
}

float Rad(float p_degrees) { return p_degrees * 0.01745329251; }
float Deg(float p_radians) { return p_radians * 57.2957795131; }

float UnitConvert(pointpair p_slope, float p_x) {
  float dely = p_slope.second.m_Y - p_slope.first.m_Y;
  float delx = p_slope.second.m_X - p_slope.first.m_X;
  float m = dely / delx;
  float b = -((m * p_slope.second.m_X) - p_slope.second.m_Y);
  return (m * p_x) + b;
}

float NormalizeAngle(float p_angle) {
  float angle = fmod(p_angle, 360.0);
  if (angle < -180) {
    angle += 360.0;
  } else if (angle >= 180) {
    angle -= 360;
  }
  return angle;
}
P2D::P2D(float p_x, float p_y) {
  this->m_X = p_x;
  this->m_Y = p_y;
}
float P2D::Magnitude() {
  return std::sqrt(std::pow(m_X, 2) + std::pow(m_Y, 2));
}
float P2D::Angle() { return Deg(std::atan2(m_Y, m_X)); }
bool P2D::operator==(const P2D &p_other) const {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y);
}
bool P2D::operator!=(const P2D &p_other) const { return !(*this == p_other); }
P2D P2D::operator+(const P2D &p_other) const {
  return P2D(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y);
}
P2D P2D::operator-(const P2D &p_other) const {
  return P2D(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y);
}
P2D P2D::operator*(const float scalar) const {
  return P2D(this->m_X * scalar, this->m_Y * scalar);
}
P2D P2D::operator/(const float scalar) const {
  return P2D(this->m_X / scalar, this->m_Y / scalar);
}
void P2D::operator=(const P2D &p_other) {
  this->m_X = p_other.m_X;
  this->m_Y = p_other.m_Y;
}
void P2D::operator+=(const P2D &p_other) {
  P2D temp(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
}
void P2D::operator-=(const P2D &p_other) {
  P2D temp(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
}
void P2D::operator*=(const float scalar) {
  this->m_X *= scalar;
  this->m_Y *= scalar;
}
void P2D::operator/=(const float scalar) {
  this->m_X /= scalar;
  this->m_Y /= scalar;
}

P3D::P3D(float p_x, float p_y, float p_z) {
  this->m_X = p_x;
  this->m_Y = p_y;
  this->m_Z = p_z;
}
float P3D::Magnitude() {
  return std::sqrt(std::pow(m_X, 2) + std::pow(m_Y, 2) + std::pow(m_Z, 2));
}
float P3D::Pitch() { return Deg(std::atan(m_Y / m_Z)); }
float P3D::Yaw() { return Deg(std::atan(m_X / m_Z)); }
bool P3D::operator==(const P3D &p_other) const {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y) &&
         (this->m_Z == p_other.m_Z);
}
bool P3D::operator!=(const P3D &p_other) const { return !(*this == p_other); }
P3D P3D::operator+(const P3D &p_other) const {
  return P3D(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
             this->m_Z + p_other.m_Z);
}
P3D P3D::operator-(const P3D &p_other) const {
  return P3D(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
             this->m_Z - p_other.m_Z);
}
P3D P3D::operator*(const float scalar) const {
  return P3D(this->m_X * scalar, this->m_Y * scalar, this->m_Z * scalar);
}
P3D P3D::operator/(const float scalar) const {
  return P3D(this->m_X / scalar, this->m_Y / scalar, this->m_Z / scalar);
}
void P3D::operator=(const P3D &p_other) {
  this->m_X = p_other.m_X;
  this->m_Y = p_other.m_Y;
  this->m_Z = p_other.m_Z;
}
void P3D::operator+=(const P3D &p_other) {
  P3D temp(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
           this->m_Z + p_other.m_Z);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
  this->m_Z = temp.m_Z;
}
void P3D::operator-=(const P3D &p_other) {
  P3D temp(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
           this->m_Z - p_other.m_Z);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
  this->m_Z = temp.m_Z;
}
void P3D::operator*=(const float scalar) {
  this->m_X *= scalar;
  this->m_Y *= scalar;
  this->m_Z *= scalar;
}
void P3D::operator/=(const float scalar) {
  this->m_X /= scalar;
  this->m_Y /= scalar;
  this->m_Z /= scalar;
}

worldSpace::worldSpace(float p_x, float p_y, float p_o, float p_r) {
  this->m_Orientation = p_o;
  this->m_X = p_x;
  this->m_Y = p_y;
  this->m_Radius = p_r;
}
bool worldSpace::operator==(const worldSpace &p_other) const {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y) &&
         (this->m_Orientation == p_other.m_Orientation);
}
bool worldSpace::operator!=(const worldSpace &p_other) const {
  return (this != &p_other);
}
worldSpace worldSpace::operator+(const P2D &p_other) const {
  return worldSpace(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
                    (this->m_Orientation));
}
worldSpace worldSpace::operator-(const P2D &p_other) const {
  return worldSpace(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
                    (this->m_Orientation));
}
worldSpace worldSpace::operator-(const worldSpace &p_other) const {
  return worldSpace(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
                    (this->m_Orientation - p_other.m_Orientation));
}
worldSpace worldSpace::operator+(const worldSpace &p_other) const {
  return worldSpace(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
                    this->m_Orientation + p_other.m_Orientation);
}
void worldSpace::operator+=(const P2D &p_other) {
  P2D temp(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
}
void worldSpace::operator-=(const P2D &p_other) {
  P2D temp(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y);
  this->m_X = temp.m_X;
  this->m_Y = temp.m_Y;
}

void worldSpace::operator+=(const worldSpace &p_other) {
  *this = (*this + p_other);
}
void worldSpace::operator-=(const worldSpace &p_other) {
  *this = (*this - p_other);
}
worldSpace worldSpace::operator=(const worldSpace &p_other) { return p_other; }
P3D IMU::Rotate(P3D p_position, P3D p_angle) {
  float cos_roll = cos(p_angle.m_Y);
  float sin_roll = sin(p_angle.m_Y);
  float cos_pitch = cos(p_angle.m_X);
  float sin_pitch = sin(p_angle.m_X);
  float cos_yaw = cos(p_angle.m_Z);
  float sin_yaw = sin(p_angle.m_X);
  float xprime =
      (cos_yaw * cos_pitch) * p_position.m_X +
      (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * p_position.m_Y +
      (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * p_position.m_Z;
  float yprime =
      (sin_yaw * cos_pitch) * p_position.m_X +
      (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * p_position.m_Y +
      (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * p_position.m_Z;
  float zprime = (-sin_pitch) * p_position.m_X +
                 (cos_pitch * sin_roll) * p_position.m_Y +
                 (cos_pitch * cos_roll) * p_position.m_Z;
  return P3D(xprime, yprime, zprime);
}
void IMU::Update() {
  Calibrate(300);
  while (G_ProgramRunning) {
    m_Velo += (sensors::accel::Raw() - m_VeloBias);
    m_Gyro += (sensors::gyro::Raw() - m_GyroBias);
    P3D delta = Rotate(m_Velo, m_Gyro);
    m_X += delta.m_X;
    m_Y += delta.m_Y;
    m_Z += delta.m_Z;
    msleep(delay);
  }
}
void IMU::Calibrate(int p_polls) {
  for (int i = 0; i < p_polls; i++) {
    m_VeloBias += sensors::accel::Raw();
    m_GyroBias += sensors::gyro::Raw();
    msleep(delay);
  }
  m_VeloBias /= p_polls;
  m_GyroBias /= p_polls;
}
float Interpolate(float p_timepercent) {
  /*2.16 for maximum smoothness*/
  return (
      std::pow(p_timepercent, 2.16) /
      (std::pow(p_timepercent, 2.16) + (std::pow((1 - p_timepercent), 2.16))));
}
P2D PointLerp(P2D p_0, P2D p_1, float p_t) { return p_0 + ((p_1 - p_0) * p_t); }
} // namespace BKND
