#include "../include/declarations.h"
namespace BKND {

BKND::pointpair TTD(BKND::P2D(0, 0), BKND::P2D(1, 5.55));
BKND::pointpair DTT(BKND::P2D(0, 0), BKND::P2D(5.55, 1));

BKND::pointpair ITD(BKND::P2D(0, 0), BKND::P2D(1, Deg(1.5) / 2));
BKND::pointpair DTI(BKND::P2D(0, 0), BKND::P2D(Deg(1.5) / 2, 1));
BKND::pointpair TTI(BKND::P2D(0, 0), BKND::P2D(230 / 1.0613, 1));
BKND::pointpair ITT(BKND::P2D(0, 0), BKND::P2D(1, 230));

BKND::pointpair PTTPS(BKND::P2D(0, 0), BKND::P2D(1, 15));
BKND::pointpair TPSTP(BKND::P2D(0, 0), BKND::P2D(15, 1));

long int G_CurrentMS = 0;
std::ofstream G_file(getLogfile());
std::vector<worldSpace *> G_Obstacles;
worldSpace G_Position(0, 0, 0, 0);

string getLogfile() { return "data/log.txt"; }
string PrettyTime(int p_ms) {
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
float mm(float p_inches) { return p_inches * 25.4; }
float inch(float p_mm) { return p_mm / 25.4; }

float lerp(pointpair p_slope, float p_x) {
  /* ( (del y over del x) * x ) + p0.y */
  float dely = p_slope.second.m_Y - p_slope.first.m_Y;
  float delx = p_slope.second.m_X - p_slope.first.m_X;
  float m = dely / delx;
  float b = -((m * p_slope.second.m_X) - p_slope.second.m_Y);
  return (m * p_x) + b;
}

P2D::P2D(float p_x, float p_y) {
  this->m_X = p_x;
  this->m_Y = p_y;
}
float P2D::Magnitude() const {
  return std::sqrt(std::pow(m_X, 2) + std::pow(m_Y, 2));
}
float P2D::Angle() const { return Deg(std::atan2(m_Y, m_X)); }
P2D P2D::operator-(const P2D &p_other) const {
  return P2D(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y);
}
P2D P2D::operator+(const P2D &p_other) const {
  return P2D(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y);
}
void P2D::operator=(const P2D &p_other) {
  this->m_X = p_other.m_X;
  this->m_Y = p_other.m_Y;
}
bool P2D::operator==(const P2D &p_other) const {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y);
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

P3D::P3D(float p_x, float p_y, float p_z) {
  this->m_X = p_x;
  this->m_Y = p_y;
  this->m_Z = p_z;
}
float P3D::Magnitude() const {
  return std::sqrt(std::pow(m_X, 2) + std::pow(m_Y, 2) + std::pow(m_Z, 2));
}
float P3D::Pitch() const { return Deg(std::atan(m_Y / m_Z)); }
float P3D::Yaw() const { return Deg(std::atan(m_X / m_Z)); }
P3D P3D::operator-(const P3D &p_other) const {
  return P3D(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
             this->m_Z - p_other.m_Z);
}
P3D P3D::operator+(const P3D &p_other) const {
  return P3D(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
             this->m_Z + p_other.m_Z);
}
void P3D::operator=(const P3D &p_other) {
  this->m_X = p_other.m_X;
  this->m_Y = p_other.m_Y;
  this->m_Z = p_other.m_Z;
}
bool P3D::operator==(const P3D &p_other) const {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y) &&
         (this->m_Z == p_other.m_Z);
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

worldSpace::worldSpace(float p_x, float p_y, float p_r, float p_orientation) {
  this->m_Orientation = p_orientation;
  this->m_X = p_x;
  this->m_Y = p_y;
  this->m_Radius = p_r;
}
bool worldSpace::operator==(const worldSpace &p_other) {
  return (this->m_X == p_other.m_X) && (this->m_Y == p_other.m_Y) &&
         (this->m_Orientation == p_other.m_Orientation);
}
bool worldSpace::operator!=(const worldSpace &p_other) {
  return (this != &p_other);
}
worldSpace worldSpace::operator-(const P2D &p_other) {
  return worldSpace(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
                    this->m_Orientation);
}
worldSpace worldSpace::operator+(const P2D &p_other) {
  return worldSpace(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
                    this->m_Orientation);
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
worldSpace worldSpace::operator-(const worldSpace &p_other) {
  return worldSpace(this->m_X - p_other.m_X, this->m_Y - p_other.m_Y,
                    this->m_Orientation - p_other.m_Orientation);
}
worldSpace worldSpace::operator+(const worldSpace &p_other) {
  return worldSpace(this->m_X + p_other.m_X, this->m_Y + p_other.m_Y,
                    this->m_Orientation + p_other.m_Orientation);
}
void worldSpace::operator+=(const worldSpace &p_other) {
  *this = (*this + p_other);
}
void worldSpace::operator-=(const worldSpace &p_other) {
  *this = (*this - p_other);
}
worldSpace worldSpace::operator=(const worldSpace &p_other) { return p_other; }

float Interpolate(float p_timepercent) {
  /*2.16 for maximum smoothness*/
  return (
      std::pow(p_timepercent, 2.16) /
      (std::pow(p_timepercent, 2.16) + (std::pow((1 - p_timepercent), 2.16))));
}

Thread::Thread(
    void (*p_func)()) { // create a new thread with a function as a parameter
  DBUG;
  m_Thethread = thread_create(p_func);
}
void Thread::Run() const { // start the thread
  DBUG;
  thread_start(m_Thethread);
}
void Thread::Kill() const { // end the thread
  DBUG;
  thread_destroy(m_Thethread);
}
} // namespace BKND
