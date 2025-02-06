#pragma once
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <kipr/accel/accel.h>
#include <kipr/kipr.h>
#include <limits>
#include <thread>
#include <vector>
using std::string;
using std::vector; // IGNORE CLANG
template <typename T> using initlist = std::initializer_list<T>;

namespace BKND { // backend

class Thread; // threading functionality (run multiple things at once)
class P2D;    // 2d point
using pointpair = std::pair<BKND::P2D, BKND::P2D>; // pair of points
class P3D;                                         // 3d point
class worldSpace; // 2d point with collision functionality
class IMU;        // innertial measurement unit (accelerometer, gyroscope)
class P2D {
public:
  float m_X;
  float m_Y;
  P2D(float x = 0, float y = 0);
  float Magnitude();
  float Angle(); // angle relative to (0,0) in degrees
  bool operator==(const P2D &p_other) const; // compare
  P2D operator+(const P2D &p_other) const;   // translate
  P2D operator-(const P2D &p_other) const;
  P2D operator*(const float p_scalar) const; // scale
  P2D operator/(const float p_scalar) const;
  void operator=(const P2D &p_other); // set
  void operator+=(const P2D &p_other);
  void operator-=(const P2D &p_other);
  void operator*=(const float p_scalar);
  void operator/=(const float p_scalar);
};
class P3D {
public:
  float m_X;
  float m_Y;
  float m_Z;
  P3D(float p_x = 0, float p_y = 0, float p_z = 0);
  float Magnitude();
  float Pitch();
  float Yaw();
  bool operator==(const P3D &p_other) const;
  P3D operator+(const P3D &p_other) const;
  P3D operator-(const P3D &p_other) const;
  P3D operator*(const float p_scalar) const;
  P3D operator/(const float p_scalar) const;
  void operator=(const P3D &p_other);
  void operator+=(const P3D &p_other);
  void operator-=(const P3D &p_other);
  void operator*=(const float p_scalar);
  void operator/=(const float p_scalar);
};
class worldSpace : public P2D {
public:
  float m_Orientation;
  float m_Radius; // radius of collision

  worldSpace(float p_x = 0, float p_y = 0, float p_o = 0, float p_r = 0);

  bool operator==(const worldSpace &p_other) const; // is coliding with other?
  bool operator!=(const worldSpace &p_other) const;
  worldSpace operator+(const P2D &p_other) const;
  worldSpace operator+(const worldSpace &p_other) const;
  worldSpace operator-(const P2D &p_other) const;
  worldSpace operator-(const worldSpace &p_other) const;
  void operator+=(const P2D &p_other);
  void operator+=(const worldSpace &p_other);
  void operator-=(const P2D &p_other);
  void operator-=(const worldSpace &p_other);
  worldSpace operator=(const worldSpace &p_other);
}; // namespace worldSpace:public P2D
struct pass {
  int leftmotor; // left motor port
  int rightmotor;
  float lmm; // left motor multiplier
  float rmm;
  float tmm = 1;     // time multiplier, maintains distances
  float wheelradius; // distance from center to edge of wheel
  float wheelbase;   // distance from center of bot to wheel
  float &leftspeed;  // dont set, just read
  float &rightspeed;
  pass(int p_leftmotorport, int p_rightmotorport = 5,
       float p_leftmultiplier = 1, float p_rightmultiplier = 1,
       float p_wheelradius = 1, float p_wheelbase = 1,
       float &p_leftspeed = *new float, float &p_rightspeed = *new float)
      : leftmotor(p_leftmotorport), rightmotor(p_rightmotorport),
        lmm(p_leftmultiplier), rmm(p_rightmultiplier),
        wheelradius(p_wheelradius), wheelbase(p_wheelbase),
        leftspeed(p_leftspeed), rightspeed(p_rightspeed) {};
};
float Deg(float p_radians); // rad to deg
/* IN: Radians
 * OUT: Degrees */
float NormalizeAngle(float p_angle); // normalize angle to +- 180 deg
/* IN: Degrees
 * OUT: Degrees
 * EG: NormalizeAngle(361) == -179
 */
float Rad(float p_degrees); // deg to rad
/* IN: Degrees
 * OUT: Radians */

float UnitConvert(pointpair p_conversion,
                  float p_x); // interpolate from point a to point b
/* IN: pointpair (2 points), any,
 * OUT: point
 * EG: lerp(pointpair(P2D(0,0),P2D(1,2)),10) == 20
 * Good for unit conversions */
BKND::P2D PointLerp(BKND::P2D p_one, BKND::P2D p_two,
                    float p_x); // lerp without pointpair syntax
float Interpolate(float p_x);   // smooth interpolation
/* IN: number from 0-1
 * OUT: smoother number from 0-1
 * 2nd derivative continuity
 * designed to be used in loops
 * EG: for (float i = 0; i <= 1;
 * i+=0.01){Tank.Speed(Interpolate(i),Interpolate(i),0.01);} */
template <typename A, typename B, typename C>
inline bool Clamp(A p_min, B p_val, C p_max) { // is b between a and c?
  return (p_min < p_val && p_val < p_max);
}

template <typename A, typename B, typename C>
inline bool MarginOfError(A p_inputa, B p_inputb,
                          C p_range) { // is a within b+-c
  float min = p_inputb - p_range;
  float max = p_inputb + p_range;
  return Clamp(min, p_inputa, max);
}
string PrettyTime(int p_milliseconds); /*display milliseconds as min:sec.ms*/

namespace misc {
void WaitForLight(int p_port); // wait until a light turns on to do next move
void Timer(); // starts a clock, updates a global variable, DOES NOT END

namespace buttons {
void Show(bool p_visibility); // set visibility of buttons
bool Visible();               // get visibility of buttons
namespace up {
bool A(); // is "A" button not pressed
bool B(); // im not repeating myself for all 6
bool C();
bool X();
bool Y();
bool Z();
}; // namespace up
namespace down {
bool A(); // is A button pressed
bool B();
bool C();
bool X();
bool Y();
bool Z();
}; // namespace down
namespace pressed {
bool A(); // has A button been pressed
bool B();
bool C();
bool X();
bool Y();
bool Z();
}; // namespace pressed
}; // namespace buttons
}; // namespace misc

namespace pathFind {
using pathfunc = std::function<BKND::P2D(
    float)>; // any function that takes a time value and returns a point
pathfunc
MakePath(initlist<BKND::P2D> p_pointlist); // makes an n degree bezier curve
/* IN: point0,point1,..,pointN
 * OUT: pathfunc from 0-1
 * EG: MakeSinglePath({P2D(0,0),P2D(0,1),P2D(1,1)});
 */
pathfunc MakePath(initlist<initlist<BKND::P2D>>
                      p_pointlistlist); // makes multiple n degree bezier
                                        // curves, stitches them together
/* IN: point0,point1,..,pointN
* OUT: pathfunc from 0-1
* EG: MakeSinglePath({{P2D(0,0),P2D(0,1),P2D(1,1)},
                        {P2D(1,1),P2D(2,1)},
                        {P2D(2,1),P2D(0,0)}}); */
pathfunc MakePath(initlist<pathfunc> p_pathlist); // stitch curves together
void FollowPath(pathfunc p_path, float p_time, float p_start, float p_end,
                pass p_vals); // follow the pathfunction given in p_time seconds
float PathLength(pathfunc p_path, float p_start = 0, float p_end = 1);
void Pathfind(float p_deltal, float p_deltar,
              pass p_vals); // where is the robot, determined by the left wheel
                            // and the right wheel's distances
void Face(float p_angle, float p_time, pass p_vals); // face x degrees in y time
void GoTo(BKND::P2D p_position, float p_time,
          pass p_vals); // goto a point in x time
std::array<float, 3> PointsToCircle(
    std::array<BKND::P2D, 3>
        p_points); // convert any 3 points into a circle, return [x,y,radius]
void FollowCircle(float p_radius, float p_theta, float p_time,
                  pass p_vals); // follow a radius for theta degrees. -to the
                                // left, +to the right
}; // namespace pathFind

namespace sensors {
enum type { Analog, Digital };

namespace digital {
bool Value(int p_port); // is port pressed?
};
namespace analog {
float Value(int p_port); // value from 0 to 1 of port
int Raw(int p_port);     // value from 0 to 2047 of port
}; // namespace analog
namespace accel {
BKND::P3D Raw();  // get raw accelerometer values
void Calibrate(); // callibrate accelerometer
float Magnitude();
float Pitch();
float Yaw();
void Update();

}; // namespace accel
namespace gyro {
BKND::P3D Raw();
void Calibrate();
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace gyro
namespace mag { // TODO
void Calibrate();
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace mag
namespace battery {
int Power();     // get power from 0 to 100 NOT ACCURATE
bool Critical(); // is power less than 33?
}; // namespace battery
}; // namespace sensors

namespace servos {
void Set(int p_port, float p_angle,
         pointpair p_conversion); // set port to angle and convert it to ticks
                                  // via pointpair unit conversion
void Change(int p_port, float p_angle,
            pointpair p_conversion); // current val + p_angle (can be negative)
void Move(int p_port, float p_angle, float p_time,
          pointpair p_conversion); // slow set
}; // namespace servos

namespace motors {
float GetLoad(int p_port);             // TODO
void ClearMotorRotations(pass p_vals); // set motor position counter to 0
void Velocity(
    pass p_vals); // get velocity of motors, put in thread, DOES NOT END

void Speed(float p_leftpercent, float p_rightpercent, float p_timeinseconds,
           pass p_vals);
void Distance(float p_leftinches, float p_rightinches, float p_timeinseconds,
              pass p_vals);
void Rotation(float p_leftdegrees, float p_rightdegrees, float p_timeinseconds,
              pass p_vals);
void Accelerate(
    float p_leftmaxpercent, float p_rightmaxpercent, float p_timeinseconds,
    pass p_vals); // accelerate from current to maxpercent in __ seconds
void Brake(pass p_vals);
}; // namespace motors

class Thread {
public:
  std::thread m_Thread; // the thread to run
  template <typename Func, typename... Args>
  Thread(Func &&p_func, Args &&...args)
      : m_Thread(std::forward<Func>(p_func), std::forward<Args>(args)...) {
  } // take a lambda and run it
  /* EG: Thread time([](){BKND::Timer();}); time.Run(); */
  void Run() { m_Thread.detach(); }
  void Kill() { m_Thread.join(); }
};

class IMU : public P3D {
  P3D m_Velo;
  P3D m_Gyro;
  P3D m_GyroBias;
  P3D m_VeloBias;
  P3D m_GlobAccel;
  Thread m_Update;
  void Update() { // TODO
    m_Velo += BKND::sensors::accel::Raw() - m_VeloBias;
    m_Gyro += BKND::sensors::gyro::Raw() - m_GyroBias;
    msleep(10);
    float cr = cos(m_Gyro.m_Y);
    float sr = sin(m_Gyro.m_Y);
    float cp = cos(m_Gyro.m_X);
    float sp = sin(m_Gyro.m_X);
    float cy = cos(m_Gyro.m_Z);
    float sy = sin(m_Gyro.m_X);
    float x = (cy * cp) * m_Velo.m_X + (cy * sp * sr - sy * cr) * m_Velo.m_Y +
              (cy * sp * cr + sy * sr) * m_Velo.m_Z;
    float y = (sy * cp) * m_Velo.m_X + (sy * sp * sr + cy * cr) * m_Velo.m_Y +
              (sy * sp * cr - cy * sr) * m_Velo.m_Z;
    float z =
        (-sp) * m_Velo.m_X + (cp * sr) * m_Velo.m_Y + (cp * cr) * m_Velo.m_Z;
    m_X += x;
    m_Y += y;
    m_Z += z;
  }

public:
  float m_Pitch, m_Yaw;
  IMU(float p_x, float p_y, float p_z, float p_pitch, float p_yaw)
      : P3D(p_x, p_y, p_z), m_Update([this]() { Update(); }), m_Pitch(p_pitch),
        m_Yaw(p_yaw) {}
  void Calibrate(int p_polls = 30) {
    /*
       V
       |
    O|---|O
     |___|
    */
    for (int i = 0; i < p_polls; i++) {              // normal
      m_GlobAccel.m_X += (float)accel_x() / p_polls; // left
      m_GlobAccel.m_Y += (float)accel_y() / p_polls; // front
      m_GlobAccel.m_Z += (float)accel_z() / p_polls; // up
      msleep(10);
    }
    /*
     \/
     |
    (O)
    |_|
    */
    for (int i = 0; i < p_polls; i++) {              // on right
      m_GlobAccel.m_X += (float)accel_z() / p_polls; // left
      m_GlobAccel.m_Y += (float)accel_y() / p_polls; // front
      m_GlobAccel.m_Z += (float)accel_x() / p_polls; // up
      msleep(10);
    }
    /*
       v
    O|='=|O
    */
    for (int i = 0; i < p_polls; i++) {              // on back
      m_GlobAccel.m_X += (float)accel_x() / p_polls; // left
      m_GlobAccel.m_Y += (float)accel_z() / p_polls; // front
      m_GlobAccel.m_Z += (float)accel_y() / p_polls; // up
      msleep(10);
    }
    for (int i = 0; i < p_polls; i++) {
      m_VeloBias += m_GlobAccel - (BKND::sensors::accel::Raw() / p_polls);
      m_GyroBias += BKND::sensors::accel::Raw() / p_polls;
      msleep(10);
    }
  }
};
inline BKND::pointpair Inverse(BKND::pointpair p) {
  return BKND::pointpair(BKND::P2D(p.first.m_Y, p.first.m_X),
                         BKND::P2D(p.second.m_Y, p.second.m_X));
}
extern long int G_CurrentMS;  // ms elapsed since timer called
extern std::ofstream G_file;  // log file
extern worldSpace G_Odometry; // odometer from wheels
extern IMU G_IMU;

extern BKND::pointpair TTD;   // ticks to degrees unit conversion
extern BKND::pointpair TTI;   // ticks to inches
extern BKND::pointpair ITD;   // inches to degrees
extern BKND::pointpair DTI;   // degrees to inches
extern BKND::pointpair DTT;   // degrees to ticks
extern BKND::pointpair ITT;   // inches to ticks
extern BKND::pointpair TPSTP; // ticks per second to percentspeed
extern BKND::pointpair PTTPS; // percentspeed to ticks per second

template <typename T>
inline void logVariable(string name, T val) { // TODO: Log variables macro
  std::cout << "    " << name << "=" << val << ";" << std::endl;
}
#define DBUG /* Debug info*/                                                   \
  BKND::G_file << __FILE__ << ":" << __LINE__ << " " << __PRETTY_FUNCTION__    \
               << " @ " << BKND::PrettyTime(BKND::G_CurrentMS) << std::endl
} // namespace BKND
