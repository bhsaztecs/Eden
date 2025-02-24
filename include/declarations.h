#pragma once
#include <array>
#include <atomic>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <kipr/kipr.h>
#include <limits>
#include <thread>
#include <vector>
template <typename T> using initlist = std::initializer_list<T>;

namespace BKND { // backend
extern std::atomic<bool> G_ProgramRunning, G_Colided;
extern float G_ColisionLimit;
class Thread; // threading functionality (run multiple things at once)
class P2D;    // 2d point
using pointpair = std::pair<P2D, P2D>; // pair of points
class P3D;                             // 3d point
class worldSpace;                      // 2d point with collision functionality
class IMU; // innertial measurement unit (accelerometer, gyroscope)
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
P2D PointLerp(P2D p_one, P2D p_two,
              float p_x);     // lerp without pointpair syntax
float Interpolate(float p_x); // smooth interpolation
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
std::string
PrettyTime(int p_milliseconds); /*display milliseconds as min:sec.ms*/

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
using pathfunc = std::function<P2D(float)>;   // any function that takes a time
                                              // value and returns a point
pathfunc MakePath(initlist<P2D> p_pointlist); // makes an n degree bezier curve
/* IN: point0,point1,..,pointN
 * OUT: pathfunc from 0-1
 * EG: MakeSinglePath({P2D(0,0),P2D(0,1),P2D(1,1)});
 */
pathfunc MakePath(
    initlist<initlist<P2D>> p_pointlistlist); // makes multiple n degree bezier
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
void GoTo(P2D p_position, float p_time,
          pass p_vals); // goto a point in x time
std::array<float, 3>
PointsToCircle(std::array<P2D, 3> p_points); // convert any 3 points into a
                                             // circle, return [x,y,radius]
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
void DetectCollision(pass p_read);
P3D Raw();        // get raw accelerometer values
void Calibrate(); // callibrate accelerometer
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace accel
namespace gyro {
P3D Raw();
void Calibrate();
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace gyro
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
void HandleColision(pass p_vals);
namespace motors {
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
  /* EG: Thread time([](){Timer();}); */
  void Detach() { m_Thread.detach(); }
  void TryEnd() {
    if (m_Thread.joinable()) {
      m_Thread.join();
    }
  }
  void BlockEnd() {
    while (G_ProgramRunning) {
      TryEnd();
      msleep(100);
    }
  }
};

class IMU : public P3D {
  const int delay = 10;

public:
  P3D m_Velo;
  P3D m_Gyro;
  P3D m_GyroBias;
  P3D m_VeloBias;
  Thread m_Update;
  P3D Rotate(P3D p_position, P3D p_angle);
  void Update();
  IMU(float p_x, float p_y, float p_z, float p_pitch, float p_yaw)
      : P3D(p_x, p_y, p_z), m_Update([this]() { Update(); }) {}
  void Calibrate(int p_polls = 60);
};

inline pointpair Inverse(pointpair p) {
  return pointpair(P2D(p.first.m_Y, p.first.m_X),
                   P2D(p.second.m_Y, p.second.m_X));
}
extern void (*G_CollisionHandler)(pass);
extern long int G_CurrentMS;  // ms elapsed since timer called
extern std::ofstream G_File;  // log file
extern worldSpace G_Odometry; // odometer from wheels
extern std::vector<worldSpace *> Obstacles;
// extern IMU G_IMU;
extern pointpair TTD;   // ticks to degrees unit conversion
extern pointpair TTI;   // ticks to inches
extern pointpair ITD;   // inches to degrees
extern pointpair DTI;   // degrees to inches
extern pointpair DTT;   // degrees to ticks
extern pointpair ITT;   // inches to ticks
extern pointpair TPSTP; // ticks per second to percentspeed
extern pointpair PTTPS; // percentspeed to ticks per second

#define DBUG /* Debug info*/                                                   \
  BKND::G_File << __FILE__ << ":" << __LINE__ << " " << __PRETTY_FUNCTION__    \
               << " @ " << BKND::PrettyTime(BKND::G_CurrentMS) << std::endl
} // namespace BKND
