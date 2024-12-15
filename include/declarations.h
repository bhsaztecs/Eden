#pragma once
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <kipr/kipr.h>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using std::string;
using std::vector;
namespace BKND {
class Thread; // thread functionality
class P2D;    // 2 dimensional coordinate
using pointpair = std::pair<BKND::P2D, BKND::P2D>;
class P3D;        // 3 dimensional coordinate
class worldSpace; // 2 dimensional coordinate with additional functionality
class P2D {
public:
  float m_X;
  float m_Y;
  P2D(float x = 0, float y = 0);
  float Magnitude() const;
  float Angle() const;
  P2D operator-(const P2D &p_other) const;
  P2D operator+(const P2D &p_other) const;
  void operator=(const P2D &p_other);
  bool operator==(const P2D &p_other) const;
  void operator+=(const P2D &p_other);
  void operator-=(const P2D &p_other);
};
class P3D {

public:
  float m_X;
  float m_Y;
  float m_Z;
  P3D(float p_x = 0, float p_y = 0, float p_z = 0);
  float Magnitude() const;
  float Pitch() const;
  float Yaw() const;
  P3D operator-(const P3D &p_other) const;
  P3D operator+(const P3D &p_other) const;
  void operator=(const P3D &p_other);
  bool operator==(const P3D &p_other) const;
  void operator+=(const P3D &p_other);
  void operator-=(const P3D &p_other);
};
class worldSpace : public P2D {
public:
  float m_Orientation;
  float m_Radius;

  worldSpace(float p_x = 0, float p_y = 0, float p_r = 0, float p_o = 0);

  bool operator==(const worldSpace &p_other);
  bool operator!=(const worldSpace &p_other);
  worldSpace operator-(const P2D &p_other);
  worldSpace operator+(const P2D &p_other);
  void operator+=(const P2D &p_other);
  void operator-=(const P2D &p_other);
  worldSpace operator=(const worldSpace &p_other);
}; // namespace worldSpace:public P2D
struct pass {
  int leftmotor; // left motor port
  int rightmotor;
  float lmm; // left motor multiplier
  float rmm;
  float tmm;
  float margin;
  float turnrate;
  float &leftspeed;
  float &rightspeed;
  pointpair slope;
  pass(int p_leftmotorport, int p_rightmotorport, float p_leftmultiplier,
       float p_rightmultiplier, float p_timemultiplier, float p_athenamargin,
       float p_turnrate, float &p_leftspeed, float &p_rightspeed, BKND::P2D min,
       BKND::P2D max)
      : leftmotor(p_leftmotorport), rightmotor(p_rightmotorport),
        lmm(p_leftmultiplier), rmm(p_rightmultiplier), tmm(p_timemultiplier),
        margin(p_athenamargin), turnrate(p_turnrate), leftspeed(p_leftspeed),
        rightspeed(p_rightspeed), slope(min, max) {}
};
float Deg(float); /*Convert Radians to Degrees*/
float Rad(float); /*Convert Degrees to Radians*/
float lerp(pointpair,
           float); /*get a linear transition from (x1,y1) to (x2,y2) at x */
/* IN {-> (minx,miny) (maxx,maxy) x
 * OUT{-> y at x
 */
float Interpolate(float); /*get a smooth transition from 0 to 1*/
/* IN {-> Time (from 1 to 0)
 * OUT{-> value returned from the equation.
 */
template <typename A, typename B, typename C>
bool Clamp(A p_min, B p_val, C p_max) {
  return (p_min < p_val && p_val < p_max);
}

template <typename A, typename B, typename C>
bool MarginOfError(A p_inputa, B p_inputb, C p_range) {
  float min = p_inputb - p_range;
  float max = p_inputb + p_range;
  return Clamp(min, p_inputa, max);
}
string PrettyTime(int); /*display milliseconds as min:sec.ms*/

namespace misc {
void waitforlight(int);
void Timer();    // start a clock that updates a global variable. does not end
void HandsOff(); // starts handsoff/shutdownin
void Start(bool, int, bool, int, int, float, float,
           float); // initializes variables

namespace buttons {
void Show(bool); // show the buttons on screen?
bool Visible();  // are the buttons on screen?
namespace up {
bool A(); // is a button not pressed
bool B();
bool C();
bool X();
bool Y();
bool Z();
}; // namespace up
namespace down {
bool A(); // is a button pressed
bool B();
bool C();
bool X();
bool Y();
bool Z();
}; // namespace down
namespace pressed {
bool A(); // has a button been clicked
bool B();
bool C();
bool X();
bool Y();
bool Z();
}; // namespace pressed
}; // namespace buttons
}; // namespace misc

namespace pathFind {
void AthenaDecision(float, float,
                    pass); /*Decide which algorithm to use, then uses it.*/
/* IN {-> Change in Left wheel position, Change in Right wheel position
 */
void AngularPathfind(float, float,
                     pass);              /*Updates orientation based on inputs*/
void LinearPathfind(float, float, pass); /*Updates position based on inputs*/
void DynamicPathfind(float, float); /*Updates position and orientation based on
         inputs. MAY DIVIDE BY 0*/
void Face(float, float, pass); // face a certain degree heading in given time
void GoTo(BKND::P2D, float, pass); // go to an (x,y) coordinate in a given time
};                                 // namespace pathFind

namespace sensors {
enum type { Analog, Digital };

namespace dgtl {
bool Value(int); // get the value of a port
};               // namespace dgtl
namespace nlg {
float Value(int); // get the value of a port as a percent of the max
int Raw(int);     // get the raw value of a port
};                // namespace nlg
namespace accel {
void Calibrate();  // calibrates the accelerometer
float Magnitude(); // gets the magnitude of the vector the accelerometer is
                   // pointing at
float Pitch();     // gets the pitch of the vector
float Yaw();       // gets the yaw of the vector
void Update();     // update the vector's values. (automatically called on
                   // mag,pitch,yaw)
};                 // namespace accel
namespace gyro {
void Calibrate();
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace gyro
namespace mag {
void Calibrate();
float Magnitude();
float Pitch();
float Yaw();
void Update();
}; // namespace mag
namespace bttry {
int Power();     // get the power level from 0-100
bool Critical(); // is the battery less than 33% full?
};               // namespace bttry
};               // namespace sensors

namespace servos {
void Set(int, float, pointpair);
void Change(int, float, pointpair);
void Move(int, float, float, pointpair);
}; // namespace servos

namespace motors {
void ClearMotorRotations(pass); // sets the counter to 0
void Velocity(pass); // updates the global velocity variables, keep it in a
                     // thread. it has no end.
void Speed(float p_leftpercent, float p_rightpercent, float p_timeinseconds,
           pass p_vals); // drive at a speed for a time
void Distance(float p_leftinches, float p_rightinches, float p_timeinseconds,
              pass p_vals); // drive to a distance per wheel in this time
void Rotation(float p_leftdegrees, float p_rightdegrees, float p_timeinseconds,
              pass p_vals); // drive to an angle per wheel in this time
void Accelerate(float p_leftmaxpercent, float p_rightmaxpercent,
                float p_timeinseconds, pass p_vals); // interpolate to a speed
void Brake(pass p_vals);                             // turn on the brakes
};                                                   // namespace motors

float Interpolate(float p_timepercent);
class Thread {
public:
  thread m_Thethread;
  Thread(void (*p_func)());
  void Run() const;  // start the thread
  void Kill() const; // end the thread
};                   // namespace newThread

string getLogfile();

extern long int G_CurrentMS;
extern std::ofstream G_file;
extern std::vector<worldSpace *> G_Obstacles;
extern worldSpace G_Position;

template <typename T>
void logVariable(const std::string &name, const T &value) {
  BKND::G_file << name << "=" << value << "; ";
}

template <typename T, typename... Ts>
void logVariables(const std::string &names, const T &value, const Ts &...rest) {
  std::istringstream iss(names);
  std::string varName;
  std::getline(iss, varName, ',');
  logVariable(varName, value);
  if (sizeof...(rest) > 0) {
    logVariables(names.substr(varName.length() + 1), rest...);
  }
}

#define LOG_VARS(...) logVariables(#__VA_ARGS__, __VA_ARGS__)
#define DBUG                                                                   \
  BKND::G_file << __FILE__ << ":" << __LINE__ << " " << __PRETTY_FUNCTION__    \
               << " @ " << BKND::PrettyTime(BKND::G_CurrentMS) << std::endl;
} // namespace BKND
