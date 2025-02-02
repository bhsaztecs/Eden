#include "../include/interface.h"
using namespace BKND;
Motors tank(3, 0, .97, 1, 3.5 / 2, 6.75 / 2);
Servos arm(3, P2D(-45, 2050), P2D(0, 1500));
Servos claw(1, P2D(0, 0), P2D(1, -7500), true);
Sensors<sensors::type::Analog> startlight(1);
Sensors<sensors::type::Analog> ground(0);
PathFind nav(tank.m_Pass);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
void printnav() {
  std::cout << "(" << G_Odometry.m_X << ", " << G_Odometry.m_Y << ")"
            << std::endl
            << std::endl;
}
int main() {
  Thread TIMER([]() { misc::Timer(); });
  if (false /*tournament mode*/) {
    misc::WaitForLight(startlight.m_Port);
    shut_down_in(119);
  }
  TIMER.Run();
  pathFind::pathfunc func =
      pathFind::MakePath({P2D(0, 0), P2D(0, 10), P2D(10, 10)});
  nav.GoTo(func(0), 1);            // reach point first
  nav.GoTo(func(0.001), .5);       // reach angle
  nav.FollowPath(func, 10, 0.002); // follow path
  ao();
  G_file.close();
  return 0;
}
