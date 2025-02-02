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
  std::cout << "here";
  pathFind::pathfunc func =
      pathFind::MakePath({P2D(0, 0), P2D(0, 10), P2D(10, 10)});
  float n = 0.01;
  nav.GoTo(func(n), 0.5);
  for (float i = n; i <= 1; i += n) {
    std::cout << "f(" << i << ")={" << func(i).m_X << "," << func(i).m_Y << "}"
              << std::endl;
    nav.GoTo(func(i), n * 15);
    printnav();
  }
  ao();
  G_file.close();
  return 0;
}
