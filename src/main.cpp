#include "../include/interface.h"
using namespace BKND;
Motors tank(3, 0, .97, 1, 3.5 / 2, 6.75 / 2);
Servos arm(0, P2D(315, 0), P2D(1365, 90));
Servos claw(1, P2D(0, 0), P2D(1000, 90));
Sensors<sensors::type::Analog> startlight(1);
Sensors<sensors::type::Analog> ground(0);
PathFind nav(tank.m_pass);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
void printnav() {
  std::cout << "(" << G_Position.m_X << ", " << G_Position.m_Y << "), "
            << G_Position.m_Orientation << std::endl;
}
int main() {
  Thread VELOCITY([]() { motors::Velocity(tank.m_pass); });
  Thread TIMER([]() { misc::Timer(); });
  TIMER.Run();
  VELOCITY.Run();
  printnav();
  nav.Face(45, 2);
  printnav();
  nav.Face(0, 2);
  nav.GoTo(P2D(30, 30), 5);
  printnav();
  nav.GoTo(P2D(0, 0), 5);
  printnav();
  while (1) {
    nav.GoTo(P2D(0, 0), 1);
    printnav();
  }
  return 0;
}
