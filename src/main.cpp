#include "../include/interface.h"
using namespace BKND;
Motors tank(3, 0, .985, 1, 2.5, 2.4125);
Servos arm(0, P2D(315, 0), P2D(1365, 90));
Servos claw(1, P2D(0, 0), P2D(1000, 90));
Sensors<sensors::type::Analog> startlight(1);
Sensors<sensors::type::Analog> ground(0);
PathFind navigate(tank.m_pass);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
int main() {
  Thread VELOCITY([]() { motors::Velocity(tank.m_pass); });
  Thread TIMER([]() { misc::Timer(); });
  TIMER.Run();
  VELOCITY.Run();
  arm.Set(180);
  std::cout << sensors::battery::Power();
  tank.Brake();
  return 0;
}
