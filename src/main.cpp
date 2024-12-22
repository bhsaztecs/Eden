#include "../include/interface.h"
Motors tank(3, 0, .985, 1, 2.5, 2.4125);
Servos arm(0, BKND::P2D(315, 0), BKND::P2D(1365, 90));
Servos claw(1, BKND::P2D(0, 0), BKND::P2D(1000, 90));
Sensors<BKND::sensors::type::Analog> startlight(1);
Sensors<BKND::sensors::type::Analog> ground(0);
PathFind navigate(tank.m_pass);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
int main() {
  BKND::Thread VELOCITY([]() { BKND::motors::Velocity(tank.m_pass); });
  BKND::Thread TIMER([]() { BKND::misc::Timer(); });
  TIMER.Run();
  VELOCITY.Run();
  arm.Set(180);
  navigate.GoTo(BKND::P2D(3 * 12, 0), 5);
  navigate.GoTo(BKND::P2D(1, 1), 5);
  navigate.Face(0, 2);
  tank.Brake();
  return 0;
}
