#include "../include/interface.h"
Motors tank(3, 0, .97, 1.0, 5, 2.8, BKND::P2D(0, 0), BKND::P2D(100, 1500));
Servos arm(0, BKND::P2D(0, 315), BKND::P2D(90, 1365));
Servos claw(1, BKND::P2D(0, 0), BKND::P2D(90, 1000));
Sensors<BKND::sensors::type::Analog> startlight(1);
PathFind navigate(tank.m_pass);
Sensors<BKND::sensors::type::Analog> ground(0);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
int main() {
  msleep(1000);
  arm.Set(90 + 45);
  claw.Set(90);
  if (false /*tournament mode*/) {
    BKND::misc::waitforlight(startlight.m_Port);
    shut_down_in(119);
  }
  BKND::Thread VELOCITY([]() { BKND::motors::Velocity(tank.m_pass); });
  BKND::Thread TIMER([]() { BKND::misc::Timer(); });
  TIMER.Run();
  VELOCITY.Run();
  while (ground.Value() < 0.92) {
    tank.Speed(-100, -20, 0.1);
  }
  tank.Speed(-100, -20, 1);
  while (ground.Value() < 0.92) {
    tank.Speed(-100, 20, 0.1);
  }
  ao();
  msleep(500);
  /* REORIENTED */
  tank.Speed(85, 100, 1.5);
  tank.Speed(100, 15, 1.18);
  /* HALLWAY */
  auto t1 = BKND::G_CurrentMS;
  while (ground.Value() < 0.92) {
    tank.Speed(50, 50, 0.1);
  }
  auto t2 = BKND::G_CurrentMS;
  tank.Speed(50, 50, ((float)(t2 - t1) / 1000) - 2);
  tank.Speed(0, 100, 1.25);
  tank.Speed(-75, -25, 1.5);
  tank.Speed(-50, -50, 8);
  Wait(5);
  /* IN STARTING BOX */
  tank.Speed(50, 50, 2);
  tank.Speed(50, -50, 3);
  tank.Speed(-75, -75, 3);
  arm.Set(0);
  tank.Speed(-75, -75, 1);
  Wait(1);
  claw.Set(0);
  Wait(1);
  arm.Set(90 + 45);
  Wait(1);
  tank.Speed(75, 75, 4);
  tank.Speed(0, 50, 3);
  tank.Speed(50, 50, 2);
  arm.Set(0);
  claw.Set(90);
  return 0;
}
