#include "../include/interface.h"
#include <kipr/time/time.h>
using namespace BKND;
Motors tank(3, 0, .97, 1, 3.5 / 2, 6.75 / 2);
Servos arm(0, P2D(315, 0), P2D(1365, 90));
Servos claw(1, P2D(0, 0), P2D(1, 10000), 'm');
Sensors<sensors::type::Analog> startlight(1);
Sensors<sensors::type::Analog> ground(0);
PathFind nav(tank.m_pass);
void Wait(float seconds) {
  tank.Brake();
  msleep(seconds * 1000);
}
void printnav() {
  std::cout << "(" << G_Odometry.m_X << ", " << G_Odometry.m_Y << "), "
            << G_Odometry.m_Orientation << std::endl;
}
int main() {
  Thread VELOCITY([]() { motors::Velocity(tank.m_pass); });
  Thread TIMER([]() { misc::Timer(); });
  VELOCITY.Run();
  claw.Set(90);
  arm.Set(180);
  if (false /*tournament mode*/) {
    misc::WaitForLight(startlight.m_Port);
    shut_down_in(119);
  }
  TIMER.Run();
  while (ground.Value() < 0.92) {
    tank.Speed(-100, -20, 0.1);
  }
  tank.Speed(-100, -20, 1);
  while (ground.Value() < 0.92) {
    tank.Speed(-100, 20, 0.1);
  }
  Wait(.5);
  /* REORIENTED */
  tank.Speed(85, 100, 1.5);
  tank.Speed(100, 15, 1.18);
  /* HALLWAY */
  long t1 = BKND::G_CurrentMS;
  while (ground.Value() < 0.92) {
    tank.Speed(50, 50, 0.1);
  }
  long t2 = BKND::G_CurrentMS;
  tank.Speed(50, 50, ((float)(t2 - t1) / 1000) - 2);
  tank.Speed(0, 100, 1.25);
  tank.Speed(-75, -25, 1.5);
  tank.Speed(-50, -50, 8);
  Wait(5);
  /* IN STARTING BOX */
  tank.Speed(50, 50, 2);
  tank.Speed(50, -50, 3);
  tank.Speed(-75, -75, 3);
  arm.Set(45);
  tank.Speed(-75, -75, 1);
  Wait(1);
  claw.Set(0);
  Wait(1);
  arm.Set(180);
  Wait(1);
  tank.Speed(75, 75, 4);
  tank.Speed(0, 50, 3);
  tank.Speed(50, 50, 2);
  arm.Set(0);
  claw.Set(90);
  return 0;
}
