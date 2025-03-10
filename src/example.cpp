#include "../include/interface.h"
using namespace BKND;
Motors tank(3, 0, .97, 1, 3.5 / 2, 6.75 / 2);
Servos arm(3, P2D(-45, 2050), P2D(0, 1500));
Servos claw(1, P2D(0, 0), P2D(1, -7500), true);
Sensors<sensors::type::Analog> startlight(0);
Sensors<sensors::type::Analog> ground(1);
PathFind nav(tank.m_Pass);

class ConData {
public:
  ConData(std::string) {}
  ConData(const ConData &) = default;
  ConData(worldSpace Position = G_Odometry, int ElapsedMS = G_CurrentMS,
          bool Running = G_ProgramRunning, bool Colided = G_Colided)
      : Position(Position), ElapsedMS(ElapsedMS), Running(Running),
        Colided(Colided) {}
  worldSpace Position;
  P2D Velocity;
  int ElapsedMS;
  bool Running, Colided;
};
void cleanup() {
  ao();
  G_ProgramRunning = false;
  G_File.close();
}
void MyColisionHandler(pass p_vals) {
  motors::Brake(p_vals);
  G_Colided = false;
}

int main() {
  G_CollisionHandler = MyColisionHandler;
  std::atexit(cleanup);
  if (false /*tournament mode*/) {
    misc::WaitForLight(startlight.m_Port);
    shut_down_in(119);
  }
  while (ground.Value() < 0.75) { // until you see black tape
    tank.Speed(100, 100, 0.1);    // drive forward
  }
  arm.GoTo(45, 1); // bring the arm to 45 degrees over 1 second
  claw.Set(0);     // close the claw
  Connection<ConData> connection("192.168.125.1", true);
  // connect to that ip as host
  connection.Connect();
  ConData other = connection.m_Recieved.back(); // other = last recieved thing
  while (!other.Running) {                      // wait until other is running
    msleep(1000);
    other = connection.m_Recieved.back();
  }
  auto path = path::MakePath({G_Odometry, P2D(10, 10), P2D(0, 10)});
  // make bezier curve with points (current, (10,10),(0,10))
  nav.FollowPath(path, 10); // follow that path for 10 seconds
  return 0;
}
