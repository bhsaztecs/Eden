#include "../include/interface.h"
using namespace BKND;
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
  pathFind::pathfunc func =
      pathFind::MakePath({P2D(0, 0), P2D(0, 10), P2D(10, 10)});
  nav.GoTo(func(0), 1);            // reach point first
  nav.GoTo(func(0.001), .5);       // reach angle
  nav.FollowPath(func, 10, 0.002); // follow path
  Connection<ConData> con("192.168.1.30", false);
  con.Connect();
  while (1) {
    ConData tosend;
    con.Send(tosend);
    std::cout << "Recieved " << tosend.Position.Magnitude() << " "
              << tosend.ElapsedMS << std::endl;
    ConData otherdata = con.m_Recieved.back();
    std::cout << "Recieved " << otherdata.Position.Magnitude() << " "
              << otherdata.ElapsedMS << std::endl;
    msleep(1000);
  }
  return 0;
}
