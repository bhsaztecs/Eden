#include "../include/declarations.h"
namespace BKND {
namespace misc {
namespace buttons {
namespace down {
bool A() { return a_button() != 0; }
bool B() { return b_button() != 0; }
bool C() { return c_button() != 0; }
bool X() { return x_button() != 0; }
bool Y() { return y_button() != 0; }
bool Z() { return z_button() != 0; }
} // namespace down
namespace up {
bool A() { return a_button() == 0; }
bool B() { return b_button() == 0; }
bool C() { return c_button() == 0; }
bool X() { return x_button() == 0; }
bool Y() { return y_button() == 0; }
bool Z() { return z_button() == 0; }
} // namespace up
namespace pressed {
bool A() { return a_button_clicked() != 0; }
bool B() { return b_button_clicked() != 0; }
bool C() { return c_button_clicked() != 0; }
bool X() { return x_button_clicked() != 0; }
bool Y() { return y_button_clicked() != 0; }
bool Z() { return z_button_clicked() != 0; }
} // namespace pressed
void Show(bool p_vis) {
  if (p_vis) {
    extra_buttons_show();
    return;
  }
  extra_buttons_hide();
}
bool Visible() { return get_extra_buttons_visible() != 0; }
} // namespace buttons
void WaitForLight(int p_port) {
  DBUG;
  int onvalue;
  int offvalue;
  std::cout << "Running";
  while (any_button() == 0) {
    onvalue = analog(p_port);
    std::cout << "ON: " << onvalue << std::endl;
    msleep(25);
    console_clear();
    if (any_button() != 0) {
      break;
    }
    msleep(25);
  }
  msleep(500);
  while (any_button() == 0) {
    offvalue = analog(p_port);
    std::cout << "OFF: " << offvalue << std::endl;
    msleep(25);
    console_clear();
    if (any_button() != 0) {
      break;
    }
    msleep(25);
  }
  msleep(500);
  int minimumtorun = (offvalue - onvalue) * 0.1 + onvalue;
  std::cout << "thresh: " << minimumtorun << std::endl;

  for (int i = 0; analog(p_port) > minimumtorun; i++) {
    std::cout << "Waiting";
    if (i % 4 == 0) {
      std::cout << std::endl;
    } else if (i % 4 == 1) {
      std::cout << "." << std::endl;
    } else if (i % 4 == 2) {
      std::cout << ".." << std::endl;
    } else if (i % 4 == 3) {
      std::cout << "..." << std::endl;
    }
    msleep(500);
    console_clear();
  }
}
void Timer() {
  DBUG;
  while (true) {
    msleep(1);
    BKND::G_CurrentMS++;
  }
}
} // namespace misc
} // namespace BKND
