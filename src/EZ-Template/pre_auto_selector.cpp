#include "pre_auto_selector.hpp"

namespace ez
{
  bool limitSwitchesExist = false;
  pros::ADIDigitalIn left_limit_switch(-1);
  pros::ADIDigitalIn right_limit_switch(-1);
  void limit_switch_lcd_initialize(int left_limit_port, int right_limit_port)
  {
    if(left_limit_port == -1 || right_limit_port == -1)
    {
      limit_switch_task.suspend();
      return;
    }
    limitSwitchesExist = true;
    left_limit_switch = pros::ADIDigitalIn(left_limit_port);
    right_limit_switch = pros::ADIDigitalIn(right_limit_port);
    limit_switch_task.resume();
  }

  void limitSwitchTask()
  {
    while(true)
    {
      if(limitSwitchesExist)
      {
        if(left_limit_switch.get_new_press())
        {
          ez::as::page_down();
        }
        else if(right_limit_switch.get_new_press())
        {
          ez::as::page_up();
        }
      }

      pros::delay(20);
  }
}
  pros::Task limit_switch_task(limitSwitchTask, nullptr);
}
