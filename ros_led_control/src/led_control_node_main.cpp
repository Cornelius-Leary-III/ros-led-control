#include "gui/LedControlMainWindow.h"

#include <QApplication>

#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv)
{
   QApplication led_control_gui_app(argc, argv);

   LedControlMainWindow led_control_gui_main_window(argc, argv);
   led_control_gui_main_window.show();

   led_control_gui_app.exec();

   return 0;
}
