#ifndef LEDCONTROLMAINWINDOW_H
#define LEDCONTROLMAINWINDOW_H

#include <QMainWindow>

#include <ros_led_control/led_control_node.h>

namespace Ui
{
class LedControlMainWindow;
}

class LedControlMainWindow : public QMainWindow
{
   Q_OBJECT

 public:
   explicit LedControlMainWindow(int argc, char** argv, QWidget* parent = nullptr);

   ~LedControlMainWindow();

 signals:
   void pulsePeriodChanged(const QString& text);
   void pulseToggleStepsChanged(const QString& text);
   void pulseOffStepsChanged(const QString& text);

   void currentBehaviorChanged(int index);
   void currentGroupBehaviorChanged(int index);

   void sendLedControlSetupMsg(bool send_msg);
   void sendCurrentLedControlMsg(bool send_msg);
   void turnOffAllLeds(bool turn_off_leds);

 public slots:
   void onLineEditPulsePeriodChanged();
   void onLineEditPulseToggleStepsChanged();
   void onLineEditPulseOffStepsChanged();
   void onComboBoxBehaviorSelected(int index);
   void onComboBoxGroupBehaviorSelected(int index);

 private:
   Ui::LedControlMainWindow* mUi;

   int    mArgC;
   char** mArgV;

   LedControlNode* mLedControlNode;

   QString mPulsePeriodCurrentText;
   QString mPulseToggleStepsCurrentText;
   QString mPulseOffStepsCurrentText;

   int mBehaviorCurrentIndex;
   int mGroupBehaviorCurrentIndex;
};

#endif // LEDCONTROLMAINWINDOW_H
