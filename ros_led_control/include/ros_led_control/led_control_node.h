#ifndef LED_CONTROL_NODE_H
#define LED_CONTROL_NODE_H

#include <QObject>
#include <QThread>
#include <QMutex>

#include <ros/ros.h>

#include <led_msgs/ledControlSetup.h>
#include <led_msgs/ledControl.h>

class LedControlNode : public QThread
{
   Q_OBJECT

 public:
   LedControlNode(int argc, char** argv, QObject* parent = nullptr);

   virtual ~LedControlNode();

   void run() override;

 signals:

 public slots:
   void onTurnOffAllLeds(bool turn_off_leds);
   void onSendLedControlSetupMsg(bool send_msg);
   void onSendCurrentLedControlMsg(bool send_msg);

   void onBehaviorSelected(int index);
   void onGroupBehaviorSelected(int index);

   void onPulsePeriodChanged(const QString& text);
   void onPulseToggleStepsChanged(const QString& text);
   void onPulseOffStepsChanged(const QString& text);

 private:
   void sendTurnOffLeds();
   void sendLedControlSetupMsg();
   void sendLedControlMsg();

   ros::NodeHandle* mNodeHandle;

   int    mArgC;
   char** mArgV;

   QMutex* mMutex;

   ros::Publisher       mPublisherLedControl;
   led_msgs::ledControl mCurrentLedControlMsg;

   ros::Publisher            mPublisherLedControlSetup;
   led_msgs::ledControlSetup mCurrentLedControlSetupMsg;
};

#endif // LED_CONTROL_NODE_H
