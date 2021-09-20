#include <ros_led_control/led_control_node.h>

const double gRunLoopFreqInHz               = 10;
static int   gCurrentControlSequenceID      = 0;
static int   gCurrentControlSetupSequenceID = 0;

LedControlNode::LedControlNode(int argc, char** argv, QObject* parent)
  : QThread(parent),
    mNodeHandle(nullptr),
    mArgC(argc),
    mArgV(argv),
    mMutex(new QMutex()),
    mCurrentLedControlMsg(),
    mCurrentLedControlSetupMsg()
{
}

LedControlNode::~LedControlNode()
{
   if (mNodeHandle != nullptr)
   {
      delete mNodeHandle;
      mNodeHandle = nullptr;
   }
}

void LedControlNode::run()
{
   ros::init(mArgC, mArgV, "led_control_node");

   mNodeHandle = new ros::NodeHandle();

   //   ros::AsyncSpinner spinner(0);
   //   spinner.start();

   mPublisherLedControl = mNodeHandle->advertise<led_msgs::ledControl>("led_control", 10);

   mPublisherLedControlSetup =
         mNodeHandle->advertise<led_msgs::ledControlSetup>("led_control_setup", 10);

   //   ros::waitForShutdown();

   sendTurnOffLeds();

   ros::Rate sleep_rate(gRunLoopFreqInHz);

   while (ros::ok())
   {
      //      mPublisherLedControlSetup.publish(mCurrentLedControlSetupMsg);
      //      mPublisherLedControl.publish(mCurrentLedControlMsg);

      sleep_rate.sleep();
   }

   sendTurnOffLeds();
}

void LedControlNode::onTurnOffAllLeds(bool turn_off_leds)
{
   Q_UNUSED(turn_off_leds)

   sendTurnOffLeds();
}

void LedControlNode::sendTurnOffLeds()
{
   QMutexLocker lock(mMutex);

   mCurrentLedControlMsg.behavior       = led_msgs::ledControl::BEHAVIOR_OFF;
   mCurrentLedControlMsg.group_behavior = led_msgs::ledControl::GROUP_ALL_LEDS;

   lock.unlock();

   sendLedControlMsg();
}

void LedControlNode::onSendLedControlSetupMsg(bool send_msg)
{
   Q_UNUSED(send_msg)

   sendLedControlSetupMsg();
}

void LedControlNode::onSendCurrentLedControlMsg(bool send_msg)
{
   Q_UNUSED(send_msg)

   sendLedControlMsg();
}

void LedControlNode::sendLedControlSetupMsg()
{
   QMutexLocker lock(mMutex);

   mCurrentLedControlSetupMsg.header.stamp = ros::Time::now();
   mCurrentLedControlSetupMsg.header.seq   = gCurrentControlSetupSequenceID++;

   lock.unlock();

   mPublisherLedControlSetup.publish(mCurrentLedControlSetupMsg);
}

void LedControlNode::sendLedControlMsg()
{
   QMutexLocker lock(mMutex);

   mCurrentLedControlMsg.header.stamp = ros::Time::now();
   mCurrentLedControlMsg.header.seq   = gCurrentControlSequenceID++;

   lock.unlock();

   mPublisherLedControl.publish(mCurrentLedControlMsg);
}

void LedControlNode::onBehaviorSelected(int index)
{
   int selected_behavior = -1;

   switch (index)
   {
      case 0:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_UNKNOWN;
         break;
      }
      case 1:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_OFF;
         break;
      }
      case 2:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_ON;
         break;
      }
      case 3:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_TOGGLE;
         break;
      }
      case 4:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_PULSE;
         break;
      }
      default:
      {
         selected_behavior = led_msgs::ledControl::BEHAVIOR_UNKNOWN;
         break;
      }
   }

   QMutexLocker lock(mMutex);

   mCurrentLedControlMsg.behavior = selected_behavior;
}

void LedControlNode::onGroupBehaviorSelected(int index)
{
   int selected_group_behavior = -1;

   switch (index)
   {
      case 0:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_UNKNOWN;
         break;
      }
      case 1:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_NO_LED;
         break;
      }
      case 2:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_SINGLE_LED;
         break;
      }
      case 3:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_ALL_LEDS;
         break;
      }
      case 4:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_CASCADE_LEDS_FORWARD;
         break;
      }
      case 5:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_CASCADE_LEDS_REVERSE;
         break;
      }
      default:
      {
         selected_group_behavior = led_msgs::ledControl::GROUP_UNKNOWN;
         break;
      }
   }

   QMutexLocker lock(mMutex);

   mCurrentLedControlMsg.group_behavior = selected_group_behavior;
}

void LedControlNode::onPulsePeriodChanged(const QString& text)
{
   if (text.isEmpty())
   {
      return;
   }

   QMutexLocker lock(mMutex);

   mCurrentLedControlSetupMsg.pulse_period_in_millis = text.toUInt();
}

void LedControlNode::onPulseToggleStepsChanged(const QString& text)
{
   if (text.isEmpty())
   {
      return;
   }

   QMutexLocker lock(mMutex);

   mCurrentLedControlSetupMsg.pulse_num_toggle_steps_in_cycle = text.toUInt();
}

void LedControlNode::onPulseOffStepsChanged(const QString& text)
{
   if (text.isEmpty())
   {
      return;
   }

   QMutexLocker lock(mMutex);

   mCurrentLedControlSetupMsg.pulse_num_off_steps_in_cycle = text.toUInt();
}
