/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

#include <led_msgs/ledControl.h>
#include <led_msgs/ledControlSetup.h>

const int gPinLedBlue   = 22;
const int gPinLedRed    = 26;
const int gPinLedYellow = 32;
const int gPinLedGreen  = 36;

const unsigned long gPulsePeriodInMillis        = 400;
const int           gPulseNumToggleStepsInCycle = 5;
const int           gPulseNumOffStepsInCycle    = 12;

unsigned long pulse_period_in_millis          = gPulsePeriodInMillis;
int           pulse_num_toggle_steps_in_cycle = gPulseNumToggleStepsInCycle;
int           pulse_num_off_steps_in_cycle    = gPulseNumOffStepsInCycle;
int           single_led_color                = gPinLedBlue;

ros::NodeHandle nh;
unsigned long   prev_time_in_millis            = 0;
int             current_num_toggles_in_cycle   = gPulseNumToggleStepsInCycle;
int             current_num_off_steps_in_cycle = gPulseNumOffStepsInCycle;
int             current_behavior               = -1;
int             current_group_behavior         = -1;
int             current_led_cascade            = led_msgs::ledControl::COLOR_GREEN;

bool is_current_operation_completed = false;

void writeLed(int pin, int state)
{
   digitalWrite(pin, state);
}

void writeAllLeds(int state)
{
   digitalWrite(LED_BUILTIN, state);
   digitalWrite(gPinLedBlue, state);
   digitalWrite(gPinLedRed, state);
   digitalWrite(gPinLedYellow, state);
   digitalWrite(gPinLedGreen, state);
}

void toggleLed(int pin)
{
   int current_state = digitalRead(pin);
   int next_state    = HIGH - current_state;

   digitalWrite(pin, next_state);
}

void toggleAllLeds()
{
   toggleLed(LED_BUILTIN);
   toggleLed(gPinLedBlue);
   toggleLed(gPinLedRed);
   toggleLed(gPinLedYellow);
   toggleLed(gPinLedGreen);
}

void pulseLed(int pin)
{
   if (current_num_toggles_in_cycle > 0)
   {
      toggleLed(pin);

      --current_num_toggles_in_cycle;

      is_current_operation_completed = false;
   }
   else if (current_num_off_steps_in_cycle > 0)
   {
      if (current_num_off_steps_in_cycle == pulse_num_off_steps_in_cycle)
      {
         toggleLed(pin);
      }

      --current_num_off_steps_in_cycle;

      is_current_operation_completed = false;
   }
   else
   {
      is_current_operation_completed = true;

      current_num_toggles_in_cycle   = pulse_num_toggle_steps_in_cycle;
      current_num_off_steps_in_cycle = pulse_num_off_steps_in_cycle;
   }
}

void pulseAllLedsSimultaneously()
{
   if (current_num_toggles_in_cycle > 0)
   {
      toggleAllLeds();

      --current_num_toggles_in_cycle;

      is_current_operation_completed = false;
   }
   else if (current_num_off_steps_in_cycle > 0)
   {
      if (current_num_off_steps_in_cycle == pulse_num_off_steps_in_cycle)
      {
         toggleAllLeds();
      }

      --current_num_off_steps_in_cycle;

      is_current_operation_completed = false;
   }
   else
   {
      is_current_operation_completed = true;

      current_num_toggles_in_cycle   = pulse_num_toggle_steps_in_cycle;
      current_num_off_steps_in_cycle = pulse_num_off_steps_in_cycle;
   }
}

int getPinFromColor(int color)
{
   switch (color)
   {
      case led_msgs::ledControl::COLOR_RED:
      {
         return gPinLedRed;
      }
      case led_msgs::ledControl::COLOR_YELLOW:
      {
         return gPinLedYellow;
      }
      case led_msgs::ledControl::COLOR_GREEN:
      {
         return gPinLedGreen;
      }
      case led_msgs::ledControl::COLOR_BLUE:
      {
         return gPinLedBlue;
      }
   }
}

int advanceLedForward(int current_color)
{
   int next_color = led_msgs::ledControl::COLOR_UNKNOWN;

   if (current_color == led_msgs::ledControl::COLOR_GREEN)
   {
      next_color = led_msgs::ledControl::COLOR_YELLOW;
   }
   else if (current_color == led_msgs::ledControl::COLOR_BLUE)
   {
      next_color = led_msgs::ledControl::COLOR_GREEN;
   }
   else if (current_color == led_msgs::ledControl::COLOR_RED)
   {
      next_color = led_msgs::ledControl::COLOR_BLUE;
   }
   else if (current_color == led_msgs::ledControl::COLOR_YELLOW)
   {
      next_color = led_msgs::ledControl::COLOR_RED;
   }

   return next_color;
}

int advanceLedReverse(int current_color)
{
   int next_color = led_msgs::ledControl::COLOR_UNKNOWN;

   if (current_color == led_msgs::ledControl::COLOR_RED)
   {
      next_color = led_msgs::ledControl::COLOR_YELLOW;
   }
   else if (current_color == led_msgs::ledControl::COLOR_YELLOW)
   {
      next_color = led_msgs::ledControl::COLOR_GREEN;
   }
   else if (current_color == led_msgs::ledControl::COLOR_GREEN)
   {
      next_color = led_msgs::ledControl::COLOR_BLUE;
   }
   else if (current_color == led_msgs::ledControl::COLOR_BLUE)
   {
      next_color = led_msgs::ledControl::COLOR_RED;
   }

   return next_color;
}

void led_control_setup_msg_callback(const led_msgs::ledControlSetup& led_control_setup_msg)
{
   pulse_period_in_millis          = led_control_setup_msg.pulse_period_in_millis;
   pulse_num_toggle_steps_in_cycle = led_control_setup_msg.pulse_num_toggle_steps_in_cycle;
   pulse_num_off_steps_in_cycle    = led_control_setup_msg.pulse_num_off_steps_in_cycle;

   single_led_color = led_control_setup_msg.single_led_color;
}

void led_control_msg_callback(const led_msgs::ledControl& led_control_msg)
{
   current_behavior       = led_control_msg.behavior;
   current_group_behavior = led_control_msg.group_behavior;

   switch (current_behavior)
   {
      case led_msgs::ledControl::BEHAVIOR_UNKNOWN:
      {
         // make no changes.
         break;
      }
      case led_msgs::ledControl::BEHAVIOR_OFF:
      {
         if (current_group_behavior == led_msgs::ledControl::GROUP_SINGLE_LED)
         {
            int pin = getPinFromColor(single_led_color);
            writeLed(pin, LOW);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_ALL_LEDS)
         {
            writeAllLeds(LOW);
         }
         break;
      }
      case led_msgs::ledControl::BEHAVIOR_ON:
      {
         if (current_group_behavior == led_msgs::ledControl::GROUP_SINGLE_LED)
         {
            int pin = getPinFromColor(single_led_color);
            writeLed(pin, HIGH);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_ALL_LEDS)
         {
            writeAllLeds(HIGH);
         }
         break;
      }
      case led_msgs::ledControl::BEHAVIOR_TOGGLE:
      {
         if (current_group_behavior == led_msgs::ledControl::GROUP_SINGLE_LED)
         {
            int pin = getPinFromColor(single_led_color);
            toggleLed(pin);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_ALL_LEDS)
         {
            toggleAllLeds();
         }
         break;
      }
      case led_msgs::ledControl::BEHAVIOR_PULSE:
      {
         // process this in loop() function.
         break;
      }
      default:
      {
         // do nothing.
         break;
      }
   }
}

ros::Subscriber<led_msgs::ledControl> sub_control("led_control", &led_control_msg_callback);

ros::Subscriber<led_msgs::ledControlSetup> sub_control_setup("led_control_setup",
                                                             &led_control_setup_msg_callback);

void setup()
{
   pinMode(LED_BUILTIN, OUTPUT);
   pinMode(gPinLedBlue, OUTPUT);
   pinMode(gPinLedRed, OUTPUT);
   pinMode(gPinLedYellow, OUTPUT);
   pinMode(gPinLedGreen, OUTPUT);

   nh.initNode();
   nh.subscribe(sub_control);
   nh.subscribe(sub_control_setup);

   digitalWrite(LED_BUILTIN, LOW);
   digitalWrite(gPinLedBlue, HIGH);
   digitalWrite(gPinLedRed, LOW);
   digitalWrite(gPinLedYellow, LOW);
   digitalWrite(gPinLedGreen, HIGH);
}

void loop()
{
   nh.spinOnce();
   delay(1);

   unsigned long current_time_in_millis = millis();

   bool is_period_elapsed = (current_time_in_millis - prev_time_in_millis) > pulse_period_in_millis;

   if (!is_period_elapsed)
   {
      return;
   }

   prev_time_in_millis = current_time_in_millis;

   switch (current_behavior)
   {
      case led_msgs::ledControl::BEHAVIOR_TOGGLE:
      {
         if (current_group_behavior == led_msgs::ledControl::GROUP_CASCADE_LEDS_FORWARD)
         {
            int color           = advanceLedForward(current_led_cascade);
            current_led_cascade = color;

            int pin = getPinFromColor(color);

            toggleLed(pin);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_CASCADE_LEDS_REVERSE)
         {
            int color           = advanceLedReverse(current_led_cascade);
            current_led_cascade = color;

            int pin = getPinFromColor(color);

            toggleLed(pin);
         }
         break;
      }
      case led_msgs::ledControl::BEHAVIOR_PULSE:
      {
         if (current_group_behavior == led_msgs::ledControl::GROUP_SINGLE_LED)
         {
            int pin = getPinFromColor(single_led_color);

            pulseLed(pin);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_ALL_LEDS)
         {
            pulseAllLedsSimultaneously();
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_CASCADE_LEDS_FORWARD)
         {
            if (is_current_operation_completed)
            {
               int color           = advanceLedForward(current_led_cascade);
               current_led_cascade = color;
            }

            int pin = getPinFromColor(current_led_cascade);

            pulseLed(pin);
         }
         else if (current_group_behavior == led_msgs::ledControl::GROUP_CASCADE_LEDS_REVERSE)
         {
            if (is_current_operation_completed)
            {
               int color           = advanceLedReverse(current_led_cascade);
               current_led_cascade = color;
            }

            int pin = getPinFromColor(current_led_cascade);

            pulseLed(pin);
         }
         break;
      }
      default:
      {
         // process additional cases in callback function.
         break;
      }
   }
}
