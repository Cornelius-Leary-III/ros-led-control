/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

const int gPinLedBlue = 22;
const int gPinLedRed = 26;
const int gPinLedYellow = 32;
const int gPinLedGreen = 36;

ros::NodeHandle  nh;

void messageCb( const std_msgs::Empty& toggle_msg)
{
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));     // blink the led
  
  digitalWrite(gPinLedBlue, HIGH-digitalRead(gPinLedBlue));     // blink the led
  digitalWrite(gPinLedRed, HIGH-digitalRead(gPinLedRed));       // blink the led
  digitalWrite(gPinLedYellow, HIGH-digitalRead(gPinLedYellow)); // blink the led
  digitalWrite(gPinLedGreen, HIGH-digitalRead(gPinLedGreen));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(gPinLedBlue, OUTPUT);
  pinMode(gPinLedRed, OUTPUT);
  pinMode(gPinLedYellow, OUTPUT);
  pinMode(gPinLedGreen, OUTPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(gPinLedBlue, HIGH);
  digitalWrite(gPinLedRed, LOW);
  digitalWrite(gPinLedYellow, HIGH);
  digitalWrite(gPinLedGreen, LOW);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
