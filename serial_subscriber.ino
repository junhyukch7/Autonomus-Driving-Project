#if (ARDUINO >= 100)

 #include <Arduino.h>

#else

 #include <WProgram.h>

#endif

 

#include <Servo.h> 

#include <ros.h>

#include <std_msgs/UInt16.h>

 

ros::NodeHandle  nh;

 

Servo servo;

Servo motor;

 

void servo_cb( const std_msgs::UInt16& cmd_msg){

  servo.write(cmd_msg.data); //set servo angle, should be from 0-180  

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  

}

 

void motor_cb( const std_msgs::UInt16& cmd_msg){

  motor.write(cmd_msg.data); //set Throtle 99 is good speed

}

 

 

ros::Subscriber<std_msgs::UInt16> sub1("/Servo_angle", servo_cb);

ros::Subscriber<std_msgs::UInt16> sub2("/Throtle", motor_cb);

 

void setup(){

  pinMode(13, OUTPUT);

 

  nh.initNode();

  nh.subscribe(sub1);

  nh.subscribe(sub2);

 

  servo.attach(11); //attach it to pin 11(Servo)

  motor.attach(10); //attach it to pin 10(Motor)

  delay(2500);

}

 

void loop(){

  nh.spinOnce();

  delay(1);

}
