#include <Ultrasonic.h>
int trigPin1 = 9;
int echoPin1 = 10;
int trigPin2 = 11;
int echoPin2 = 12; 
long duration1, cm1, duration2, cm2; 
Ultrasonic ultrasonic1(trigPin1, echoPin1);
Ultrasonic ultrasonic2(trigPin2, echoPin2);
int sample_size=10; //set avg sampling size
long sum1,sum2;
int max1,max2,min1,min2;
bool detected=false;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  Serial.println("Ultrasonic Started");
}
 
void loop() {
  int i=0;
  sum1=0;
  sum2=0;
  max1=0;
  max2=0;
  min1=1000;
  min2=1000;
  while(i<sample_size){
    int cur1=ultrasonic1.read();
    int cur2=ultrasonic2.read();
    sum1+=cur1;
    sum2+=cur2;
    if(cur1>max1) max1=cur1;
    if(cur1<min1) min1=cur1;
    if(cur2>max2) max2=cur2;
    if(cur2<min2) min2=cur2;
    i+=1;
  }
  cm1 = (sum1-max1-min1)/(sample_size-2); 
  cm2 = (sum2-max2-min2)/(sample_size-2);

  if(cm1<20 || cm2<20){
    //send detected
  }
  else{
    //send not detected
  }

  /*Serial.print("Detected ");
  Serial.print(cm1);
  Serial.print("cm   ");
  Serial.print(cm2);
  Serial.print("cm");
  Serial.println();
  */
  //delay(10);
}
