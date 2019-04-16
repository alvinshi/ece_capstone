#include <Ultrasonic.h>

int trigPin1 = 9;
int echoPin1 = 10;
int trigPin2 = 11;
int echoPin2 = 12; 
Ultrasonic ultrasonic1(trigPin1, echoPin1, 40000UL); //Using a 40ms timeout should give you a maximum range of approximately 6.8m.
Ultrasonic ultrasonic2(trigPin2, echoPin2, 40000UL);

int sample_size = 10; //set avg sampling size
int detection_threshold = 18;
long cm1, cm2;
long sum1,sum2;
int max1,max2,min1,min2;

void setup() {
  //Serial Port begin
  Serial.begin(9600);
  Serial.println("Ultrasonic Started");
}
 
void loop()
{
   if (Serial.available() > 0) {
    char input = Serial.read();
    if (input = 'M') {
        sum1=0;
        sum2=0;
        max1=0;
        max2=0;
        min1=1000;
        min2=1000;
        for (int i = 0; i < sample_size; i++) {
          int cur1=ultrasonic1.read();
          int cur2=ultrasonic2.read();
          sum1+=cur1;
          sum2+=cur2;
          if(cur1>max1) max1=cur1;
          if(cur1<min1) min1=cur1;
          if(cur2>max2) max2=cur2;
          if(cur2<min2) min2=cur2;
        }
        cm1 = (sum1-max1-min1)/(sample_size-2); 
        cm2 = (sum2-max2-min2)/(sample_size-2);
        
        if(cm1 <= detection_threshold || cm2 <= detection_threshold) {
          Serial.println("1");
        }
        else {
          Serial.println("0");
        }
    }
    else {
      Serial.println("-1");
    }
    delay(100);
   }
}
