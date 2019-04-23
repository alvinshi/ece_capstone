#include <Ultrasonic.h>
#include <FastLED.h>

// 0 wait, 1 track, 2 approach, 3 detected
int prev_mode = -1;
int mode = 0;

// Ultrasonic Sensor Setup
int trigPin1 = 9;
int echoPin1 = 10;
int trigPin2 = 11;
int echoPin2 = 12; 
Ultrasonic ultrasonic1(trigPin1, echoPin1, 40000UL); //Using a 40ms timeout should give you a maximum range of approximately 6.8m.
Ultrasonic ultrasonic2(trigPin2, echoPin2, 40000UL);

int sample_size = 10; //set avg sampling size
int detection_threshold = 10;

// LED Setup
#define LED_PIN_1   4
#define LED_PIN_2   5
#define COLOR_ORDER RGB
#define CHIPSET     WS2811
#define NUM_LEDS    3

#define BRIGHTNESS  200
#define FRAMES_PER_SECOND 60

bool gReverseDirection = false;

CRGB leds_1[NUM_LEDS];
CRGB leds_2[NUM_LEDS];

void setup() {
  // sanity check delay - allows reprogramming if accidently blowing power w/leds
  delay(2000);
  FastLED.addLeds<CHIPSET, LED_PIN_1, COLOR_ORDER>(leds_1, NUM_LEDS);
  FastLED.addLeds<CHIPSET, LED_PIN_2, COLOR_ORDER>(leds_2, NUM_LEDS);
  //Serial Port begin
  Serial.begin(9600);
  Serial.println("Ultrasonic Started");
}

bool measure() {
  long sum1=0;
  long sum2=0;
  int max1=0;
  int max2=0;
  int min1=1000;
  int min2=1000;
  int cm1 = 0;
  int cm2 = 0;
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
    return true;
  }
  else {
    return false;
  }
}

int get_next_mode(int current_mode) {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'M') {
      if (measure()) {
        Serial.println("1");
        return 3;
      }
      else {
        Serial.println("0");
        return 2;
      }
    }
    else if (input == 'T') {
      return 1;
    }
    else if (input == 'W') {
      return 0;
    }
  }
  else {
    return current_mode;
  }
}
 
void loop()
{
   switch (mode) {
    case 0:
      if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds_1[i] = CRGB::Black;
          leds_2[i] = CRGB::Black;
        }
      }
      FastLED.show();
      prev_mode = mode;
      mode = get_next_mode(mode);
      break;
    case 1:
      for (int i = 0; i < NUM_LEDS; i++) {
        leds_1[i] = CRGB::White;
        leds_2[i] = CRGB::White;
        FastLED.show();
        delay(100);
        leds_1[i] = CRGB::Black;
        leds_2[i] = CRGB::Black;
      }
      prev_mode = mode;
      mode = get_next_mode(mode);
      break;
    case 2:
      if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds_1[i] = CRGB::White;
          leds_2[i] = CRGB::White;
        }
        FastLED.show();
      }
      prev_mode = mode;
      mode = get_next_mode(mode);
      break;
    case 3:
      if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds_1[i] = CRGB(250,60,0); // Light green, object order, GRB
          leds_2[i] = CRGB(250,60,0);
        }
        FastLED.show();
      }
      prev_mode = mode;
      mode = get_next_mode(mode);
   }
}
