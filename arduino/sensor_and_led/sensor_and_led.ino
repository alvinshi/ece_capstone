#include <Ultrasonic.h>
#include <FastLED.h>


enum state {
  WAIT,
  TRACK,
  APPROACH,
  DETECTED,
  PLAYER_SEARCH,
  GO_PLAYER,
  RETURN,
  INIT,
  CAPTURE_FAILURE
};

// 0 wait, 1 track, 2 approach, 3 detected
state prev_mode = INIT;
state mode = WAIT;

// Ultrasonic Sensor Setup
int trigPin1 = 9;
int echoPin1 = 10;
int trigPin2 = 11;
int echoPin2 = 12; 
Ultrasonic ultrasonic1(trigPin1, echoPin1, 40000UL); //Using a 40ms timeout should give you a maximum range of approximately 6.8m.
Ultrasonic ultrasonic2(trigPin2, echoPin2, 40000UL);

bool player_search_light_on = false;

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

state get_next_mode(state current_mode) {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'M') {
      if (measure()) {
        Serial.println("1");
        return DETECTED;
      }
      else {
        Serial.println("0");
        return APPROACH;
      }
    }
    else if (input == 'V') {
      if (measure()) {
        Serial.println("1");
        return DETECTED;
      }
      else {
        Serial.println("0");
        return CAPTURE_FAILURE;
      }
    }
    else if (input == 'T') {
      return TRACK;
    }
    else if (input == 'W') {
      return WAIT;
    }
    else if (input == 'P') {
      return PLAYER_SEARCH;
    }
    else if (input == 'G') {
      return GO_PLAYER;
    }
    else if (input == 'R') {
      return RETURN;
    }
    else {
      // should not happen
      return current_mode;
    }
  }
  else {
    return current_mode;
  }
}
 
void loop()
{
   switch (mode) {
    case WAIT: // Wait state, lights off
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
    case TRACK: // Track state, flowing white lights
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
    case APPROACH: // Approach state, white lights fully on
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
    case DETECTED: // Detect state, green lights fully on
      if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds_1[i] = CRGB(250,60,0); // Light green, object order, GRB
          leds_2[i] = CRGB(250,60,0);
        }
        FastLED.show();
      }
      prev_mode = mode;
      mode = get_next_mode(mode);
      break;
   case CAPTURE_FAILURE:
     if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
            leds_1[i] = CRGB::Red;
            leds_2[i] = CRGB::Red;
        }
        FastLED.show();
     }
     prev_mode = mode;
     mode = get_next_mode(mode);
     break;
   case PLAYER_SEARCH: // Player search state, flashing white lights
     if (player_search_light_on) {
       for (int i = 0; i < NUM_LEDS; i++) {
         leds_1[i] = CRGB::Black;
         leds_2[i] = CRGB::Black;
       }
       FastLED.show();
       player_search_light_on = false;
     }
     else {
       for (int i = 0; i < NUM_LEDS; i++) {
         leds_1[i] = CRGB::White;
         leds_2[i] = CRGB::White;
       }
       FastLED.show();
       player_search_light_on = true;
     }
     delay(300);
     prev_mode = mode;
     mode = get_next_mode(mode);
     break;
  case GO_PLAYER: // Player Approach state, flowing white lights
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
  case RETURN: // return state, blue lights fully on
      if (mode != prev_mode) {
        for (int i = 0; i < NUM_LEDS; i++) {
          leds_1[i] = CRGB::Blue;
          leds_2[i] = CRGB::Blue;
        }
        FastLED.show();
      }
      prev_mode = mode;
      mode = get_next_mode(mode);
   }
}
