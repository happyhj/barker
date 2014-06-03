// SENSOR UNIT
// Constants for barker
#define DEFAULT_BARK_DURATION 1000
#define DEFAULT_STRONG_BARK_DURATION 5000
#define CHAIN_BARK_DELAY 1000
#define RAISE_LEVEL_ALERT_GAP 6000 // level 1 sound file length should silghtly shorter than this
#define LEVEL_2_THRESHOLD 400 //
#define LEVEL_1_THRESHOLD 60 //
#define VOLUME_SCALE 20 //20,40,80

#define NUM_OF_SAMPLE 1 //

#include <SPI.h>

// Globals for barker
int sensorPin = 2;
int ledPin = 9;
int state = 0; // 0: ideal,1: alert, 2: strong alert
int neighborAlertLevel = 0;
int values[NUM_OF_SAMPLE] = {0};
int average = 0;
unsigned long alertTimestamp = 0;
unsigned long neighborAlertTimestamp = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sum = 0,
      i= 0, val= 0, delta = 0;
  unsigned long currentTime = millis(),
  alertGap = 0,
  n_gap = 0;

  alertGap =abs(currentTime - alertTimestamp);
    
  val = analogRead(sensorPin);
  val = map(val, 1, 50, 0, 100);
if(alertGap > RAISE_LEVEL_ALERT_GAP) {  
  // prepare values from my sensor 
  for(i=0;i<NUM_OF_SAMPLE;i++){
    sum = sum + values[i];
  }
   
  if(values[0] != 0) {
    if( values[0] - val > 0 ) {
     delta = 0; 
    } else {
      delta = val - values[0];
    }
  } else {
    delta = 0;
  }
}
  if(alertGap > RAISE_LEVEL_ALERT_GAP) {
  if( delta > LEVEL_1_THRESHOLD )   {
    // update Timer 
    state = 1;
    
    // Raise Level by Intensity
    if(delta > LEVEL_2_THRESHOLD) {
      state = 2;
    }
   // set alert level 2 from my alertTimestamp
   alertGap =abs(currentTime - alertTimestamp);
   // Raise Level by alertGap
   if( ((state == 1) || (state == 2)) && (alertGap < RAISE_LEVEL_ALERT_GAP) && (alertGap != alertTimestamp)) {
      // update Timer 
      state = 2;
    }
    alertTimestamp = currentTime;
    Serial.println(state);  
  } 
  }
  // store values
  // shift-left values
    for(i=1; i<NUM_OF_SAMPLE; i++) { 
      values[i-1] = values[i]; 
    }
    values[NUM_OF_SAMPLE-1] = val;

  // bark 3 second after neighbor get alert Event
  // set alert level from my neighbor
  n_gap = currentTime - neighborAlertTimestamp;

  if( (neighborAlertTimestamp != 0) && (n_gap < 5000) &&  (n_gap> CHAIN_BARK_DELAY)) {  
      state = neighborAlertLevel;
  } 

  turnOffWhenSituationEnded(); 
  
  delay(100);
}


void turnOffWhenSituationEnded(){
  unsigned long currentTime = millis();
  if((state == 1) && (currentTime - alertTimestamp > DEFAULT_BARK_DURATION) && (alertTimestamp != 0)) {
    state = 0;
    alertTimestamp = 0;
  } else if((state == 2) && (currentTime - alertTimestamp > DEFAULT_STRONG_BARK_DURATION) && (alertTimestamp != 0)) {
    state = 0;
    alertTimestamp = 0;
  }   
  if((state == 1) && (currentTime - neighborAlertTimestamp > DEFAULT_BARK_DURATION) && (neighborAlertTimestamp != 0)){
    state = 0;
    neighborAlertTimestamp = 0;
    neighborAlertLevel = 0;
  } else if((state == 2) && (currentTime - neighborAlertTimestamp > DEFAULT_STRONG_BARK_DURATION) && (neighborAlertTimestamp != 0)){
    state = 0;
    neighborAlertTimestamp = 0;
    neighborAlertLevel = 0;
  }
}


