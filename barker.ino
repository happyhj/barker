// Constants for barker
#define DEFAULT_BARK_DURATION 1000
#define DEFAULT_STRONG_BARK_DURATION 5000
#define CHAIN_BARK_DELAY 1000
#define RAISE_LEVEL_ALERT_GAP 5000 // level 1 sound file length should silghtly shorter than this
#define LEVEL_2_THRESHOLD 600 //
#define LEVEL_1_THRESHOLD 100 //
#define VOLUME_SCALE 20 //20,40,80

#define NUM_OF_SAMPLE 1 //

// Import SD card library 
#include <SD.h>
#include <SPI.h>

// Add the SdFat Libraries
#include <SdFat.h>
#include <SdFatUtil.h> 
// globals for mp3 shield
//Create the variables to be used by SdFat Library
Sd2Card card;
SdVolume volume;
SdFile root;
SdFile track;
char errorMsg[100]; //This is a generic array used for sprintf of error messages
#define TRUE  0
#define FALSE  1
#define MP3_XCS 6 //Control Chip Select Pin (for accessing SPI Control/Status registers)
#define MP3_XDCS 7 //Data Chip Select / BSYNC Pin
#define MP3_DREQ 2 //Data Request Pin: Player asks for more data
#define MP3_RESET 8 //Reset is active low
#define SCI_MODE 0x00
#define SCI_STATUS 0x01
#define SCI_BASS 0x02
#define SCI_CLOCKF 0x03
#define SCI_DECODE_TIME 0x04
#define SCI_AUDATA 0x05
#define SCI_WRAM 0x06
#define SCI_WRAMADDR 0x07
#define SCI_HDAT0 0x08
#define SCI_HDAT1 0x09
#define SCI_AIADDR 0x0A
#define SCI_VOL 0x0B
#define SCI_AICTRL0 0x0C
#define SCI_AICTRL1 0x0Dmn,  g
#define SCI_AICTRL2 0x0E  uyk,  mkfu f fv   vf  
#define SCI_AICTRL3 0x0F

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
   
  pinMode(MP3_DREQ, INPUT);
  pinMode(MP3_XCS, OUTPUT);
  pinMode(MP3_XDCS, OUTPUT);
  pinMode(MP3_RESET, OUTPUT);
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  digitalWrite(MP3_RESET, LOW); //Put VS1053 into hardware reset
  pinMode(10, OUTPUT);       //Pin 10 must be set as an output for the SD communication to work.
  
  card.init(SPI_FULL_SPEED);
  volume.init(&card);
  root.openRoot(&volume);
  
  SPI.setClockDivider(SPI_CLOCK_DIV16); //Set SPI bus speed to 1MHz (16MHz / 16 = 1MHz)
  SPI.transfer(0xFF); //Throw a dummy byte at the bus
  delay(10);
  digitalWrite(MP3_RESET, HIGH); //Bring up VS1053
  Mp3SetVolume(VOLUME_SCALE, VOLUME_SCALE); //Set initial volume (20 = -10dB) Manageable
  int MP3Mode = Mp3ReadRegister(SCI_MODE);
  int MP3Status = Mp3ReadRegister(SCI_STATUS);
  int MP3Clock = Mp3ReadRegister(SCI_CLOCKF);
  int vsVersion = (MP3Status >> 4) & 0x000F; //Mask out only the four version bits
  Mp3WriteRegister(SCI_CLOCKF, 0x60, 0x00); //Set multiplier to 3.0x
  SPI.setClockDivider(SPI_CLOCK_DIV4); //Set SPI bus speed to 4MHz (16MHz / 4 = 4MHz)
  MP3Clock = Mp3ReadRegister(SCI_CLOCKF);
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

  // getting alert Level from my neighbor 
   if( Serial.available() > 0 )
   {
     char inByte = Serial.read();
     if( inByte == '2' ) {
       Serial.println("Yatta!"); 
       neighborAlertTimestamp = currentTime;
        neighborAlertLevel = 2; 
     }
     while(Serial.available() > 0) {
       Serial.read(); 
     }
   }
  // bark 3 second after neighbor get alert Event
  // set alert level from my neighbor
  n_gap = currentTime - neighborAlertTimestamp;
  Serial.println("--*--");
  Serial.println("-"+String(neighborAlertTimestamp));
  Serial.println("-"+String(currentTime));
  Serial.println("-----");

  if( (neighborAlertTimestamp != 0) && (n_gap < 5000) &&  (n_gap> CHAIN_BARK_DELAY)) {  
      state = neighborAlertLevel;
  } 

  turnOffWhenSituationEnded(); 
  alert();
  
  delay(100);
}

void alert() {
 if(state == 1) {
    analogWrite(ledPin, 100); 
    playMP3("track001.mp3");
 } else if(state == 2) {
    analogWrite(ledPin, 255); 
    playMP3("track002.mp3");
 }
}

void turnOffWhenSituationEnded(){
  unsigned long currentTime = millis();
  if((state == 1) && (currentTime - alertTimestamp > DEFAULT_BARK_DURATION) && (alertTimestamp != 0)) {
    state = 0;
    analogWrite(ledPin,0);     
    alertTimestamp = 0;
  } else if((state == 2) && (currentTime - alertTimestamp > DEFAULT_STRONG_BARK_DURATION) && (alertTimestamp != 0)) {
    state = 0;
    analogWrite(ledPin,0);     
    alertTimestamp = 0;
  }   
  if((state == 1) && (currentTime - neighborAlertTimestamp > DEFAULT_BARK_DURATION) && (neighborAlertTimestamp != 0)){
    state = 0;
    analogWrite(ledPin,0);  
    neighborAlertTimestamp = 0;
    neighborAlertLevel = 0;
  } else if((state == 2) && (currentTime - neighborAlertTimestamp > DEFAULT_STRONG_BARK_DURATION) && (neighborAlertTimestamp != 0)){
    state = 0;
    analogWrite(ledPin,0);  
    neighborAlertTimestamp = 0;
    neighborAlertLevel = 0;
  }
}

void playMP3(char* fileName) {
  if (!track.open(&root, fileName, O_READ)) { //Open the file in read mode.
    sprintf(errorMsg, "Failed to open %s", fileName);
    return;
  }
  uint8_t mp3DataBuffer[32]; //Buffer of 32 bytes. VS1053 can take 32 bytes at a go.
  int need_data = TRUE; 
  long replenish_time = millis();
  while(1) {
    while(!digitalRead(MP3_DREQ)) { 
      if(need_data == TRUE) {
        if(!track.read(mp3DataBuffer, sizeof(mp3DataBuffer))) { //Try reading 32 new bytes of the song
          break;
        }
        need_data = FALSE;
      }
      long start_time = millis();
      delay(100); //Do NOTHING - sounds fine
      replenish_time = millis();
    }
    if(need_data == TRUE){ //This is here in case we haven't had any free time to load new data
      if(!track.read(mp3DataBuffer, sizeof(mp3DataBuffer))) { //Go out to SD card and try reading 32 new bytes of the song
        break;
      }
      need_data = FALSE;
    }
    //Once DREQ is released (high) we now feed 32 bytes of data to the VS1053 from our SD read buffer
    digitalWrite(MP3_XDCS, LOW); //Select Data
    for(int y = 0 ; y < sizeof(mp3DataBuffer) ; y++) {
      SPI.transfer(mp3DataBuffer[y]); // Send SPI byte
    }
    digitalWrite(MP3_XDCS, HIGH); //Deselect Data
    need_data = TRUE; //We've just dumped 32 bytes into VS1053 so our SD read buffer is empty. Set flag so we go get more data
  }
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating transfer is complete
  digitalWrite(MP3_XDCS, HIGH); //Deselect Data
  track.close(); //Close out this track
}

void Mp3WriteRegister(unsigned char addressbyte, unsigned char highbyte, unsigned char lowbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control
  SPI.transfer(0x02); //Write instruction
  SPI.transfer(addressbyte);
  SPI.transfer(highbyte);
  SPI.transfer(lowbyte);
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
}

unsigned int Mp3ReadRegister (unsigned char addressbyte){
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating IC is available
  digitalWrite(MP3_XCS, LOW); //Select control
  SPI.transfer(0x03);  //Read instruction
  SPI.transfer(addressbyte);
  char response1 = SPI.transfer(0xFF); //Read the first byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  char response2 = SPI.transfer(0xFF); //Read the second byte
  while(!digitalRead(MP3_DREQ)) ; //Wait for DREQ to go high indicating command is complete
  digitalWrite(MP3_XCS, HIGH); //Deselect Control
  int resultvalue = response1 << 8;
  resultvalue |= response2;
  return resultvalue;
}

void Mp3SetVolume(unsigned char leftchannel, unsigned char rightchannel){
  Mp3WriteRegister(SCI_VOL, leftchannel, rightchannel);
}

