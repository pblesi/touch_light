/*
 * Project: touch_light
 * Description: A touch light that syncs with other touch lights. Adapted from
 *              http://www.instructables.com/id/Networked-RGB-Wi-Fi-Decorative-Touch-Lights/
 * Author: Patrick Blesi
 * Date: 2017-12-09
 *
 */

#include "neopixel.h"
#include "application.h"

// CONFIGURATION SETTINGS START
// DEBUG SETTINGS:
#define D_SERIAL false
#define D_WIFI true

#define NUM_PARTICLES 1 // number of touch lights in your group
// Number each Filimin starting at 1.
String particleId[] = {
  "",                            // 0
  "330022001547353236343033"     // Filimin 1
  // "330022001547353236343033", // Filimin 2
  // "330022001547353236343033", // Filimin 3
  // "330022001547353236343033", // Filimin 4
  // "330022001547353236343033", // Filimin 5
  // "330022001547353236343033"  // Filimin 6
};

// TWEAKABLE VALUES FOR CAP SENSING. THE BELOW VALUES WORK WELL AS A STARTING PLACE:
// BASELINE_VARIANCE: The higher the number the less the baseline is affected by current readings. (was 4)
#define BASELINE_VARIANCE 512.0
// SENSITIVITY: Integer. Higher is more sensitive (was 8)
#define SENSITIVITY 8
// BASELINE_SENSITIVITY: Integer. A trigger point such that values exceeding this point will not affect the baseline. Higher values make the trigger point sooner. (was 16)
#define BASELINE_SENSITIVITY 16
// SAMPLE_SIZE: Number of samples to take for one reading. Higher is more accurate but large values cause some latency.(was 32)
#define SAMPLE_SIZE 512
#define SAMPLES_BETWEEN_PIXEL_UPDATES 32
#define LOOPS_TO_FINAL_COLOR 150

const int minMaxColorDiffs[2][2] = {
  {5,20},   // min/Max if color change last color change from same touch light
  {50,128}  // min/Max if color change last color change from different touch light
};

// CONFIGURATION SETTINGS END

int sPin = D4;
int rPin = D3;

// STATES:
#define PRE_ATTACK 0
#define ATTACK 1
#define DECAY 2
#define SUSTAIN 3
#define RELEASE1 4
#define RELEASE2 5
#define OFF 6

#define END_VALUE 0
#define TIME 1

// END VALUE, TIME
// 160 is approximately 1 second
const long envelopes[6][2] = {
  {0, 0},      // OFF
  {255, 30} ,  // ATTACK
  {200, 240},  // DECAY
  {200, 1000}, // SUSTAIN
  {150, 60},   // RELEASE1
  {0, 1000000} // RELEASE2 (65535 is about 6'45")
};

// NEOPIXEL
#define PIXEL_PIN D2
#define PIXEL_COUNT 24
#define PIXEL_TYPE WS2812B

#define tEVENT_NONE 0
#define tEVENT_TOUCH 1
#define tEVENT_RELEASE 2

String eventTypes[] = {
  "None",
  "Touch",
  "Release"
};


Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

// STATE
unsigned char myId = 0;

int currentEvent = tEVENT_NONE;

int initColor = 0;
int currentColor = 0;   // 0 to 255
int finalColor = 0;     // 0 to 255

int initBrightness = 0; // 0 to 255
int brightness = 0;     // 0 to 255

int brightnessDistanceToNextState = 0;
int colorChangeToNextState = 0;

unsigned char prevState = OFF;
unsigned char state = OFF;

unsigned char lastColorChangeDeviceId = 0;

long loopCount = 0;
long colorLoopCount = 0;

double tDelayExternal = 0;
double tBaselineExternal = 0;

uint8_t shouldUpdateServer = tEVENT_NONE;

// timestamps
unsigned long tS;
volatile unsigned long tR;

// reading and baseline
float tBaseline;

void setup()
{
  Particle.function("poll", pollLamp);

  if (D_SERIAL) Serial.begin(9600);
  if (D_WIFI) {
    Particle.variable("tDelay", &tDelayExternal, DOUBLE);
    Particle.variable("tBaseline", &tBaselineExternal, DOUBLE);
  }

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(sPin, OUTPUT);
  attachInterrupt(rPin, touchSense, RISING);

  myId = getMyId(particleId, NUM_PARTICLES);

  flashWhite(&strip);

  // Calibrate touch sensor- Keep hands off!!!
  tBaseline = touchSampling(); // initialize to first reading
  if (D_WIFI) tBaselineExternal = tBaseline;

  traverseColorWheel(&strip);
  fade(&strip);
}

void loop() {
  int touchEvent = touchEventCheck();
  if (touchEvent == tEVENT_NONE) return;

  currentEvent = touchEvent;
  if (D_SERIAL) Serial.println(eventTypes[touchEvent]);
  if(touchEvent == tEVENT_TOUCH) state = PRE_ATTACK;
}

//============================================================
//	Setup functions
//============================================================
//------------------------------------------------------------
// Functions used during setup
//------------------------------------------------------------
int getMyId(String particleId[], int numParticles) {
  for (int i = 1; i <= numParticles; i++) {
    if (!particleId[i].compareTo(Particle.deviceID())) {
      return i;
    }
  }
}

void flashWhite(Adafruit_NeoPixel* strip) {
  int numPixels = strip->numPixels();
  for (byte j = 0; j < numPixels; j++) {
    strip->setPixelColor(j, 255, 255, 255);
  }
  strip->show();
  delay(250);
  for (byte j = 0; j < numPixels; j++) {
    strip->setPixelColor(j, 0, 0, 0);
  }
  strip->show();
  delay(250);
}

void traverseColorWheel(Adafruit_NeoPixel* strip) {
  int numPixels = strip->numPixels();
  for (int i = 0; i < 256; i++) {
    uint32_t color = wheelColor(i, 255);
    for (byte j = 0; j < numPixels; j++) {
      strip->setPixelColor(j, color);
      strip->show();
    }
    delay(1);
  }
}

void fade(Adafruit_NeoPixel* strip) {
  int numPixels = strip->numPixels();
  for (int j = 255; j >= 0; j--) {
    uint32_t color = wheelColor(255, j);
    for (byte k = 0; k < numPixels; k++) {
      strip->setPixelColor(k, color);
      strip->show();
    }
    delay(1);
  }
}

//============================================================
//	Touch UI
//============================================================
//------------------------------------------------------------
// ISR for touch sensing
//------------------------------------------------------------
void touchSense() {
  tR = micros();
}

//------------------------------------------------------------
// touch sampling
//
// sample touch sensor SAMPLE_SIZE times and get average RC delay [usec]
//------------------------------------------------------------
long touchSampling() {
  long tDelay = 0;
  int mSample = 0;

  for (int i = 0; i < SAMPLE_SIZE; i++) {
    if (!(i % SAMPLES_BETWEEN_PIXEL_UPDATES)) {
      stateAndPixelMagic();
    }
    pinMode(rPin, OUTPUT); // discharge capacitance at rPin
    digitalWrite(sPin,LOW);
    digitalWrite(rPin,LOW);
    pinMode(rPin,INPUT); // revert to high impedance input
    // timestamp & transition sPin to HIGH and wait for interrupt in a read loop
    tS = micros();
    tR = tS;
    digitalWrite(sPin,HIGH);
    do {
      // wait for transition
    } while (digitalRead(rPin)==LOW);

    // accumulate the RC delay samples
    // ignore readings when micros() overflows
    if (tR > tS) {
      tDelay = tDelay + (tR - tS);
      mSample++;
    }
  }

  // calculate average RC delay [usec]
  if ((tDelay > 0) && (mSample > 0)) {
    tDelay = tDelay/mSample;
  } else {
    tDelay = 0;     // this is an error condition!
  }
  if (D_SERIAL) Serial.println(tDelay);
  if (D_WIFI) tDelayExternal = tDelay;
  // autocalibration using exponential moving average on data below specified point
  if (tDelay < (tBaseline + tBaseline/BASELINE_SENSITIVITY)) {
    tBaseline = tBaseline + (tDelay - tBaseline)/BASELINE_VARIANCE;
    if (D_WIFI) tBaselineExternal = tBaseline;
  }
  return tDelay;
}

//------------------------------------------------------------
// touch event check
//
// check touch sensor for events:
//      tEVENT_NONE     no change
//      tEVENT_TOUCH    sensor is touched (Low to High)
//      tEVENT_RELEASE  sensor is released (High to Low)
//
//------------------------------------------------------------
int touchEventCheck() {
  int touchSense;                     // current reading
  static int touchSenseLast = LOW;    // last reading

  static unsigned long touchDebounceTimeLast = 0; // debounce timer
  int touchDebounceTime = 50;                     // debounce time

  static int touchNow = LOW;  // current debounced state
  static int touchLast = LOW; // last debounced state

  int tEvent = tEVENT_NONE;   // default event

  // read touch sensor
  long tReading = touchSampling();

  // touch sensor is HIGH if trigger point some threshold above Baseline
  if (tReading > (tBaseline + tBaseline / SENSITIVITY)) {
    touchSense = HIGH;
  } else {
    touchSense = LOW;
  }

  // debounce touch sensor
  // if state changed then reset debounce timer
  if (touchSense != touchSenseLast) {
    touchDebounceTimeLast = millis();
  }
  touchSenseLast = touchSense;

  // accept as a stable sensor reading if the debounce time is exceeded without reset
  if (millis() > touchDebounceTimeLast + touchDebounceTime) {
    touchNow = touchSense;
  }

  // set events based on transitions between readings
  if (!touchLast && touchNow) {
    tEvent = tEVENT_TOUCH;
  }

  if (touchLast && !touchNow) {
    tEvent = tEVENT_RELEASE;
  }

  // update last reading
  touchLast = touchNow;
  return tEvent;
}

int pollLamp(String command) {
  static int lastServerColorAndTouch = 0;
  static bool serverShouldBeUpdated = false;
  static unsigned char serverAndLocalDifferentLoopCount = 0;

  static uint32_t lastLocalResponse;
  uint32_t localResponse;

  int lastServerColor = lastServerColorAndTouch & 0xff;

  uint32_t serverResponse = command.toInt();
  unsigned char deviceId = serverResponse >> 10;
  int serverColorAndTouch = serverResponse & 0x03ff; // color and touch
  int serverColor = serverColorAndTouch & 0xff;
  int serverTouch = serverColorAndTouch >> 8;

  if (D_SERIAL) {
    Serial.print("received id: ");
    Serial.print(deviceId);
    Serial.print(" cmd: ");
    Serial.print(serverTouch);
    Serial.print(" color: ");
    Serial.println(serverColorAndTouch & 0xff);
  }

  /*
  if (serverColor == lastServerColor && serverShouldBeUpdated && false) {
    if (D_SERIAL) Serial.println("* SERVER LOST LAST UPDATE. RESENDING");
    return lastLocalResponse;
  }
  */

  if (
    (serverColorAndTouch != lastServerColorAndTouch) && (deviceId != myId) && (deviceId != 0)
  ) { // server update
    serverAndLocalDifferentLoopCount = 0;
    if (D_SERIAL) Serial.println("SERVER -> LOCAL");
    // if the color changed but we have a release, we missed an event.
    // and if so, create the missing event
    // TODO: a queue for events would be a better solution but hey, it's Dec 24.
    if (serverColor != lastServerColor && serverTouch == tEVENT_RELEASE) {
        serverTouch = tEVENT_TOUCH;
        serverColorAndTouch = (serverTouch << 8) + (serverColorAndTouch & 0xff);
        if (D_SERIAL) Serial.println("*** missing server touch event created ***");
    }

    if (shouldUpdateServer) { // local update
      if (D_SERIAL) { Serial.print("----& LOCAL -> SERVER: "); Serial.println(shouldUpdateServer); }
      if (shouldUpdateServer == tEVENT_TOUCH) {
        if (D_SERIAL) Serial.println("---- local touch");
        if (serverTouch == tEVENT_TOUCH) {
          if (D_SERIAL) Serial.println("---- server touch");
          lastColorChangeDeviceId = deviceId;
          generateColor();
          changeState(ATTACK);
        } else {
          if (D_SERIAL) Serial.println("---- server release");
          generateColor();
        }
        // serverTouch == touch or release:
        localResponse = (myId << 10) + (shouldUpdateServer << 8) + (finalColor & 0xff);
      } else { // local touchEvent == tEVENT_RELEASE
        if (D_SERIAL) Serial.println("---- local release");
        if (serverTouch == tEVENT_TOUCH) {
          if (D_SERIAL) Serial.println("---- server touch");
          getColorFromServer(serverColor, deviceId);
          changeState(ATTACK);
        } else { // server and local have tEVENT_RELEASE
          if (D_SERIAL) Serial.println("---- server release");
          changeState(RELEASE1);
        }
        localResponse = serverResponse;
      }
    } else { // no local update
      if (D_SERIAL) Serial.println("-no local update");
      if (serverTouch == tEVENT_TOUCH) {
        if (D_SERIAL) Serial.println("--server touch");
        getColorFromServer(serverColor, deviceId);
        changeState(ATTACK);
      } else {
        if (D_SERIAL) Serial.println("--server release");
        changeState(RELEASE1);
      }
      localResponse = serverResponse;
    }
  } else { // no server update
    if (shouldUpdateServer || serverShouldBeUpdated) {
      if (serverShouldBeUpdated) shouldUpdateServer = tEVENT_TOUCH;
      serverShouldBeUpdated = false;
      serverAndLocalDifferentLoopCount = 0;
      if (D_SERIAL) Serial.println("LOCAL -> SERVER");
      localResponse = (myId << 10) + (shouldUpdateServer << 8) + (finalColor & 0xff);
    } else { // no updates at all
      if (((finalColor & 0xff) != (serverColorAndTouch & 0xff)) && (deviceId != 0)) {
        if (++serverAndLocalDifferentLoopCount > 2) {
          //lastColorChangeDeviceId = myId;
          serverShouldBeUpdated = true;
          if (D_SERIAL) { Serial.println("*** forcing server update"); }
        }
      }
    }
    localResponse = serverResponse;
  }

  if (D_SERIAL) {
    Serial.print("sending id: ");
    Serial.print(localResponse >> 10 );
    Serial.print(" cmd: ");
    Serial.print(localResponse >> 8 & 0x03);
    Serial.print(" color: ");
    Serial.println(localResponse & 0xff);
  }
  
  if (shouldUpdateServer && localResponse != serverResponse) {
    lastLocalResponse = localResponse;
    //serverShouldBeUpdated = true;
  }

  shouldUpdateServer = tEVENT_NONE;
  lastServerColorAndTouch = (serverColorAndTouch == 0 ? lastServerColorAndTouch : serverColorAndTouch);
  return localResponse;
}

void getColorFromServer(int color, unsigned char deviceId) {
  colorLoopCount = 0;
  initColor = currentColor;
  finalColor = color;
  colorChangeToNextState = calcColorChange(initColor, finalColor);
  lastColorChangeDeviceId = deviceId;
  if (D_SERIAL) {
    Serial.print("get Color From Server Final color: ");
    Serial.print(finalColor);
    Serial.print(", ");
    Serial.println(colorChangeToNextState);
  }
}

void generateColor() {
  Serial.println("generating color...");
  initColor = currentColor;
  if (prevState == OFF) {
    initColor = currentColor = finalColor = random(256);
  } else {
    bool foreignId = (lastColorChangeDeviceId != myId);
    int minChange = minMaxColorDiffs[foreignId][0];
    int maxChange = minMaxColorDiffs[foreignId][1];
    int direction = random(2) * 2 - 1;
    int magnitude = random(minChange, maxChange + 1);
    finalColor += direction * magnitude;
    finalColor = (finalColor + 256) % 256;
    // finalColor = 119; // FORCE A COLOR
  }
  colorChangeToNextState = calcColorChange(initColor, finalColor);
  lastColorChangeDeviceId = myId;
  if (D_SERIAL) { Serial.print("final color: "); Serial.println(finalColor); }
}

int calcColorChange(int currentColor, int finalColor) {
  int colorChange = finalColor - currentColor;
  int direction = (colorChange < 0) * 2 - 1;
  colorChange += direction * (abs(colorChange) > 127) * 256;
  return colorChange;
}

void changeState(unsigned char newState) {
  if (D_SERIAL) { Serial.print("state: "); Serial.println(newState); }
  prevState = state;
  state = newState;
  loopCount = 0;
  initBrightness = brightness;
  brightnessDistanceToNextState = envelopes[newState][END_VALUE] - brightness;
}

void stateAndPixelMagic() {
  switch (state) {
    case PRE_ATTACK:
      generateColor();
      shouldUpdateServer = tEVENT_TOUCH;
      changeState(ATTACK);
      colorLoopCount = 0;
      break;
    case ATTACK:
      updatePixelSettings();
      currentEvent = tEVENT_NONE;
      if (loopCount >= envelopes[ATTACK][TIME]) {
        changeState(DECAY);
      }
      break;
    case DECAY:
      updatePixelSettings();
      if ((loopCount >= envelopes[DECAY][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(SUSTAIN);
      }
      break;
    case SUSTAIN:
      updatePixelSettings();
      if ((loopCount >= envelopes[SUSTAIN][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(RELEASE1);
        shouldUpdateServer = tEVENT_RELEASE;
        currentEvent = tEVENT_NONE;
      }
      break;
    case RELEASE1:
      updatePixelSettings();
      if (loopCount >= envelopes[RELEASE1][TIME]) {
        changeState(RELEASE2);
      }
      break;
    case RELEASE2:
      updatePixelSettings();
      if (loopCount >= envelopes[RELEASE2][TIME]) {
        changeState(OFF);
      }
      break;
    case OFF:
      brightness = 0;
      break;
  }

  uint32_t colorAndBrightness = wheelColor(currentColor, brightness);
  updateNeoPixels(colorAndBrightness);
  loopCount++;
  colorLoopCount++;
}

void updatePixelSettings() {
  brightness = min(255, max(0, initBrightness + loopCount * brightnessDistanceToNextState / envelopes[state][TIME]));

  if (colorLoopCount > LOOPS_TO_FINAL_COLOR) {
    currentColor = finalColor;
  }

  if (currentColor != finalColor) {
    currentColor = (initColor + 256 + colorLoopCount * colorChangeToNextState / LOOPS_TO_FINAL_COLOR ) % 256;
  }
}

void updateNeoPixels(uint32_t color) {
  for(char i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

//============================================================
//	NEOPIXEL
//============================================================
//------------------------------------------------------------
// Wheel
//------------------------------------------------------------

uint32_t wheelColor(byte WheelPos, byte iBrightness) {
  float R, G, B;
  float brightness = iBrightness / 255.0;

  if (WheelPos < 85) {
    R = WheelPos * 3;
    G = 255 - WheelPos * 3;
    B = 0;
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    R = 255 - WheelPos * 3;
    G = 0;
    B = WheelPos * 3;
  } else {
    WheelPos -= 170;
    R = 0;
    G = WheelPos * 3;
    B = 255 - WheelPos * 3;
  }
  R = R * brightness + .5;
  G = G * brightness + .5;
  B = B * brightness + .5;
  return strip.Color((byte) R,(byte) G,(byte) B);
}
