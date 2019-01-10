/*
 * Project: touch_light
 * Description: A touch light that syncs with other touch lights. Adapted from
 *              http://www.instructables.com/id/Networked-RGB-Wi-Fi-Decorative-Touch-Lights/
 * Original Author: Patrick Blesi
 * Date: 2017-12-09
 *
 * Modifications By: Jeff Bush (coderforlife)
 * Date: 2018-12-21
 */

#include "neopixel.h"
#include "application.h"

#include "wifi_creds.h"

// CONFIGURATION SETTINGS START
// DEBUG SETTINGS:
#define D_SERIAL false
#define D_WIFI false

String touchEventName = "touch_event";

// Number each starting at 1
String particleId[] = {
  "",                         // 0
  "330022001547353236343033", // pblesi
  "2d0047001247353236343033", // carol
  "2a0026000b47353235303037", // cindy
  "2e003e001947353236343033"  // tammy
};

int particleColors[] = {
  0,   // Green
  90,  // Magenta
  170, // Blue
  79,  // Orange
  131  // Purple
};


#define ARRAY_SIZE(a) sizeof(a)/sizeof(a[0])

// TWEAKABLE VALUES FOR CAP SENSING. THE BELOW VALUES WORK WELL AS A STARTING PLACE:
// BASELINE_VARIANCE: The higher the number the less the baseline is affected by
// current readings. (was 4)
#define BASELINE_VARIANCE 512.0
// SENSITIVITY: Integer. Higher is more sensitive (was 8)
#define SENSITIVITY 8
// BASELINE_SENSITIVITY: Integer. A trigger point such that values exceeding this point
// will not affect the baseline. Higher values make the trigger point sooner. (was 16)
#define BASELINE_SENSITIVITY 16
// SAMPLE_SIZE: Number of samples to take for one reading. Higher is more accurate
// but large values cause some latency.(was 32)
#define SAMPLE_SIZE 512
#define SAMPLES_BETWEEN_PIXEL_UPDATES 32
#define LOOPS_TO_FINAL_COLOR 150

const int minMaxColorDiffs[2][2] = {
  {5,20},   // min/Max if color change last color change from same touch light
  {50,128}  // min/Max if color change last color change from different touch light
};

// END VALUE, TIME
// 160 is approximately 1 second
const long envelopes[8][2] = {
  {0, 0},       // NOT USED
  {255, 30},    // ATTACK   ~200 ms
  {205, 240},   // DECAY    ~1.5 sec
  {205, 1000},  // SUSTAIN  ~6.25 sec
  {155, 60},    // RELEASE1 ~400 ms to go from ~80% brightness to ~60% brightness
  {40, 300000}, // RELEASE2 ~30 min to go from ~60% brightness to ~15% brightness
  {10, 300000}, // RELEASE3 ~30 min to go from ~15% brightness to ~4% brightness
  {10, 0},      // HOLD     will be held at ~4% brightness forever
};

#define PERIODIC_UPDATE_TIME 5 // seconds
#define COLOR_CHANGE_WINDOW 10 // seconds

// CONFIGURATION SETTINGS END

// STATES:
#define ATTACK 1
#define DECAY 2
#define SUSTAIN 3
#define RELEASE1 4
#define RELEASE2 5
#define RELEASE3 6
#define HOLD 7
#define OFF 8

#define LOCAL_CHANGE 0
#define REMOTE_CHANGE 1

#define END_VALUE 0
#define TIME 1

#define tEVENT_NONE 0
#define tEVENT_TOUCH 1
#define tEVENT_RELEASE 2

String eventTypes[] = {
  "None",
  "Touch",
  "Release"
};

int sPin = D4;
int rPin = D3;

// NEOPIXEL
#define PIXEL_PIN D2
#define PIXEL_COUNT 24
#define PIXEL_TYPE WS2812B

// STATE
unsigned char myId = 0;

int currentEvent = tEVENT_NONE;
int eventTime = 0;
int eventTimePrecision = random(INT_MAX);

int initColor = 0;
int currentColor = 0; // 0 to 255
int finalColor = 0;   // 0 to 255
int lastLocalColorChangeTime = Time.now();

int initBrightness = 0;    // 0 to 255
int currentBrightness = 0; // 0 to 255

unsigned char prevState = OFF;
unsigned char state = OFF;

unsigned char lastColorChangeDeviceId = 1;

long loopCount = 0;
long colorLoopCount = 0;
int lastPeriodicUpdate = Time.now();

// timestamps
unsigned long tS;
volatile unsigned long tR;

// reading and baseline
float tBaseline;

double tDelayExternal = 0;
double tBaselineExternal = 0;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

#ifdef WIFI_CREDENTIALS_SPECIFIED
SYSTEM_MODE(SEMI_AUTOMATIC);
#endif

void setup()
{
#ifdef WIFI_CREDENTIALS_SPECIFIED
  setupWifi();
#endif

  Particle.subscribe(touchEventName, handleTouchEvent, MY_DEVICES);

  if (D_SERIAL) Serial.begin(9600);
  if (D_WIFI) {
    Particle.variable("tDelay", &tDelayExternal, DOUBLE);
    Particle.variable("tBaseline", &tBaselineExternal, DOUBLE);
  }

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  pinMode(sPin, OUTPUT);
  attachInterrupt(rPin, touchSense, RISING);

  myId = getMyId(particleId, ARRAY_SIZE(particleId));

  flashWhite(&strip);

  // Calibrate touch sensor- Keep hands off!!!
  tBaseline = touchSampling(); // initialize to first reading
  if (D_WIFI) tBaselineExternal = tBaseline;

  traverseColorWheel(&strip);
  fade(&strip);
}

void loop() {
  int touchEvent = touchEventCheck();

  if (touchEvent == tEVENT_NONE) {
    // Publish periodic updates to synchronize state
    bool touchedBefore = currentEvent != tEVENT_NONE;
    if (lastPeriodicUpdate < Time.now() - PERIODIC_UPDATE_TIME && touchedBefore) {
      publishTouchEvent(currentEvent, finalColor, eventTime, eventTimePrecision);
      lastPeriodicUpdate = Time.now();
    }
    return;
  }

  // Random eventTimePrecision prevents ties with other
  // server events. This allows us to determine dominant
  // color in the event of ties.
  setEvent(touchEvent, Time.now(), random(INT_MAX));

  if (D_SERIAL) Serial.println(eventTypes[touchEvent]);
  if (touchEvent == tEVENT_TOUCH) {
    int newColor = generateColor(finalColor, prevState, lastColorChangeDeviceId);
    setColor(newColor, prevState, myId);
    changeState(ATTACK, LOCAL_CHANGE);
  }
}

//============================================================
//	Setup functions
//============================================================
//------------------------------------------------------------
// Functions used during setup
//------------------------------------------------------------
void setupWifi() {
  WiFi.on();
  WiFi.disconnect();
  WiFi.clearCredentials();
  int numWifiCreds = sizeof(wifiCreds) / sizeof(*wifiCreds);
  for (int i = 0; i < numWifiCreds; i++) {
    credentials creds = wifiCreds[i];
    WiFi.setCredentials(creds.ssid, creds.password, creds.authType, creds.cipher);
  }
  WiFi.connect();
  waitUntil(WiFi.ready);
  Particle.connect();
}

int getMyId(String particleId[], int numParticles) {
  int id = 0;
  for (int i = 1; i <= numParticles; i++) {
    if (!particleId[i].compareTo(Particle.deviceID())) {
      id = i;
      break;
    }
  }
  return id;
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
  }
}

void fade(Adafruit_NeoPixel* strip) {
  int numPixels = strip->numPixels();
  for (int j = 255; j >= 0; j-=2) {
    uint32_t color = wheelColor(255, j);
    for (byte k = 0; k < numPixels; k++) {
      strip->setPixelColor(k, color);
      strip->show();
    }
  }
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
      updateState();
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

void setEvent(int event, int timeOfEvent, int timePrecision) {
  currentEvent = event;
  eventTime = timeOfEvent;
  eventTimePrecision = timePrecision;
}

void handleTouchEvent(const char *event, const char *data) {
  String eventData = String(data);
  int deviceIdEnd = eventData.indexOf(',');
  int deviceId = eventData.substring(0, deviceIdEnd).toInt();
  int eventEnd = eventData.indexOf(',', deviceIdEnd + 1);
  int serverEvent = eventData.substring(deviceIdEnd + 1, eventEnd).toInt();
  int colorEnd = eventData.indexOf(',', eventEnd + 1);
  int serverColor = eventData.substring(eventEnd + 1, colorEnd).toInt();
  int eventTimeEnd = eventData.indexOf(',', colorEnd + 1);
  int serverEventTime = eventData.substring(colorEnd + 1, eventTimeEnd).toInt();
  int serverEventTimePrecision = eventData.substring(eventTimeEnd + 1).toInt();

  if (false) {
    String response = "deviceId: " + String(deviceId) + " " +
                      "serverEvent: " + String(serverEvent) + " " +
                      "serverColor: " + String(serverColor) + " " +
                      "localEventTime: " + String(eventTime) + " " +
                      "serverEventTime: " + String(serverEventTime) + " " +
                      "serverEventTimePrecision: " + String(serverEventTimePrecision);
    Particle.publish("touch_response", response, 61, PRIVATE);
  }

  if (deviceId == myId) { return; }
  if (serverEventTime < eventTime) { return; }
  // Race condition brought colors out of sync
  if (
    serverEventTime == eventTime &&
    serverEventTimePrecision == eventTimePrecision &&
    serverColor != finalColor &&
    myId < deviceId
  ) {
    setColor(serverColor, prevState, deviceId);
    changeState(ATTACK, REMOTE_CHANGE);
    return;
  }
  if (serverEventTime == eventTime && serverEventTimePrecision <= eventTimePrecision) { return; }

  // Valid remote update
  setEvent(serverEvent, serverEventTime, serverEventTimePrecision);

  if (serverEvent == tEVENT_TOUCH) {
    setColor(serverColor, prevState, deviceId);
    changeState(ATTACK, REMOTE_CHANGE);
  } else {
    changeState(RELEASE1, REMOTE_CHANGE);
  }
}

void setColor(int color, unsigned char prevState, unsigned char deviceId) {
  lastColorChangeDeviceId = deviceId;
  if (prevState == OFF) currentColor = color;
  initColor = currentColor;
  finalColor = color;
  colorLoopCount = 0;
  if (D_SERIAL) {
    Serial.print("get Color From Server Final color: ");
    Serial.print(initColor);
    Serial.print(", ");
    Serial.print(finalColor);
    Serial.print(", ");
  }
}

int generateColor(int currentFinalColor, unsigned char prevState, int lastColorChangeDeviceId) {
  int color = 0;
  int now = Time.now();
  Serial.println("generating color...");
  if (prevState == OFF || lastLocalColorChangeTime < now - COLOR_CHANGE_WINDOW) {
    color = particleColors[myId];
  } else {
    bool foreignId = (lastColorChangeDeviceId != myId);
    int minChange = minMaxColorDiffs[foreignId][0];
    int maxChange = minMaxColorDiffs[foreignId][1];
    int direction = random(2) * 2 - 1;
    int magnitude = random(minChange, maxChange + 1);
    color = currentFinalColor + direction * magnitude;
    color = (color + 256) % 256;
    // color = 119; // FORCE A COLOR
  }
  lastLocalColorChangeTime = now;
  if (D_SERIAL) { Serial.print("final color: "); Serial.println(finalColor); }
  return color;
}

void changeState(unsigned char newState, int remoteChange) {
  prevState = state;
  state = newState;
  initBrightness = currentBrightness;
  loopCount = 0;
  if (D_SERIAL) { Serial.print("state: "); Serial.println(newState); }

  if (remoteChange) return;

  if (newState == ATTACK || newState == RELEASE1) {
    publishTouchEvent(currentEvent, finalColor, eventTime, eventTimePrecision);
  }
}

void publishTouchEvent(int event, int color, int time, int timePrecision) {
  String response = String(myId)  + "," +
                    String(event) + "," +
                    String(color) + "," +
                    String(time)  + "," +
                    String(timePrecision);
  Particle.publish(touchEventName, response, 60, PRIVATE);
}

void updateState() {
  switch (state) {
    case ATTACK:
      if (loopCount >= envelopes[ATTACK][TIME]) {
        changeState(DECAY, LOCAL_CHANGE);
      }
      break;
    case DECAY:
      if ((loopCount >= envelopes[DECAY][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(SUSTAIN, LOCAL_CHANGE);
      }
      break;
    case SUSTAIN:
      if ((loopCount >= envelopes[SUSTAIN][TIME]) || (currentEvent == tEVENT_RELEASE)) {
        changeState(RELEASE1, LOCAL_CHANGE);
      }
      break;
    case RELEASE1:
      if (loopCount >= envelopes[RELEASE1][TIME]) {
        changeState(RELEASE2, LOCAL_CHANGE);
      }
      break;
    case RELEASE2:
      if (loopCount >= envelopes[RELEASE2][TIME]) {
        changeState(RELEASE3, LOCAL_CHANGE);
      }
      break;
    case RELEASE3:
      if (loopCount >= envelopes[RELEASE3][TIME]) {
        changeState(HOLD, LOCAL_CHANGE);
      }
      break;
    case HOLD:
      /*if (loopCount >= envelopes[HOLD][TIME]) {
        changeState(OFF, LOCAL_CHANGE);
      }*/
      break;
  }

  currentBrightness = getCurrentBrightness(state, initBrightness, loopCount);
  if (currentColor != finalColor) {
    currentColor = getCurrentColor(finalColor, initColor, colorLoopCount);
  }

  uint32_t colorAndBrightness = wheelColor(currentColor, currentBrightness);
  updateNeoPixels(colorAndBrightness);
  loopCount++;
  colorLoopCount++;
}

int getCurrentBrightness(unsigned char state, int initBrightness, int loopCount) {
  if (state == OFF) return 0;
  if (envelopes[state][TIME] == 0) return envelopes[state][END_VALUE];
  int brightnessDistance = envelopes[state][END_VALUE] - initBrightness;
  int brightnessDistanceXElapsedTime = brightnessDistance * loopCount / envelopes[state][TIME];
  return min(255, max(0, initBrightness + brightnessDistanceXElapsedTime));
}

int getCurrentColor(int finalColor, int initColor, int colorLoopCount) {
  if (colorLoopCount > LOOPS_TO_FINAL_COLOR) return finalColor;
  int colorDistance = calcColorChange(initColor, finalColor);
  int colorDistanceXElapsedTime = colorDistance * colorLoopCount / LOOPS_TO_FINAL_COLOR;
  return (256 + initColor + colorDistanceXElapsedTime) % 256;
}

int calcColorChange(int currentColor, int finalColor) {
  int d = currentColor - finalColor;
  return abs(d) > 127 ? d : -d;
}

void updateNeoPixels(uint32_t color) {
  uint16_t n = strip.numPixels();
  for (char i = 0; i < n; i++) {
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

byte scale(byte x, float scale) {
    return (byte)(x * scale + .5);
}

// Converts HSV color to RGB color
// hue is from 0 to 255 around the circle
// saturation is fixed to max
// value (brightness) is from 0 to 255
uint32_t wheelColor(byte hue, byte value) {
    //hue %= 256;
    //if (hue < 0) { hue += 256; }
    int H6 = 6*(int)hue;
    byte R = 0, G = 0, B = 0;
    // each 1/6 of a circle (42.5 is 1/6 of 255)
    if (hue < 43) { R = 255; G = H6; }
    else if (hue < 86) { R = 510-H6; G = 255; }
    else if (hue < 128) { G = 255; B = H6-510; }
    else if (hue < 171) { G = 1020-H6; B = 255; }
    else if (hue < 213) { R = H6-1020; B = 255; }
    else { R = 255; B = 1530-H6; }
    float brightness = value / 255.0;
    return strip.Color(scale(R, brightness), scale(G, brightness), scale(B, brightness));
}
