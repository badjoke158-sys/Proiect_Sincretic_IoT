#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

#include <WiFi.h>
#include <PubSubClient.h>

// WIFI 
const char* WIFI_SSID = "Wokwi-GUEST";
const char* WIFI_PASS = "";

//  MQTT 
const char* MQTT_HOST = "broker.emqx.io";
const int   MQTT_PORT = 1883;

const char* DEV_ID = "petdoor01";

// topic-uri comenzi
String T_CMD_BUTTON = String(DEV_ID) + "/cmd/button"; // payload: PRESS
String T_CMD_OPEN   = String(DEV_ID) + "/cmd/open";   // payload: 1
String T_CMD_CLOSE  = String(DEV_ID) + "/cmd/close";  // payload: 1
String T_CMD_AUTO   = String(DEV_ID) + "/cmd/auto";   // payload: 0/1

// topic-uri status
String T_ST_DOOR  = String(DEV_ID) + "/status/door";   // retained
String T_ST_EVENT = String(DEV_ID) + "/status/event";  // feed de evenimente
String T_ST_LCD   = String(DEV_ID) + "/status/lcd";    // optional (oglinda lcd)

WiFiClient espClient;
PubSubClient mqtt(espClient);

//  PINI 
#define PIR_PIN       27
#define BTN_PIN       14
#define SERVO_PIN     13
#define BUZZ_PIN      25

#define TRIG_LOW      18
#define ECHO_LOW      19
#define TRIG_HIGH     5
#define ECHO_HIGH     17

#define LDR_A0_PIN    34

// LCD 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SERVO 
Servo door;

//SETARI 
bool autoEnabled = true;         // se poate opri din app
bool nightLockEnabled = true;    // ramas pt viitor (nu insistam acum)

const int   LDR_THRESHOLD = 2000;

const int OPEN_ANGLE   = 90;
const int CLOSED_ANGLE = 0;

const unsigned long OPEN_TIME_MS  = 5000;
const unsigned long DEBOUNCE_MS   = 40;

const unsigned int ANIMAL_MAX_CM  = 35;
const unsigned int HUMAN_MIN_CM   = 45;

const unsigned int OBSTACLE_CM    = 15;
const unsigned long CLOSE_STEP_MS = 60;

//  STARE 
bool doorOpen = false;
unsigned long openSince = 0;

// buton fizic debounce
int lastBtnRaw = HIGH;
unsigned long lastDebounce = 0;
bool btnLatched = false;

//  HELPER: timp pt log 
// "timestamp" simplu (secunde de la pornire) ca sa nu complicam cu NTP
String ts() {
  return String(millis() / 1000) + "s";
}

// MQTT publish helpers 
void pub(const String& topic, const String& payload, bool retained=false){
  if (mqtt.connected()) {
    mqtt.publish(topic.c_str(), payload.c_str(), retained);
  }
}

void publishDoorState(){
  pub(T_ST_DOOR, doorOpen ? "OPEN" : "CLOSED", true);
}

void publishEvent(const String& e){
  // evenimente in feed: "123s: DOOR_OPENED (BTN)" etc.
  pub(T_ST_EVENT, ts() + ": " + e, false);
}

// optional: oglinda LCD pe telefon
void publishLCD(const String& l1, const String& l2){
  String msg = l1;
  if (l2.length() > 0) msg += " | " + l2;
  pub(T_ST_LCD, msg, false);
}

//  BUZZER simplu 
void beepDigital(int ms = 80, int times = 1, int gap = 60) {
  for (int i=0; i<times; i++){
    digitalWrite(BUZZ_PIN, HIGH);
    delay(ms);
    digitalWrite(BUZZ_PIN, LOW);
    delay(gap);
  }
}

//  LCD afisare unica (si trimite pe telefon) 
void lcdMsg(const String& l1, const String& l2=""){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
  publishLCD(l1, l2);
}

// SENZORI
bool isNight(){
  int ldr = analogRead(LDR_A0_PIN);
  return ldr > LDR_THRESHOLD;
}

unsigned int readCM(uint8_t trig, uint8_t echo){
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long dur = pulseIn(echo, HIGH, 30000UL);
  if(!dur) return 400;
  return dur / 58;
}

// ACTIUNI USA 
void openDoor(const String& reason){
  door.write(OPEN_ANGLE);
  doorOpen = true;
  openSince = millis();

  beepDigital(60,1);
  publishDoorState();
  publishEvent("DOOR_OPENED (" + reason + ")");
  lcdMsg("Door OPEN", reason);
}

void closeDoorWithSafety(const String& reason){
  int pos = OPEN_ANGLE;

  lcdMsg("Closing...", reason);

  while(pos > CLOSED_ANGLE){
    pos -= 3;
    if(pos < CLOSED_ANGLE) pos = CLOSED_ANGLE;
    door.write(pos);
    delay(CLOSE_STEP_MS);

    // siguranta: obstacol jos
    if(readCM(TRIG_LOW, ECHO_LOW) < OBSTACLE_CM){
      beepDigital(50,3,40);
      publishEvent("OBSTACLE -> REOPEN");
      lcdMsg("Obstacle!", "Reopen");

      door.write(OPEN_ANGLE);
      doorOpen = true;
      openSince = millis();
      publishDoorState();
      return;
    }
  }

  doorOpen = false;
  beepDigital(60,2,80);
  publishDoorState();
  publishEvent("DOOR_CLOSED (" + reason + ")");
  lcdMsg("Door CLOSED", reason);
}

// "apasare buton" comuna (fizic sau din app)
void toggleDoorFromButton(const String& origin){
  if(doorOpen) closeDoorWithSafety(origin);
  else         openDoor(origin);
}

//  MQTT callback
void onMqtt(char* topic, byte* payload, unsigned int len){
  String t = topic;
  String msg;
  for(unsigned int i=0;i<len;i++) msg += (char)payload[i];
  msg.trim();

  Serial.print("[MQTT] "); Serial.print(t);
  Serial.print(" = "); Serial.println(msg);

  if(t == T_CMD_BUTTON && msg == "PRESS"){
    // simuleaza butonul fizic
    toggleDoorFromButton("APP_BTN");
    return;
  }

  if(t == T_CMD_OPEN && msg == "1"){
    openDoor("APP_OPEN");
    return;
  }

  if(t == T_CMD_CLOSE && msg == "1"){
    if(doorOpen) closeDoorWithSafety("APP_CLOSE");
    return;
  }

  if(t == T_CMD_AUTO){
    autoEnabled = (msg == "1");
    publishEvent(autoEnabled ? "AUTO_ON" : "AUTO_OFF");
    lcdMsg("AUTO mode", autoEnabled ? "ON" : "OFF");
    return;
  }
}

//  CONNECT WiFi / MQTT
void connectWiFi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  lcdMsg("WiFi connect...", "");
  Serial.print("Connecting WiFi");
  while(WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi CONNECTED!");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  lcdMsg("WiFi OK", WiFi.localIP().toString());
}

void connectMQTT(){
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqtt);

  lcdMsg("MQTT connect...", "");
  while(!mqtt.connected()){
    Serial.print("MQTT try... state=");
    Serial.println(mqtt.state());

    String clientId = String("esp32-") + DEV_ID + "-" + String(random(0xffff), HEX);

    if(mqtt.connect(clientId.c_str())){
      Serial.println("MQTT CONNECTED!");

      mqtt.subscribe(T_CMD_BUTTON.c_str());
      mqtt.subscribe(T_CMD_OPEN.c_str());
      mqtt.subscribe(T_CMD_CLOSE.c_str());
      mqtt.subscribe(T_CMD_AUTO.c_str());

      publishDoorState();
      publishEvent("BOOT_OK");
      lcdMsg("MQTT OK", "Ready");
    } else {
      Serial.println("MQTT FAILED, retry...");
      delay(1000);
    }
  }
}

// SETUP 
void setup(){
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_PIN, INPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);

  pinMode(BUZZ_PIN, OUTPUT);
  digitalWrite(BUZZ_PIN, LOW);

  pinMode(TRIG_LOW, OUTPUT);  pinMode(ECHO_LOW, INPUT);
  pinMode(TRIG_HIGH, OUTPUT); pinMode(ECHO_HIGH, INPUT);

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  door.setPeriodHertz(50);
  door.attach(SERVO_PIN, 500, 2500);

  // test servo rapid
  lcdMsg("Init...", "Servo test");
  door.write(CLOSED_ANGLE); delay(300);
  door.write(OPEN_ANGLE);   delay(300);
  door.write(CLOSED_ANGLE); delay(300);
  doorOpen = false;

  connectWiFi();
  connectMQTT();

  lcdMsg("AUTO: READY", "Btn/App = toggle");
}

//  LOOP 
void loop(){
  if(!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // ---- buton fizic (toggle) ----
  int raw = digitalRead(BTN_PIN);
  if(raw != lastBtnRaw){
    lastDebounce = millis();
    lastBtnRaw = raw;
  }

  if(millis() - lastDebounce > DEBOUNCE_MS){
    bool pressed = (raw == LOW);
    if(pressed && !btnLatched){
      toggleDoorFromButton("BTN_PHYS");
      btnLatched = true;
    } else if(!pressed){
      btnLatched = false;
    }
  }

  // logica AUTO (doar daca autoEnabled) 
  if(autoEnabled){
    // noaptea: optional, nu insistam acum
    if(nightLockEnabled && isNight()){
      // daca e deschisa, inchidem
      if(doorOpen) closeDoorWithSafety("NIGHT");
      delay(80);
      return;
    }

    bool motion = (digitalRead(PIR_PIN) == HIGH);
    unsigned int dLow  = readCM(TRIG_LOW,  ECHO_LOW);
    unsigned int dHigh = readCM(TRIG_HIGH, ECHO_HIGH);

    bool isAnimal = (dLow < ANIMAL_MAX_CM) && (dHigh >= HUMAN_MIN_CM);

    if(!doorOpen && motion && isAnimal){
      openDoor("AUTO_ANIMAL");
    }

    if(doorOpen){
      if(motion && isAnimal) openSince = millis();
      if(millis() - openSince >= OPEN_TIME_MS){
        closeDoorWithSafety("TIMER");
      }
    }
  }

  delay(40);
}
