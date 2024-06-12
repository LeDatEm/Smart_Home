
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "Adafruit_SHTC3.h"
#include <ESP32Servo.h>
#include <BH1750.h>
#include <EEPROM.h>     
#include <SPI.h>       
#include <MFRC522.h> 
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <BigNumbers_I2C.h>


#define WIFI_SSID "Wi-Fi"
#define WIFI_PASSWORD "20022002"
#define BUZZER 5
#define LIVING_ROOM_LAMP 2
#define BEDROOM_LAMP 15
#define LIVING_ROOM_FAN 13
#define BEDROOM_FAN 12
#define FIRE_SENSOR 25
#define RAIN_SENSOR 33
#define MAIN_DOOR_BT 34
#define WINDOWN_BT 35
#define SKYLIGHT_BT 32
#define LIVING_ROOM_FAN_BT 36
#define BEDROOM_FAN_BT 39
#define GARDEN_LIGHT 26
#define LIVING_ROOM_LAMP_BT 3
#define BEDROOM_LAMP_BT 1

#define MAIN_DOOR_SERVO 4
#define WINDOWN_SERVO 16
#define SKYLIGHT_SERVO 17


WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x3f,20,4);
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();
BigNumbers_I2C bigNum(&lcd);
WiFiUDP ntpUDP;
#define offset 25200  // UTC +7 (7 * 3600)                 
NTPClient timeClient(ntpUDP, "pool.ntp.org");

BH1750 lightMeter;
Servo main_door_servo;
Servo windown_servo;
Servo skylight_servo;

const char* mqtt_server = "192.168.70.143";


unsigned long lastButtonClickTime = 0;
const unsigned long debounceDelay = 100;


bool MainDoorOutState = false;
bool WindownOutState = false;
bool SkyLightOutState = false;
bool LivingRoomFanOutState = false;
bool BedRoomFanOutState = false;
bool LivingRoomLampOutState = false;
bool BedRoomLampOutState = false;
bool GardenLightsOutState = false;
bool RainSensingOutState = false;
bool FireAlarmOutState = false;
bool LightingOutState = false;
bool TemperatureOutState = false;
bool lastMainDoorButtonState = HIGH;
bool lastWindownButtonState = HIGH;
bool lastSkyLightButtonState = HIGH;
bool lastLivingRoomFanButtonState = HIGH;
bool lastBedRoomFanButtonState = HIGH;
bool lastLivingRoomLampButtonState = HIGH;
bool lastBedRoomLampButtonState = HIGH;

boolean match = false;   
boolean programMode = false;  
boolean replaceMaster = false;
uint8_t successRead;

byte storedCard[4];  
byte readCard[4];
byte masterCard[4]; 
constexpr uint8_t RST_PIN = 27;    
constexpr uint8_t SS_PIN = 14; 


byte degree_angle_main_door = 110;
byte degree_angle_windown = 140;
byte degree_angle_sky_light = 123;
byte DONVI[8] = {
  B11000,
  B11000,
  B00110,
  B01001,
  B01000,
  B01001,
  B00110,
  B00000
};

float SHTC3_temperature;
float SHTC3_humidity;
float BH1750_lux ;
bool RAIN_sensor;
bool FIRE_sensor;

long now = millis();
long lastMeasure = 0;

int brightnessLVRLamp  = 255;
int brightnessBRLamp = 255;
int brightnessGardenLights = 255;
int speedLVRFan = 255;
int speedBRFan = 255;
int temperatureSet = 35;

MFRC522 mfrc522(SS_PIN, RST_PIN);







void OUTPUT_connect(){
  pinMode(BUZZER, OUTPUT);
  pinMode(LIVING_ROOM_FAN,OUTPUT);
  pinMode(BEDROOM_FAN,OUTPUT);
  pinMode(LIVING_ROOM_LAMP,OUTPUT);
  pinMode(BEDROOM_LAMP,OUTPUT);
  pinMode(GARDEN_LIGHT,OUTPUT);
  digitalWrite(BUZZER,LOW);
  digitalWrite(LIVING_ROOM_FAN, LOW);
  digitalWrite(BEDROOM_FAN, LOW);
  digitalWrite(LIVING_ROOM_LAMP, LOW);
  digitalWrite(BEDROOM_LAMP, LOW);
  digitalWrite(GARDEN_LIGHT, LOW);
}

void BUTTON_connect(){
  pinMode(MAIN_DOOR_BT, INPUT_PULLUP);
  pinMode(WINDOWN_BT, INPUT_PULLUP);
  pinMode(SKYLIGHT_BT, INPUT_PULLUP);
  pinMode(LIVING_ROOM_FAN_BT, INPUT_PULLUP);
  pinMode(BEDROOM_FAN_BT, INPUT_PULLUP);
  pinMode(LIVING_ROOM_LAMP_BT, INPUT_PULLUP);
  pinMode(BEDROOM_LAMP_BT, INPUT_PULLUP); 
}

void FIRE_SENSOR_connect(){
  pinMode(FIRE_SENSOR, INPUT);
}

void RAIN_SENSOR_connect(){
  pinMode(RAIN_SENSOR,INPUT);
}

void ShowReaderDetails() {
  byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.print(F("MFRC522 software version: 0x"));
  Serial.print(v, HEX);
  if (v == 0x91)
    Serial.print(F(" = v1.0"));
  else if (v == 0x92)
    Serial.print(F(" = v2.0"));
  else
    Serial.print(F(" (unknown), probably a Chinese clone?"));//(không xác định), có lẽ là một bản sao Trung Quốc?
  Serial.println("");
  // When 0x00 or 0xFF is returned, communication probably failed
  if ((v == 0x00) || (v == 0xFF)) {
    Serial.println(F("ALERT: Communication failure, is the MFRC522 module connected correctly?"));//CẢNH BÁO: Lỗi giao tiếp, mô-đun MFRC522 đã được kết nối đúng cách chưa?
    Serial.println(F("SYSTEM ABORTED: Check the connections"));//HỆ THỐNG ĐÃ HỦY BỎ: Kiểm tra kết nối
    // Visualize system is halted
    // digitalWrite(greenLed, LED_OFF);  // Make sure green LED is off
    // digitalWrite(blueLed, LED_OFF);   // Make sure blue LED is off
    // digitalWrite(redLed, LED_ON);   // Turn on red LED
    while (true); // do not go further
  }
}

void Wipe_code(){
    if (digitalRead(MAIN_DOOR_BT) == LOW) {  // when button pressed pin should get low, button connected to ground
    //digitalWrite(redLed, LED_ON); // Red Led stays on to inform user we are going to wipe
    Serial.println(F("Format button pressed"));
    Serial.println(F("You have 10 seconds to cancel"));//Bạn có 10 giây để hủy bỏ
    Serial.println(F("This will erase all your records, and there's no way to undo it"));//Điều này sẽ xóa tất cả các bản ghi của bạn, và không có cách nào để hoàn tác
    bool buttonState = monitorWipeButton(10000); // Give user enough time to cancel operation
    if (buttonState == true && digitalRead(MAIN_DOOR_BT) == LOW) {    // If button still be pressed, wipe EEPROM
      Serial.println(F("EEPROM formatting started"));//Bắt đầu định dạng EEPROM
      for (uint16_t x = 0; x < EEPROM.length(); x = x + 1) {    //Loop end of EEPROM address
        if (EEPROM.read(x) == 0) {              //If EEPROM address 0
          // do nothing, already clear, go to the next address in order to save time and reduce writes to EEPROM
        }
        else {
          EEPROM.write(x, 0);       // if not write 0 to clear, it takes 3.3mS
        }
      }
      Serial.println(F("EEPROM formatted successfully"));//EEPROM được định dạng thành công
      // digitalWrite(redLed, LED_OFF);  // visualize a successful wipe
      // delay(200);
      // digitalWrite(redLed, LED_ON);
      // delay(200);
      // digitalWrite(redLed, LED_OFF);
      // delay(200);
      // digitalWrite(redLed, LED_ON);
      // delay(200);
      // digitalWrite(redLed, LED_OFF);
    }
    else {
      Serial.println(F("Formatacao cancelada")); // Show some feedback that the wipe button did not pressed for 15 seconds
      // digitalWrite(redLed, LED_OFF);
    }
  }

  if (EEPROM.read(1) != 143) {
    Serial.println(F("Master card not set"));//Thẻ Master chưa được đặt
    lcd.setCursor(0,0);
    lcd.print("Master card not set!");
    Serial.println(F("Read a chip to set the Master card"));//Đọc một chip để đặt thẻ Master
    lcd.setCursor(1,2);
    lcd.print("Read a chip to set");
    lcd.setCursor(2,3);
    lcd.print("the Master card");
    do {
      successRead = getID();            // sets successRead to 1 when we get read from reader otherwise 0
      // digitalWrite(blueLed, LED_ON);    // Visualize Master Card need to be defined
      // delay(200);
      // digitalWrite(blueLed, LED_OFF);
      // delay(200);

    }
    while (!successRead);                  // Program will not go further while you not get a successful read
    for ( uint8_t j = 0; j < 4; j++ ) {        // Loop 4 times
      EEPROM.write( 2 + j, readCard[j] );  // Write scanned PICC's UID to EEPROM, start from address 3
    }
    EEPROM.write(1, 143);                  // Write to EEPROM we defined Master Card.
    lcd.clear();
    Serial.println(F("Master card set"));//Thẻ Master đã được đặt
    lcd.setCursor(0,1);
    lcd.print("[OK] Master card set");
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(50);
    digitalWrite(BUZZER, HIGH);
    delay(100);
    digitalWrite(BUZZER, LOW);
    delay(1000);
    lcd.clear();
  }
  Serial.println(F("-------------------"));
  Serial.println(F("UID of the Master card"));//UID của thẻ Master
  for ( uint8_t i = 0; i < 4; i++ ) {          // Read Master Card's UID from EEPROM
    masterCard[i] = EEPROM.read(2 + i);    // Write it to masterCard
    Serial.print(masterCard[i], HEX);
  }
  Serial.println("");
  Serial.println(F("-------------------"));
  Serial.println(F("Everything is ready"));//Mọi thứ đã sẵn sàng
  Serial.println(F("Waiting for the chips to be read"));//Chờ đợi các chip để đọc
  cycleLeds();    // Everything ready lets give user some feedback by cycling leds

  EEPROM.commit();
}

void LCD_connect(){
  lcd.init();
  lcd.backlight();
  bigNum.begin();
  lcd.setCursor(4,1);
  lcd.print("Smart Home");
  lcd.setCursor(1,2);
  lcd.print("UTILIZING NODE-RED");
  lcd.setCursor(10,3);
  lcd.print("By Group10");
  delay(1000);
  lcd.clear();
}

void SHTC3_connect(){
  while (!Serial)
    delay(10);
  Serial.println("SHTC3 test");
  if (!shtc3.begin()) {
    Serial.println("Couldn't find SHTC3");
    lcd.setCursor(0,2);
    lcd.print("Couldn't find SHTC3");
    while (1) delay(1);
  }
  Serial.println("Found SHTC3 sensor");
  lcd.clear();
}

void BH1750_connect(){
  lightMeter.begin();
  Serial.println(F("BH1750 Test begin"));
}

void SERVO_connect(){
  main_door_servo.setPeriodHertz(50);  
  windown_servo.setPeriodHertz(50);  
  skylight_servo.setPeriodHertz(50);  
	main_door_servo.attach(MAIN_DOOR_SERVO, 500, 2400);
  windown_servo.attach(WINDOWN_SERVO, 500, 2400);
  skylight_servo.attach(SKYLIGHT_SERVO, 500, 2400);
  main_door_servo.write(0);
  windown_servo.write(140);
  skylight_servo.write(0);
}

// void setup_wifi() {
//   digitalWrite(BUZZER, LOW);
//   delay(10);
//   Serial.println();
//   Serial.print("Connecting to ");
//   Serial.println(WIFI_SSID);
//   lcd.setCursor(0, 0);
//   lcd.print("CONNECTING TO");
//   lcd.setCursor(0, 1);
//   lcd.print(WIFI_SSID);

//   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
//   while (WiFi.status() != WL_CONNECTED) {
//     delay(500);
//     Serial.print(".");
//   }
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print("CONNECTED IP:");
//   lcd.setCursor(0, 1);
//   lcd.print(WiFi.localIP());
//   Serial.println("");
//   Serial.println("WiFi connected");
//   Serial.println("IP address: ");
//   Serial.println(WiFi.localIP());
//   delay(2000);
//   lcd.clear();
// }

// void callback(String topic, byte* message, unsigned int length) {
//   Serial.print("Message arrived on topic: ");
//   Serial.print(topic);
//   Serial.print(". Message: ");
//   String message_maindoor;
//   String message_windown;
//   String message_skylight;
//   String message_lvrlamp;
//   String message_lvrfan;
//   String message_brlamp;
//   String message_brfan;
//   String message_gardenlights;
//   String message_autorain;
//   String message_autofire;
//   String message_autolight;
//   String message_autotemperature;
//   String message_brightness_lvr_lamp;
//   String message_brightness_br_lamp;
//   String message_brightness_garden_lights;
//   String message_speed_lvr_fan;
//   String message_speed_br_fan;  
//   String message_temperatureSet;  
//   //String message_reset_default;
//   for (int i = 0; i < length; i++) {
//     Serial.print((char)message[i]);
//     message_maindoor += (char)message[i];
//     message_windown += (char)message[i];
//     message_skylight += (char)message[i];
//     message_lvrlamp += (char)message[i];
//     message_lvrfan += (char)message[i];
//     message_brlamp += (char)message[i];
//     message_brfan += (char)message[i];
//     message_gardenlights += (char)message[i];
//     message_autorain += (char)message[i];
//     message_autofire += (char)message[i];
//     message_autolight += (char)message[i];
//     message_autotemperature += (char)message[i];
//     message_brightness_lvr_lamp += (char)message[i];
//     message_brightness_br_lamp += (char)message[i];
//     message_brightness_garden_lights += (char)message[i];
//     message_speed_lvr_fan += (char)message[i];
//     message_speed_br_fan += (char)message[i];
//     message_temperatureSet += (char)message[i];
//   }
//   Serial.println();
//   unsigned long currentMillis = millis(); 
//   if (topic == "main_door_out") {
//     Serial.print("Changing [MAIN DOOR] to ");
//     if (message_maindoor == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       main_door_servo.write(degree_angle_main_door);
//       MainDoorOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_maindoor == "0") {
//       main_door_servo.write(0);
//       MainDoorOutState = false;
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "windown_out") {
//     Serial.print("Changing [WINDOWN] to ");
//     if (message_windown == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       windown_servo.write(0);
//       WindownOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_windown == "0") {
//       windown_servo.write(degree_angle_windown);
//       WindownOutState = false;
//       Serial.print("OFF");

//     }
//   }
  
//   if (topic == "sky_light_out") {
//     Serial.print("Changing [SKY LIGHT] to ");
//     if (message_skylight == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       skylight_servo.write(degree_angle_sky_light);
//       SkyLightOutState = true;
//       Serial.print("ON");

//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_skylight == "0") {
//       skylight_servo.write(0);
//       SkyLightOutState = false;
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "lvr_lamp_out") {
//     Serial.print("Changing [LIVING ROOM LAMP] to ");
//     if (message_lvrlamp == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       LivingRoomLampOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_lvrlamp == "0") {
//       analogWrite(LIVING_ROOM_LAMP,0);
//       LivingRoomLampOutState = false;
//       Serial.print("OFF");
//     }
//   }
//   if (topic == "lvr_fan_out") {
//     Serial.print("Changing [LIVING ROOM FAN] to ");
//     if (message_lvrfan == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
      
//       LivingRoomFanOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_lvrfan == "0") {
//       digitalWrite(LIVING_ROOM_FAN, LOW);
//       LivingRoomFanOutState = false;
//       Serial.print("OFF");
//     }
//   }
//   if (topic == "br_lamp_out") {
//     Serial.print("Changing [BED ROOM LAMP] to ");
//     if (message_brlamp == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       digitalWrite(BEDROOM_LAMP, HIGH);
//       BedRoomLampOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_brlamp == "0") {
//       digitalWrite(BEDROOM_LAMP, LOW);
//       BedRoomLampOutState = false;
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "br_fan_out") {
//     Serial.print("Changing [BED ROOM FAN] to ");
//     if (message_brfan == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       digitalWrite(BEDROOM_FAN, HIGH);
//       BedRoomFanOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_brfan == "0") {
//       digitalWrite(BEDROOM_FAN, LOW);
//       BedRoomFanOutState = false;
//       Serial.print("OFF");
//     }
//   }
//   if (topic == "gardenlights_out") {
//     Serial.print("Changing [BED ROOM FAN] to ");
//     if (message_gardenlights == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       GardenLightsOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_gardenlights == "0") {
//       digitalWrite(GARDEN_LIGHT, LOW);
//       GardenLightsOutState = false;
//       Serial.print("OFF");
//     }
//   }
//   if (topic == "automatic_rain_sensing_out") {
//     if (message_autorain == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       RainSensingOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_autorain == "0") {
//       RainSensingOutState = false;
//       skylight_servo.write(0);
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "automatic_fire_alarm_out") {
//     if (message_autofire == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       FireAlarmOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_autofire == "0") {
//       FireAlarmOutState = false;
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "automatic_lighting_out") {
//     if (message_autolight == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       LightingOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_autolight == "0") {
//       LightingOutState = false;
//       Serial.print("OFF");
//     }
//   }

//   if (topic == "automatic_temperature_out") {
//     if (message_autotemperature == "1" && (currentMillis - lastButtonClickTime) > debounceDelay) {
//       lastButtonClickTime = currentMillis;
//       TemperatureOutState = true;
//       Serial.print("ON");
//       digitalWrite(BUZZER, HIGH);
      
//     } else if (message_autotemperature == "0") {
//       TemperatureOutState = false;
//       Serial.print("OFF");
//     }
//   }

//  if (topic == "brightness_lvr_lamp") {
//     brightnessLVRLamp = message_brightness_lvr_lamp.toFloat();

//   }
//  if (topic == "brightness_br_lamp") {
//     brightnessBRLamp = message_brightness_br_lamp.toFloat();

//   }
//   if (topic == "brightness_garden_lights") {
//     brightnessGardenLights = message_brightness_garden_lights.toFloat();

//   }
//   if (topic == "speed_lvr_fan") {
//     speedLVRFan = message_speed_lvr_fan.toFloat();

//   }
//   if (topic == "speed_br_fan") {
//     speedBRFan = message_speed_br_fan.toFloat();

//   }
//   if (topic == "temperature_set") {
//     temperatureSet = message_temperatureSet.toFloat();

//   }


//   if ((currentMillis - lastButtonClickTime) > 30) {
//     digitalWrite(BUZZER, LOW);
//   }

// }

////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600);
  EEPROM.begin(1024);
  OUTPUT_connect();
  BUTTON_connect();
  FIRE_SENSOR_connect();
  RAIN_SENSOR_connect();
  LCD_connect();
  SHTC3_connect();
  BH1750_connect();
  SERVO_connect();
  SPI.begin();       
  mfrc522.PCD_Init(); 
  ShowReaderDetails(); 
  Wipe_code();
  //setup_wifi();
  //timeClient.begin();
  //timeClient.setTimeOffset(offset);    
  //client.setServer(mqtt_server, 1883);
  //client.setCallback(callback);
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////


void SHTC3_read(){
  sensors_event_t humidity, temp;
  shtc3.getEvent(&humidity, &temp);
  SHTC3_temperature = temp.temperature;
  SHTC3_humidity = humidity.relative_humidity;
}

void BH1750_read(){
  BH1750_lux = lightMeter.readLightLevel();
}
void FIRE_read(){
  FIRE_sensor = digitalRead(FIRE_SENSOR);
  //client.publish("fire", FIRE_sensor ? "false" : "true");

}
void RAIN_read(){
  RAIN_sensor = digitalRead(RAIN_SENSOR);
  //client.publish("rain", RAIN_sensor ? "false" : "true");
}

void display_show(){
  char tempStr[6];
  char humiStr[6];
  char luxStr[6];
  dtostrf(SHTC3_temperature, 1, 1, tempStr);
  dtostrf(SHTC3_humidity, 1, 1, humiStr);
  dtostrf(BH1750_lux,1, 0, luxStr);
  lcd.setCursor(0, 0);
  lcd.print("Temp:");
  lcd.setCursor(5,0);
  lcd.print(tempStr);
  lcd.createChar(0, DONVI);
  lcd.setCursor(9,0);
  lcd.write((byte)0);
  lcd.setCursor(10,0);
  lcd.print("|");
  lcd.setCursor(11, 0);
  lcd.print("Lux:");
  lcd.setCursor(15, 0);
  lcd.print(luxStr);
  if(BH1750_lux < 10000){
    lcd.setCursor(19, 0);
    lcd.print(" ");
    if(BH1750_lux <1000){
      lcd.setCursor(18, 0);
      lcd.print("  ");
      if(BH1750_lux < 100){
        lcd.setCursor(17, 0);
        lcd.print("   ");
        if(BH1750_lux < 10){
          lcd.setCursor(16, 0);
          lcd.print("    ");
        }
      }
    }
  }
  lcd.setCursor(0, 1);
  lcd.print("Humi:");
  lcd.setCursor(5, 1);
  lcd.print(humiStr);
  lcd.setCursor(9, 1);
  lcd.print("%");
  lcd.setCursor(10,1);
  lcd.print("|");
  lcd.setCursor(11, 1);
  lcd.print("Fi:");
  lcd.setCursor(14, 1);
  lcd.print(!FIRE_sensor);
  lcd.setCursor(15,1);
  lcd.print("|");
  lcd.setCursor(16,1);
  lcd.print("Ra:");
  lcd.setCursor(19, 1);
  lcd.print(!RAIN_sensor);
  lcd.setCursor(0,2);
  lcd.print("MD:");
  lcd.setCursor(3,2);
  lcd.print(MainDoorOutState);
  lcd.setCursor(4,2);
  lcd.print("|");
  lcd.setCursor(5,2);
  lcd.print("Wd:");
  lcd.setCursor(8,2);
  lcd.print(WindownOutState);
  lcd.setCursor(9,2);
  lcd.print("|");
  lcd.setCursor(10,2);
  lcd.print("SL:");
  lcd.setCursor(13,2);
  lcd.print(SkyLightOutState);
  lcd.setCursor(14,2);
  lcd.print("|");
  lcd.setCursor(15,2);
  lcd.print("PL:");
  lcd.setCursor(18,2);
  lcd.print(GardenLightsOutState);

  lcd.setCursor(0,3);
  lcd.print("LL:");
  lcd.setCursor(3,3);
  lcd.print(LivingRoomLampOutState);
  lcd.setCursor(4,3);
  lcd.print("|");
  lcd.setCursor(5,3);
  lcd.print("LF:");
  lcd.setCursor(8,3);
  lcd.print(LivingRoomFanOutState);
  lcd.setCursor(9,3);
  lcd.print("|");
  lcd.setCursor(10,3);
  lcd.print("BL:");
  lcd.setCursor(13,3);
  lcd.print(BedRoomLampOutState);
  lcd.setCursor(14,3);
  lcd.print("|");
  lcd.setCursor(15,3);
  lcd.print("BF:");
  lcd.setCursor(18,3);
  lcd.print(BedRoomFanOutState);
}

void check_button_state() {
  unsigned long currentMillis = millis(); 
  bool currentMainDoorButtonState        = digitalRead(MAIN_DOOR_BT) == LOW;
  bool currentWindownButtonState         = digitalRead(WINDOWN_BT) == LOW;
  bool currentSkyLightButtonState        = digitalRead(SKYLIGHT_BT) == LOW;
  bool currentLivingRoomFanButtonState   = digitalRead(LIVING_ROOM_FAN_BT) == LOW;
  bool currentBedRoomFanButtonState      = digitalRead(BEDROOM_FAN_BT) == LOW;
  bool currentLivingRoomLampButtonState  = digitalRead(LIVING_ROOM_LAMP_BT) == LOW;
  bool currentBedRoomLampButtonState     = digitalRead(BEDROOM_LAMP_BT) == LOW;

  if (currentMainDoorButtonState && !lastMainDoorButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis; 
    MainDoorOutState = !MainDoorOutState;
    if(MainDoorOutState){
      
      main_door_servo.write(degree_angle_main_door);
    } else{
      main_door_servo.write(5);
    }
    //client.publish("main_door_in", String(MainDoorOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }

  if (currentWindownButtonState && !lastWindownButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;
    WindownOutState = !WindownOutState;
    if(WindownOutState){
      windown_servo.write(0);
      
    } else{
      windown_servo.write(degree_angle_windown);
    }
    //client.publish("windown_in", String(WindownOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }

  if (currentSkyLightButtonState && !lastSkyLightButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;
    SkyLightOutState = !SkyLightOutState;
    if(SkyLightOutState){
      skylight_servo.write(degree_angle_sky_light);
    } else{
      skylight_servo.write(0);
    }
    //client.publish("sky_light_in", String(SkyLightOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }
  if (currentLivingRoomLampButtonState && !lastLivingRoomLampButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;

    LivingRoomLampOutState = !LivingRoomLampOutState;
    
    //client.publish("lvr_lamp_in", String(LivingRoomLampOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }
  if (currentLivingRoomFanButtonState && !lastLivingRoomFanButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;

    LivingRoomFanOutState = !LivingRoomFanOutState;
    
    //client.publish("lvr_fan_in", String(LivingRoomFanOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }
  if (currentBedRoomLampButtonState && !lastBedRoomLampButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;

    BedRoomLampOutState = !BedRoomLampOutState;
    
    //client.publish("br_lamp_in", String(BedRoomLampOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }
  if (currentBedRoomFanButtonState && !lastBedRoomFanButtonState && (currentMillis - lastButtonClickTime) > debounceDelay) {
    lastButtonClickTime = currentMillis;

    BedRoomFanOutState = !BedRoomFanOutState;
    
    //client.publish("br_fan_in", String(BedRoomFanOutState).c_str());
    // saveSettingsToEEPROM();
    digitalWrite(BUZZER, HIGH);
  }

 


  if ((currentMillis - lastButtonClickTime) > 50) {
    digitalWrite(BUZZER, LOW);
  }
  
  lastMainDoorButtonState = currentMainDoorButtonState;
  lastWindownButtonState = currentWindownButtonState;
  lastSkyLightButtonState = currentSkyLightButtonState;
  lastLivingRoomLampButtonState = currentLivingRoomLampButtonState;
  lastLivingRoomFanButtonState = currentLivingRoomFanButtonState;
  lastBedRoomLampButtonState = currentBedRoomLampButtonState;
  lastBedRoomFanButtonState = currentBedRoomFanButtonState;
  
 
}

void check_sensor_state() {

  digitalWrite(LIVING_ROOM_FAN, SHTC3_temperature >= 30 ? HIGH : LOW);
  digitalWrite(LIVING_ROOM_LAMP, BH1750_lux <= 30 ? HIGH : LOW);
}

void RFID_read(){
  do {
    //MQTT_connect();
    successRead = getID();  // sets successRead to 1 when we get read from reader otherwise 0
    SHTC3_read();
    BH1750_read();
    FIRE_read();
    RAIN_read();
    check_button_state();
    automation_control();
    Display_print();
    check_out();
    // if (now - lastMeasure > 3000) {
    // lastMeasure = now;
    // data_sent_mqtt();
    // }
    // When device is in use if wipe button pressed for 10 seconds initialize Master Card wiping
    if (digitalRead(MAIN_DOOR_BT) == LOW) { // Check if button is pressed
      // Visualize normal operation is iterrupted by pressing wipe button Red is like more Warning to user
      // digitalWrite(redLed, LED_ON);  // Make sure led is off
      // digitalWrite(greenLed, LED_OFF);  // Make sure led is off
      // digitalWrite(blueLed, LED_OFF); // Make sure led is off
      // Give some feedback
      Serial.println(F("Format button pressed"));
      Serial.println(F("The Master card will be erased! in 10 seconds"));//Thẻ Master sẽ bị xóa! trong 10 giây
      bool buttonState = monitorWipeButton(10000); // Give user enough time to cancel operation
      if (buttonState == true && digitalRead(MAIN_DOOR_BT) == LOW) {    // If button still be pressed, wipe EEPROM
        EEPROM.write(1, 0);                  // Reset Magic Number.
        EEPROM.commit();
        digitalWrite(BUZZER, LOW);
        Serial.println(F("Master card unlinked from the device"));
        Serial.println(F("Press the board reset to reprogram the Master card"));//Nhấn nút reset của bo mạch để lập trình lại thẻ Master
        while (1);
      }
      Serial.println(F("Unlinking of the Master card canceled"));//Hủy liên kết thẻ Master
    }
    if (programMode) {
      cycleLeds();              // Program Mode cycles through Red Green Blue waiting to read a new card
    }
    else {
      normalModeOn();     // Normal mode, blue Power LED is on, all others are off
    }
  }
  while (!successRead);   //the program will not go further while you are not getting a successful read
  if (programMode) {
    if ( isMaster(readCard) ) { //When in program mode check First If master card scanned again to exit program mode
      lcd.setCursor(2, 3);
      lcd.print("Exiting...");
      Serial.println(F("Reading the Master card"));//Đọc thẻ Master
      Serial.println(F("Exiting programming mode"));//Thoát khỏi chế độ lập trình
      Serial.println(F("-----------------------------"));
      programMode = false;
      return;
    }
    else {
      if ( findID(readCard) ) { // If scanned card is known delete it
        Serial.println(F("I know this chip, removing..."));
        deleteID(readCard);
        Serial.println("-----------------------------");
        Serial.println(F("Read a chip to add or remove from the EEPROM"));//Đọc một chip để thêm hoặc loại bỏ từ EEPROM
      }
      else {                    // If scanned card is not known add it
        Serial.println(F("I don't recognize this chip, including..."));
        writeID(readCard);
        Serial.println(F("-----------------------------"));
        Serial.println(F("Read a chip to add or remove from the EEPROM"));//Đọc một chip để thêm hoặc loại bỏ từ EEPROM
      }
    }
  }
  else {
    if ( isMaster(readCard)) {    // If scanned card's ID matches Master Card's ID - enter program mode
      programMode = true;
      lcd.setCursor(1, 2);
      lcd.print("Master Mode");
      Serial.println(F("Hello Master - Programming mode initiated"));
      uint8_t count = EEPROM.read(0);   // Read the first Byte of EEPROM that
      Serial.print(F("Existem "));     // stores the number of ID's in EEPROM
      Serial.print(count);
      Serial.print(F(" record(s) in the EEPROM"));//bản ghi trong EEPROM
      Serial.println("");
      Serial.println(F("Read a chip to add or remove from the EEPROM"));//Đọc một chip để thêm hoặc loại bỏ từ EEPROM
      Serial.println(F("Read the Master card again to exit programming mode"));//Đọc thẻ Master lại để thoát khỏi chế độ lập trình
      Serial.println(F("-----------------------------"));
      delay(200);

    }
    else {
      if ( findID(readCard) ) { // If not, see if the card is in the EEPROM
        Serial.println(F("Welcome, you can pass"));//Chào mừng, bạn có thể đi qua
        granted(3000);         // Open the door lock for 300 ms
      }
      else {      // If not, show that the ID was not valid
        Serial.println(F("You cannot pass"));//Bạn không thể đi qua
        lcd.setCursor(0, 3);
        lcd.print("X Cannot pass");
        denied();
      }
    }
  }
}


/////////////////////////////////////////  Access Granted    ///////////////////////////////////
void granted ( uint16_t setDelay) {
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW);
  delay(40);
  digitalWrite(BUZZER, HIGH);
  delay(70);
  digitalWrite(BUZZER, LOW); 
  lcd.setCursor(2, 2);
  lcd.print(" Windown ");
  lcd.setCursor(4, 3);
  lcd.print("OPEN");
  main_door_servo.write(degree_angle_main_door);
  delay(setDelay);          // Hold door lock open for given seconds
  main_door_servo.write(0);        // Hold green LED on for a second
  lcd.setCursor(2, 2);
  lcd.print("          ");
  lcd.setCursor(4, 3);
  lcd.print("         ");
}

///////////////////////////////////////// Access Denied  ///////////////////////////////////
void denied() {
  digitalWrite(BUZZER, HIGH);
  delay(300);
  digitalWrite(BUZZER, LOW);
  delay(300);
}


///////////////////////////////////////// Get PICC's UID ///////////////////////////////////
uint8_t getID() {
  // Getting ready for Reading PICCs
  if ( ! mfrc522.PICC_IsNewCardPresent()) { //If a new PICC placed to RFID reader continue
    return 0;
  }
  if ( ! mfrc522.PICC_ReadCardSerial()) {   //Since a PICC placed get Serial and continue
    return 0;
  }
  // There are Mifare PICCs which have 4 byte or 7 byte UID care if you use 7 byte PICC
  // I think we should assume every PICC as they have 4 byte UID
  // Until we support 7 byte PICCs
  Serial.println(F("Chip UID read:"));//UID của chip đã đọc:
  for ( uint8_t i = 0; i < 4; i++) {  //
    readCard[i] = mfrc522.uid.uidByte[i];
    Serial.print(readCard[i], HEX);
  }
  Serial.println("");
  mfrc522.PICC_HaltA(); // Stop reading
  return 1;
}

///////////////////////////////////////// Cycle Leds (Program Mode) ///////////////////////////////////
unsigned long previousMilliscycleLeds = 0;
const long buzzerOnIntervalcycleLeds = 1000;
const long buzzerOffIntervalcycleLeds = 50;

void cycleLeds() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMilliscycleLeds >= buzzerOnIntervalcycleLeds) {
    digitalWrite(BUZZER, HIGH);
    previousMilliscycleLeds = currentMillis;
  } else if (currentMillis - previousMilliscycleLeds >= buzzerOffIntervalcycleLeds) {
    digitalWrite(BUZZER, LOW);
  }
}


//////////////////////////////////////// Normal Mode Led  ///////////////////////////////////
void normalModeOn () {
  // digitalWrite(blueLed, LED_ON);  // Blue LED ON and ready to read card
  // digitalWrite(redLed, LED_OFF);  // Make sure Red LED is off
  // digitalWrite(greenLed, LED_OFF);  // Make sure Green LED is off
  // digitalWrite(relay, LOW);    // Make sure Door is Locked
}

//////////////////////////////////////// Read an ID from EEPROM //////////////////////////////
void readID( uint8_t number ) {
  uint8_t start = (number * 4 ) + 2;    // Figure out starting position
  for ( uint8_t i = 0; i < 4; i++ ) {     // Loop 4 times to get the 4 Bytes
    storedCard[i] = EEPROM.read(start + i);   // Assign values read from EEPROM to array
  }
}

///////////////////////////////////////// Add ID to EEPROM   ///////////////////////////////////
void writeID( byte a[] ) {
  if ( !findID( a ) ) {     // Before we write to the EEPROM, check to see if we have seen this card before!
    uint8_t num = EEPROM.read(0);     // Get the numer of used spaces, position 0 stores the number of ID cards
    uint8_t start = ( num * 4 ) + 6;  // Figure out where the next slot starts
    num++;                // Increment the counter by one
    EEPROM.write( 0, num );     // Write the new count to the counter
    for ( uint8_t j = 0; j < 4; j++ ) {   // Loop 4 times
      EEPROM.write( start + j, a[j] );  // Write the array values to EEPROM in the right position
    }
    EEPROM.commit();
    successWrite();
    Serial.println(F("ID successfully added to the EEPROM")); //ID đã được thêm vào EEPROM thành công
  }
  else {
    failedWrite();
    Serial.println(F("Error! There is something wrong with the chip ID or a problem in the EEPROM."));//Lỗi! Có vấn đề gì đó với ID chip hoặc có vấn đề ở EEPROM
  }
}

///////////////////////////////////////// Remove ID from EEPROM   ///////////////////////////////////
void deleteID( byte a[] ) {
  if ( !findID( a ) ) {     // Before we delete from the EEPROM, check to see if we have this card!
    failedWrite();      // If not
    Serial.println(F("Error! There is something wrong with the chip ID or a problem in the EEPROM."));
  }
  else {
    uint8_t num = EEPROM.read(0);   // Get the numer of used spaces, position 0 stores the number of ID cards
    uint8_t slot;       // Figure out the slot number of the card
    uint8_t start;      // = ( num * 4 ) + 6; // Figure out where the next slot starts
    uint8_t looping;    // The number of times the loop repeats
    uint8_t j;
    uint8_t count = EEPROM.read(0); // Read the first Byte of EEPROM that stores number of cards
    slot = findIDSLOT( a );   // Figure out the slot number of the card to delete
    start = (slot * 4) + 2;
    looping = ((num - slot) * 4);
    num--;      // Decrement the counter by one
    EEPROM.write( 0, num );   // Write the new count to the counter
    for ( j = 0; j < looping; j++ ) {         // Loop the card shift times
      EEPROM.write( start + j, EEPROM.read(start + 4 + j));   // Shift the array values to 4 places earlier in the EEPROM
    }
    for ( uint8_t k = 0; k < 4; k++ ) {         // Shifting loop
      EEPROM.write( start + j + k, 0);
    }
    EEPROM.commit();
    successDelete();
    Serial.println(F("ID removido da EEPROM com sucesso"));
  }
}

///////////////////////////////////////// Check Bytes   ///////////////////////////////////
boolean checkTwo ( byte a[], byte b[] ) {
  if ( a[0] != 0 )      // Make sure there is something in the array first
    match = true;       // Assume they match at first
  for ( uint8_t k = 0; k < 4; k++ ) {   // Loop 4 times
    if ( a[k] != b[k] )     // IF a != b then set match = false, one fails, all fail
      match = false;
  }
  if ( match ) {      // Check to see if if match is still true
    return true;      // Return true
  }
  else  {
    return false;       // Return false
  }
}

///////////////////////////////////////// Find Slot   ///////////////////////////////////
uint8_t findIDSLOT( byte find[] ) {
  uint8_t count = EEPROM.read(0);       // Read the first Byte of EEPROM that
  for ( uint8_t i = 1; i <= count; i++ ) {    // Loop once for each EEPROM entry
    readID(i);                // Read an ID from EEPROM, it is stored in storedCard[4]
    if ( checkTwo( find, storedCard ) ) {   // Check to see if the storedCard read from EEPROM
      // is the same as the find[] ID card passed
      return i;         // The slot number of the card
      break;          // Stop looking we found it
    }
  }
}

///////////////////////////////////////// Find ID From EEPROM   ///////////////////////////////////
boolean findID( byte find[] ) {
  uint8_t count = EEPROM.read(0);     // Read the first Byte of EEPROM that
  for ( uint8_t i = 1; i <= count; i++ ) {    // Loop once for each EEPROM entry
    readID(i);          // Read an ID from EEPROM, it is stored in storedCard[4]
    if ( checkTwo( find, storedCard ) ) {   // Check to see if the storedCard read from EEPROM
      return true;
      break;  // Stop looking we found it
    }
    else {    // If not, return false
    }
  }
  return false;
}

///////////////////////////////////////// Write Success to EEPROM   ///////////////////////////////////
// Flashes the green LED 3 times to indicate a successful write to EEPROM


void successWrite() {
  lcd.setCursor(1, 3);
  lcd.print("Including...");
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(50);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);
  delay(50);
  digitalWrite(BUZZER, HIGH);
  delay(100);
  digitalWrite(BUZZER, LOW);

}




///////////////////////////////////////// Write Failed to EEPROM   ///////////////////////////////////
// Flashes the red LED 3 times to indicate a failed write to EEPROM
void failedWrite() {
  lcd.setCursor(1, 3);
  lcd.print("Failed...");
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);
  delay(500);
}

///////////////////////////////////////// Success Remove UID From EEPROM  ///////////////////////////////////
// Flashes the blue LED 3 times to indicate a success delete to EEPROM

void successDelete() {
  lcd.setCursor(2, 3);
  lcd.print("Remove...");
  digitalWrite(BUZZER, HIGH);
  delay(1000);
  digitalWrite(BUZZER, LOW);
}

////////////////////// Check readCard IF is masterCard   ///////////////////////////////////
// Check to see if the ID passed is the master programing card
boolean isMaster( byte test[] ) {
  if ( checkTwo( test, masterCard ) )
    return true;
  else
    return false;
}

bool monitorWipeButton(uint32_t interval) {
  uint32_t now = (uint32_t)millis();
  while ((uint32_t)millis() - now < interval)  {
    // check on every half a second
    if (((uint32_t)millis() % 500) == 0) {
      if (digitalRead(MAIN_DOOR_BT) != LOW)
        return false;
    }
  }
  return true;
}

// void MQTT_connect() {
//   if (!client.connected()) {
//     reconnect();
//   }
//   if (!client.loop()) {
//     client.connect("Client");
//   }
//   now = millis();
// }

// void reconnect() {
//   while (!client.connected()) {
//     Serial.print("Attempting MQTT connection...");
//     lcd.setCursor(0, 0);
//     lcd.print("MQTT CONNECTING");  
//     if (client.connect("Client")) {
//       Serial.println("connected");
//       lcd.clear();
//       lcd.setCursor(0, 0);
//       lcd.print("CONNECTED MQTT"); 
//       lcd.setCursor(0,1);
//       lcd.print("IP:");
//       lcd.setCursor(3,1);
//       lcd.print(mqtt_server);
//       client.subscribe("main_door_out");
//       client.subscribe("windown_out");
//       client.subscribe("sky_light_out");
//       client.subscribe("lvr_lamp_out");
//       client.subscribe("lvr_fan_out");
//       client.subscribe("br_lamp_out");
//       client.subscribe("br_fan_out");
//       client.subscribe("gardenlights_out");
//       client.subscribe("automatic_rain_sensing_out");
//       client.subscribe("automatic_fire_alarm_out");
//       client.subscribe("automatic_lighting_out");
//       client.subscribe("automatic_temperature_out");
//       client.subscribe("brightness_lvr_lamp");
//       client.subscribe("brightness_br_lamp");
//       client.subscribe("brightness_garden_lights");
//       client.subscribe("speed_lvr_fan");
//       client.subscribe("speed_br_fan");
//       client.subscribe("temperature_set");
//       delay(2000);
//       lcd.clear();
//     } else {
//       lcd.setCursor(0, 1);
//       lcd.print("Failed, rc=");
//       lcd.print(client.state());  
//       Serial.print("Failed, rc=");
//       Serial.print(client.state());
//       Serial.println(" try again in 5 seconds");
//       delay(3000);
//     }
//     lcd.clear();
//   }
// }

// void data_sent_mqtt() {
//   static char temperature_sent[7];
//   static char humidity_sent[7];
//   static char lux_sent[7];

//   dtostrf(SHTC3_temperature, 6, 2, temperature_sent);
//   dtostrf(SHTC3_humidity, 6, 1, humidity_sent);
//   dtostrf(BH1750_lux, 6, 1, lux_sent);

//   client.publish("temperature_chart", temperature_sent);
//   client.publish("humidity_chart", humidity_sent);
//   client.publish("lux", lux_sent);
  
// }

void automation_control(){
  if(RainSensingOutState){
    if(RAIN_sensor){
      skylight_servo.write(degree_angle_sky_light);
    } else {
      skylight_servo.write(0);
    }
  }
  if(FireAlarmOutState){
    if(!FIRE_sensor){
      digitalWrite(BUZZER, HIGH);
      main_door_servo.write(degree_angle_main_door);
      windown_servo.write(0);

    } else {
      
      main_door_servo.write(0);
      windown_servo.write(degree_angle_windown);
    }
  }
  if(LightingOutState){
    analogWrite(LIVING_ROOM_LAMP, BH1750_lux <= 50 ? brightnessLVRLamp : 0);
  }
  if(TemperatureOutState){
    analogWrite(LIVING_ROOM_FAN, SHTC3_temperature >= temperatureSet ? speedLVRFan : 0);
    analogWrite(BEDROOM_FAN, SHTC3_temperature >= temperatureSet ? speedBRFan : 0);
  }
}

void loop() {
 
  RFID_read();
}



unsigned long displayStartTime = 0;
const unsigned long displayDuration = 1000;
int currentCondition = 0; // Initialize to an invalid value

void Display_print() {
  //   timeClient.update();
  // time_t epochTime = timeClient.getEpochTime();
  // String formattedTime = timeClient.getFormattedTime();
  // struct tm *ptm = gmtime ((time_t *)&epochTime);
  // int monthDay = ptm->tm_mday;
  // int currentMonth = ptm->tm_mon+1;
  // int currentYear = ptm->tm_year+1900;
  // String currentDate = String(monthDay)+"/"+String(currentMonth)+"/"+String(currentYear); 
  // lcd.setCursor(1, 0);
  // lcd.print(currentDate);
  // lcd.setCursor(2, 1);
  // lcd.print(formattedTime);

  lcd.setCursor(2, 0);
  lcd.print("Smart Home");
  // lcd.setCursor(2, 1);
  // lcd.print(formattedTime);
  bigNum.displayLargeInt(SHTC3_temperature, 14, 0, 2, false);
  bigNum.displayLargeInt(SHTC3_humidity, 14, 2, 2, false);

  // Check conditions and update the currentCondition
  if (MainDoorOutState) {
    currentCondition = 1;
  } else if (WindownOutState) {
    currentCondition = 2;
  } else if (SkyLightOutState) {
    currentCondition = 3;
  } else if (LivingRoomLampOutState) {
    currentCondition = 4;
  } else if (BedRoomFanOutState) {
    currentCondition = 5;
  } else if (BedRoomLampOutState) {
    currentCondition = 6;
  } else if (LivingRoomFanOutState) {
    currentCondition = 7;
  } else if (GardenLightsOutState) {
    currentCondition = 8;
  } else {
    currentCondition = 0; // No condition met
  }

  // Display the current condition
  if (currentCondition != 0) {
    lcd.setCursor(2, 2);
    lcd.print(getConditionLabel(currentCondition));
    lcd.setCursor(4, 3);
    lcd.print("OPEN");
    displayStartTime = millis();
  }

  // Clear the display after a certain duration
  if (millis() - displayStartTime >= displayDuration) {
    lcd.setCursor(0, 2);
    lcd.print("             ");
    lcd.setCursor(0, 3);
    lcd.print("             ");  
  }
}

// Function to get the label for the current condition
String getConditionLabel(int condition) {
  switch (condition) {
    case 1: return "Main Door";
    case 2: return " Windown ";
    case 3: return "Sky Light";
    case 4: return "LRV Lamp ";
    case 5: return " BR Fan  ";
    case 6: return " BR Lamp ";
    case 7: return " LVR Fan ";
    case 8: return "Garden Li";
    default: return ""; // Invalid condition
  }
}


void check_out(){
  if(LivingRoomLampOutState ){
    analogWrite(LIVING_ROOM_LAMP,brightnessLVRLamp);
  } else if(!LightingOutState){
    analogWrite(LIVING_ROOM_LAMP,0);
  }
  if(BedRoomLampOutState){
    analogWrite(BEDROOM_LAMP,brightnessBRLamp);
  } else {
    analogWrite(BEDROOM_LAMP,0);
  }

  if(GardenLightsOutState){
    analogWrite(GARDEN_LIGHT,brightnessGardenLights);
  } else {
    analogWrite(GARDEN_LIGHT,0);
  }
  if(LivingRoomFanOutState){
    analogWrite(LIVING_ROOM_FAN,speedLVRFan);
  } else if(!TemperatureOutState){
    analogWrite(LIVING_ROOM_FAN,0);
  }
  if(BedRoomFanOutState){
    analogWrite(BEDROOM_FAN,speedBRFan);
  } else if(!TemperatureOutState){
    analogWrite(BEDROOM_FAN,0);
  }
}


