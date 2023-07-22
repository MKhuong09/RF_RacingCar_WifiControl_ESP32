  //===========================================================RF SPEED RACING WITH ESP32==============================================================
#include "WiFi.h"
#include "ESP32Servo.h"
#include "EEPROM.h"
#include "LiquidCrystal_I2C.h"

const char *wifiID = "Redmi Note 11S";
const char *password = "dua10Kday";

WiFiServer server(80);
WiFiClient client;

///////////////////////////////////////
IPAddress STA_IP(192, 168, 1, 71);
IPAddress STA_Gateway(192, 168, 1, 1);
IPAddress STA_Subnet(255, 255, 0, 0);
IPAddress STA_primaryDNS(8, 8, 8, 8);
IPAddress STA_secondaryDNS(8, 8, 4, 4);

///////////Macro define//////////////
#define EEPROM_SIZE             64
#define LED                     16

#define SteeringPin             4
#define ThrottlePin             12

#define volcheck                35

///////////Init Variables///////////////

String server_rx;

uint8_t server_rx_counter,
        server_rx_throttle,
        server_rx_steering;

uint8_t throttle_control;
uint8_t steering_control;

uint8_t throttle_trim;
uint8_t steering_trim;

uint32_t last_server_ping,
         server_ping;

bool led_toggle = true,
     client_connected = false;

uint8_t bat;

///////////Function Init////////////////

void Client_Process(void);

void throttle_init();
void throttle_speed(uint8_t Carspeed);

void steering_init();
void steering_angle(uint8_t steering_req);

void wifi_init(void);

void lcd_init();
void voltage_check();
////////////////////////////////////////

Servo steeringServo;
Servo throttleESC;

uint8_t cols = 16;
uint8_t rows = 2;
LiquidCrystal_I2C mylcd(0x27,cols,rows);
//============================================= MAIN ===========================================
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //////////////////////////
  pinMode(LED, OUTPUT);
  led_toggle = true;
  digitalWrite(LED, led_toggle);

  //////////////////////////
  EEPROM.begin(EEPROM_SIZE);
  delay(1000);
  throttle_trim = (uint8_t)EEPROM.read(0);
  steering_trim = (uint8_t)EEPROM.read(1);
  Serial.print("Steering Trim Load: ");
  Serial.print(steering_trim);
  Serial.print(" , ");
  Serial.print("Throttle Trim Load: ");
  Serial.println(throttle_trim);

  ///////LCD////////
  lcd_init();
  mylcd.print("BATTERY: ");
  
   ////EPS32 wifi connection:
  wifi_init();

  ////Start ESP32 Server
  server.begin();

  ////////////////Init Control////////////////////////
  steering_init();
  throttle_init();

}

void loop() {
  // put your main code here, to run repeatedly:
 
 /////////Check Battery Voltage://////////
  voltage_check();

//////////////////////////////////////////
    client = server.available();
  if (client) {
    client_connected = true;
    while (client.connected()) {
      if (client.available() > 0) {
        char inChar = (char)client.read();
        if (inChar == '\n') {
          Client_Process();
          client.write('\n');
        }
        server_rx += inChar;
      }
    }
  }
  else {
    client_connected = false;
    led_toggle = true;
    digitalWrite(LED, led_toggle);
    throttle_control = throttle_trim;
    steering_control = steering_trim;
    last_server_ping = millis();
    server_ping = 0;
  }
  
  delay(10);
}
//===============================================================================================

////////////////////////////connect WIFI//////////////////////////////////
void wifi_init()
{
  /////////////////////
  WiFi.mode(WIFI_STA);
  
  /////////////////////
  WiFi.disconnect(true);

  /////////////////////
  WiFi.begin(wifiID,password);
  Serial.println("Connecting Wifi......");
  /////////////////////
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print('.'); 
  }
  Serial.print("WiFi IP Address: ");
  Serial.println(WiFi.localIP());
  mylcd.setCursor(0,1);
  mylcd.print(WiFi.localIP());
}

/////////////////////Controlling Function/////////////////////////////////
void steering_init (){
  
  steeringServo.attach(SteeringPin);
}

void steering_angle(uint8_t steering_req){
  steeringServo.write(steering_req);
}

void throttle_init(){
  throttleESC.attach(ThrottlePin,1000,2000);
}

void throttle_speed(uint8_t Carspeed){
  throttleESC.write(Carspeed);
}

//////////////////
/////////////////////////
void Client_Process(void) {
  if (server_rx.indexOf("C(") >= 0 || server_rx.indexOf("S(") >= 0) {
    server_rx_counter = server_rx.substring(server_rx.indexOf('p') + 1, server_rx.indexOf('t')).toInt();
    server_rx_throttle = server_rx.substring(server_rx.indexOf('t') + 1, server_rx.indexOf('s')).toInt();
    server_rx_steering = server_rx.substring(server_rx.indexOf('s') + 1, server_rx.indexOf(')')).toInt();
    if (server_rx.indexOf("C(") >= 0) {
      steering_control = server_rx_steering;
      throttle_control = server_rx_throttle;

      Serial.print("Steering Control: ");
      Serial.print(steering_control);
      Serial.print(" , ");
      Serial.print("Throttle Control: ");
      Serial.print(throttle_control);
      Serial.print(" , ");

      steering_angle(steering_control);
      throttle_speed(throttle_control);
    }
    if (server_rx.indexOf("S(") >= 0) {
      throttle_trim = server_rx_throttle;
      steering_trim = server_rx_steering;
      EEPROM.write(0, throttle_trim);
      EEPROM.write(1, steering_trim);
      EEPROM.commit();
      Serial.print("Steering Trim: ");
      Serial.print(steering_trim);
      Serial.print(" , ");
      Serial.print("Throttle Trim: ");
      Serial.print(throttle_trim);
      Serial.print(" , ");
    }
    led_toggle = !led_toggle;
    digitalWrite(LED, led_toggle);
    server_ping = millis() - last_server_ping;
    last_server_ping = millis();
    Serial.print("Ping:");
    Serial.println(server_ping);
    server_rx = "";
  }
}

//////////////////
void lcd_init(){
  mylcd.begin();
  mylcd.backlight();
  mylcd.setCursor(0,0);
}

void voltage_check()
{
  uint16_t adcRead = analogRead(volcheck);
  bat = map(adcRead,0,4095,0,143);
  mylcd.setCursor(10,0);
  mylcd.print(bat/10);
  mylcd.print('.');
  mylcd.print(bat%10);  
}

//================================================================ The END ===============================================================
