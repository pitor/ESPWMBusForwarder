#ifndef __CONFIG_H__
#define __CONFIG_H__

#include <vector>

#define DEBUG 1 // set to 1 for detailed logging at UART
#define ESP_NAME "esp-multical21"   // for dns resolving
#define MQTT_TOPIC "forwarder/0"  // mqtt prefix topic



// WIFI configuration, supports more than one WIFI, first found first served
// if you dont use MQTT, leave broker/user/pass empty ("")
// if you dont need user/pass for MQTT, leave it empty ("")
struct CredentialStruct {
    char const* ssid;           // Wifi ssid
    char const* password;	      // Wifi password
    char const* mqtt_broker;		// MQTT broker ip address
    char const* mqtt_username;	// MQTT username
    char const* mqtt_password;	// MQTT password
};

// more than one wifi credentials are supported, upper one wins
// "ssid", "wifi_passphrase", "mqtt_broker", "mqtt_username", "mqtt_password"
std::vector<CredentialStruct> const credentials = {
     { "ssid1", "********", "", "", ""}   // no MQTT
   , { "ssid2", "********", "10.14.0.1", "", ""} // MQTT without auth
   , { "ssid3", "********", "10.0.0.111", "mqttuser", "********"}  // MQTT with auth
};

#if defined(ESP8266)
// Attach CC1101 pins to ESP8266 SPI pins
// VCC   => 3V3
// GND   => GND
// CSN   => D8
// MOSI  => D7
// MISO  => D6
// SCK   => D5
// GD0   => D2  A valid interrupt pin for your platform (defined below this)
// GD2   => not connected 
  #define CC1101_GDO0         D2   // GDO0 input interrupt pin
  #define PIN_LED_BUILTIN     D4
#elif defined(ESP32)
// Attach CC1101 pins to ESP32 SPI pins
// VCC   => 3V3
// GND   => GND
// CSN   => 4
// MOSI  => 23
// MISO  => 19
// SCK   => 18
// GD0   => 32  any valid interrupt pin for your platform will do
// GD2   => not connected 

// attach CC1101 pins to ESP32 SPI pins

  #define CC1101_GDO0          32
  #define PIN_LED_BUILTIN      2
#endif
#endif // __CONFIG_H__