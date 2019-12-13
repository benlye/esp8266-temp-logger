/*
 * This program is free software : you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.If not, see < https://www.gnu.org/licenses/>.
 */

#include <ArduinoOTA.h>
#include <DHT.h>
#include <ESP8266mDNS.h>
#include <ESP8266WiFi.h>
#include <ThingSpeak.h>
#include <Ticker.h>

// Hardware configuration parameters
const uint8_t ledPin = LED_BUILTIN;             // Pin where the LED is connected
const uint8_t dhtSensorPin = D5;                // Pin where the DHT sensor's data pin is connected; D2 = GPIO4
const uint8_t dhtSensorType = DHT11;            // Sensor type - can be a DHT11, DHT22, DHT21, DHT22, or AM2301

// Network configuration parameters
const char * networkSSID = "SSSSSSSSS";         // Wi-Fi network ID
const char * networkPassword = "PPPPPPPPPPPP";  // Wi-Fi network password
const char * networkHostname = "";              // Optional device hostname; if empty, the device-generated name will be used (e.g. ESP-0A1B2C)

// ThingSpeak configuration parameters
const uint32_t tsChannelId  = 123456;           // ThingSpeak channel ID
const char * tsApiKey = "ABCDEFG012345678";     // ThingSpeak channel write API key
const uint16_t tsLoggingInterval = 300;         // Logging interval in seconds
const uint8_t tsTempFieldId = 1;                // ID of the channel field for temperature data
const uint8_t tsHumidityFieldId = 2;            // ID of the channel field for humidity data

// Variable to keep track of when data was last logged
long lastUpdateTime = 0;

// Global instance of Wi-Fi client for ThingSpeak API to use
WiFiClient client;

// Global instance of DHT class for communicating with the DHT sensor
DHT dhtSensor(dhtSensorPin, dhtSensorType);

// Instance of Ticker class for controlling the LED
Ticker ticker;

/*
 * Toggles the state of the LED
 */
void toggleLED()
{
    digitalWrite(ledPin, !digitalRead(ledPin));
}

/*
 * Turns the LED off
 */
void disableLED()
{
    ticker.detach();
    digitalWrite(ledPin, HIGH);
}

/*
 * Turns the LED on
 */
void enableLED()
{
    ticker.detach();
    digitalWrite(ledPin, LOW);
}

/*
 * Called if device loses connection to the Wi-Fi network - resets the device
 */
void lostWifiCallback (const WiFiEventStationModeDisconnected& evt)
{
    Serial.println("WARNING: Wi-Fi network connection lost, resetting device");
    ESP.reset();
    delay(1000);
}

/*
 * Connects to the Wi-Fi network using the configured SSID and password.
 */
void connectWiFi()
{
    // Set the hostname if one was configured
    if (strlen(networkHostname) > 0) {
        WiFi.hostname(networkHostname);
    }

    Serial.printf("Connecting to Wi-Fi network %s ", networkSSID);

    // Connect to the Wi-Fi network
    WiFi.begin(networkSSID, networkPassword);
    while (WiFi.status() != WL_CONNECTED) {     
        delay(1000);
        Serial.print(".");
    }
    
    Serial.println(" connected.");

    // Configure a callback to reset device if the Wi-Fi connection is lost
    WiFi.onStationModeDisconnected(&lostWifiCallback);
}

/*
 * Configures the Arduino OTA callbacks.
  */
void configureArduinoOTA()
{
    Serial.print("Configuring Arduino OTA ...");

    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(WiFi.hostname().c_str());

    ArduinoOTA.onStart([]() {
        Serial.println("Arduino OTA Upload Start");
        });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nArduino OTA Upload End");
        });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Arduino OTA Upload Progress: %u%% (%u / %u Bytes)\n", (progress / (total / 100)), progress, total);
        });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Arduino OTA Upload Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

    ArduinoOTA.begin();

    Serial.println(" done.");
}

/*
 * Sends temperature and humidity data to ThingSpeak using the ThingSpeak library.
 */
void sendThingSpeakData(float temp, float hum)
{
    ThingSpeak.begin(client);

    // Set the field values
    ThingSpeak.setField(tsTempFieldId, temp);
    ThingSpeak.setField(tsHumidityFieldId, hum);

    // write to the ThingSpeak channel
    int tsResponseCode = ThingSpeak.writeFields(tsChannelId, tsApiKey);

    // Write the response from ThingSpeak to serial out
    if(tsResponseCode == OK_SUCCESS)
    {
        Serial.println("ThingSpeak channel update successful");
    }
    else
    {
        Serial.printf("\nERROR: Problem updating ThingSpeak channel. HTTP error code %d\n\n", tsResponseCode);
    }
}

void setup() 
{
    // Start serial loggging
    Serial.begin(115200);
    Serial.println("\nDevice initializing ...");
    
    // Configure the LED pin
    pinMode(ledPin, OUTPUT);

    // Start flashing the LED
    ticker.attach(0.1, toggleLED);

    // Connect to the Wi-Fi network
    connectWiFi();

    // Configure the Arduino OTA service
    configureArduinoOTA();

    // Initialise the DHT sensor
    dhtSensor.begin();

    // Write the configuration to serial out
    Serial.println("Device intialization complete.\n");
    Serial.println("Device Configuration");
    Serial.println("  Hostname:      " + WiFi.hostname());
    Serial.println("  MAC Address:   " + WiFi.macAddress());
    Serial.println("  Wi-Fi Network: " + WiFi.SSID());
    Serial.println("  Local IP:      " + WiFi.localIP().toString());
    Serial.println("  Sensor Type:   DHT" + String(dhtSensorType));
    Serial.println("\nThingSpeak Configuration");
    Serial.println("  Channel ID:    " + String(tsChannelId));
    Serial.println("  API Write Key: " + String(tsApiKey));
    Serial.println("  Log Interval:  " + String(tsLoggingInterval));
    Serial.println("");

    // Turn the LED on then turn it off after 1s
    enableLED();
    ticker.attach(1, disableLED);
}

void loop()
{
    // Only poll the sensor at startup or after the interval has elapsed
    if ((lastUpdateTime == 0) || (millis() - lastUpdateTime >= (tsLoggingInterval * 1000L)))
    {
        Serial.println("Reading DHT sensor ...");

        // Read data from the sensor
        float t = dhtSensor.readTemperature();
        float h = dhtSensor.readHumidity();

        // Log whatever data we got to serial
        Serial.println("T: " + String(t) + "; H: " + String(h));

        // Check if we got valid data
        if (isnan(h) || isnan(t))
        {
            // Write to serial
            Serial.println("\nERROR: Failed to read from DHT sensor!\n");
        } else {
            // Send the data to ThingSpeak
            sendThingSpeakData(t, h);
        }

        // Store the time
        lastUpdateTime = millis();

        // Write to serial
        Serial.println("Waiting for " + String(tsLoggingInterval) + " seconds ...\n");
    }

    // Handle an OTA update
    ArduinoOTA.handle();
}
