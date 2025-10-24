/**
 * @file LoRaNodeOptimized.ino
 * @author M. Rakeh Saleem
 * @brief Optimized LoRa end node for wireless sensor data transmission
 * @date Created: Unknown | Optimized: 2020
 * @version 2.0
 * 
 * Features:
 * - Modular sensor management
 * - Improved error handling
 * - Better data validation
 * - Enhanced communication reliability
 * - Configuration management
 */

#include <SPI.h>
#include <RH_RF95.h>
#include "SensorConfig.h"

// DHT11 Sensor Manager
class DHT11Manager {
private:
    int dataPin;
    byte sensorData[5];
    byte lastError;
    
public:
    DHT11Manager(int pin) : dataPin(pin), lastError(0) {}
    
    void init() {
        pinMode(dataPin, OUTPUT);
        digitalWrite(dataPin, HIGH);
    }
    
    bool readData() {
        lastError = 0;
        
        // Send start signal
        digitalWrite(dataPin, LOW);
        delay(30);
        digitalWrite(dataPin, HIGH);
        delayMicroseconds(40);
        
        // Check response
        pinMode(dataPin, INPUT);
        if (digitalRead(dataPin) != LOW) {
            lastError = 1;
            return false;
        }
        
        delayMicroseconds(80);
        if (digitalRead(dataPin) != HIGH) {
            lastError = 2;
            return false;
        }
        
        delayMicroseconds(80);
        
        // Read 40 bits of data
        for (int i = 0; i < 5; i++) {
            sensorData[i] = readByte();
        }
        
        pinMode(dataPin, OUTPUT);
        digitalWrite(dataPin, HIGH);
        
        // Verify checksum
        byte checksum = sensorData[0] + sensorData[1] + sensorData[2] + sensorData[3];
        if (sensorData[4] != checksum) {
            lastError = 3;
            return false;
        }
        
        return true;
    }
    
    float getHumidity() {
        return sensorData[0] + (sensorData[1] / 10.0);
    }
    
    float getTemperature() {
        return sensorData[2] + (sensorData[3] / 10.0);
    }
    
    byte getLastError() {
        return lastError;
    }
    
private:
    byte readByte() {
        byte result = 0;
        for (int i = 0; i < 8; i++) {
            while (digitalRead(dataPin) == LOW);
            delayMicroseconds(30);
            
            if (digitalRead(dataPin) == HIGH) {
                result |= (1 << (7 - i));
            }
            
            while (digitalRead(dataPin) == HIGH);
        }
        return result;
    }
};

// LoRa Communication Manager
class LoRaManager {
private:
    RH_RF95 rf95;
    float frequency;
    int txPower;
    char nodeId[3];
    unsigned int messageCount;
    
public:
    LoRaManager(float freq = 868.0, int power = 13) 
        : rf95(), frequency(freq), txPower(power), messageCount(1) {
        nodeId[0] = 1;
        nodeId[1] = 1;
        nodeId[2] = 1;
    }
    
    bool init() {
        if (!rf95.init()) {
            Serial.println("LoRa initialization failed");
            return false;
        }
        
        rf95.setFrequency(frequency);
        rf95.setTxPower(txPower);
        
        Serial.println("LoRa End Node Initialized");
        Serial.print("Frequency: ");
        Serial.print(frequency);
        Serial.println(" MHz");
        Serial.print("TX Power: ");
        Serial.print(txPower);
        Serial.println(" dBm");
        
        return true;
    }
    
    bool sendSensorData(float humidity, float temperature) {
        // Prepare data packet
        char data[50] = {0};
        int dataLength = 7;
        
        // Node ID
        data[0] = nodeId[0];
        data[1] = nodeId[1];
        data[2] = nodeId[2];
        
        // Humidity data
        data[3] = (byte)humidity;
        data[4] = (byte)((humidity - (byte)humidity) * 10);
        
        // Temperature data
        data[5] = (byte)temperature;
        data[6] = (byte)((temperature - (byte)temperature) * 10);
        
        // Calculate CRC
        uint16_t crc = calculateCRC16((unsigned char*)data, dataLength);
        
        // Prepare final packet with CRC
        unsigned char sendBuf[50] = {0};
        memcpy(sendBuf, data, dataLength);
        sendBuf[dataLength] = (unsigned char)crc;
        sendBuf[dataLength + 1] = (unsigned char)(crc >> 8);
        
        // Send packet
        bool success = rf95.send(sendBuf, dataLength + 2);
        
        if (success) {
            Serial.print("Message ");
            Serial.print(messageCount);
            Serial.println(" sent successfully");
            messageCount++;
        } else {
            Serial.println("Failed to send message");
        }
        
        return success;
    }
    
    bool waitForAcknowledgment() {
        uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        
        if (rf95.waitAvailableTimeout(3000)) {
            if (rf95.recv(buf, &len)) {
                // Check if acknowledgment is for this node
                if (buf[0] == nodeId[0] && buf[1] == nodeId[1] && buf[2] == nodeId[2]) {
                    Serial.println("Acknowledgment received from gateway");
                    return true;
                }
            }
        }
        
        Serial.println("No acknowledgment received");
        return false;
    }
    
private:
    uint16_t calculateCRC16(uint8_t *pBuffer, uint32_t length) {
        uint16_t wCRC16 = 0;
        
        if ((pBuffer == 0) || (length == 0)) {
            return 0;
        }
        
        for (uint32_t i = 0; i < length; i++) {
            wCRC16 = calcByte(wCRC16, pBuffer[i]);
        }
        
        return wCRC16;
    }
    
    uint16_t calcByte(uint16_t crc, uint8_t b) {
        uint32_t i;
        crc = crc ^ (uint32_t)b << 8;
        
        for (i = 0; i < 8; i++) {
            if ((crc & 0x8000) == 0x8000) {
                crc = crc << 1 ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
        
        return crc & 0xffff;
    }
};

// Global objects
DHT11Manager dht11(A0);
LoRaManager lora;

void setup() {
    Serial.begin(9600);
    
    // Initialize DHT11
    dht11.init();
    
    // Initialize LoRa
    if (!lora.init()) {
        Serial.println("LoRa initialization failed!");
        return;
    }
    
    Serial.println("LoRa End Node Ready");
    Serial.println("DHT11 Temperature and Humidity Sensor");
    Serial.println("Node ID: 111");
}

void loop() {
    Serial.println("========== Reading Sensors ==========");
    
    // Read DHT11 data
    if (dht11.readData()) {
        float humidity = dht11.getHumidity();
        float temperature = dht11.getTemperature();
        
        Serial.print("Humidity: ");
        Serial.print(humidity, 1);
        Serial.print("%  Temperature: ");
        Serial.print(temperature, 1);
        Serial.println("°C");
        
        // Send data via LoRa
        if (lora.sendSensorData(humidity, temperature)) {
            // Wait for acknowledgment
            if (lora.waitForAcknowledgment()) {
                // Blink LED to indicate successful transmission
                pinMode(4, OUTPUT);
                digitalWrite(4, HIGH);
                delay(400);
                digitalWrite(4, LOW);
            }
        }
    } else {
        // Handle DHT11 errors
        byte error = dht11.getLastError();
        switch (error) {
            case 1:
                Serial.println("Error: DHT11 start condition 1 not met");
                break;
            case 2:
                Serial.println("Error: DHT11 start condition 2 not met");
                break;
            case 3:
                Serial.println("Error: DHT11 checksum error");
                break;
            default:
                Serial.println("Error: Unknown DHT11 error");
                break;
        }
    }
    
    Serial.println("=====================================");
    delay(30000); // Send data every 30 seconds
}
