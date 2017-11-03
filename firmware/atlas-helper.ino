#include <Arduino.h>
#include <Wire.h>

const uint8_t ATLAS_SENSOR_EC_DEFAULT_ADDRESS = 9; // 0x64;
const uint8_t ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS = 0x66;
const uint8_t ATLAS_SENSOR_PH_DEFAULT_ADDRESS = 0x63;
const uint8_t ATLAS_SENSOR_DO_DEFAULT_ADDRESS = 0x61;
const uint8_t ATLAS_SENSOR_ORP_DEFAULT_ADDRESS = 0x62;

const uint8_t ATLAS_RESPONSE_CODE_NO_DATA = 0xff;
const uint8_t ATLAS_RESPONSE_CODE_NOT_READY = 0xfe;
const uint8_t ATLAS_RESPONSE_CODE_ERROR = 0x2;
const uint8_t ATLAS_RESPONSE_CODE_SUCCESS = 0x1;

class AtlasScientificBoard {
private:
    uint8_t address;

private:
    uint8_t readResponse(const char *str, char *buffer, size_t length, uint32_t read_delay = 200) {
        Wire.beginTransmission(address);
        Wire.write(str);
        Wire.endTransmission();

        delay(read_delay);

        while (true) {
            Wire.requestFrom((uint8_t)address, 1 + length, (uint8_t)1);

            uint8_t code = Wire.read();
            if (code == ATLAS_RESPONSE_CODE_NOT_READY) {
                delay(100);
                continue;
            }

            uint8_t i = 0;
            while (Wire.available()) {
                uint8_t c = Wire.read();
                if (buffer != nullptr && i < length - 1) {
                    buffer[i++] = c;
                }
                if (c == 0) {
                    Wire.endTransmission();
                    break;
                }
            }
            if (buffer != nullptr) {
                buffer[i] = 0;
            }

            return code;
        }
    }

public:
    AtlasScientificBoard(uint8_t address) {
        this->address = address;
    }

    bool find() {
        uint8_t value = readResponse("FIND", nullptr, 0);
        return value == 0x1;
    }

    bool uart(uint32_t baud) {
        uint8_t value = readResponse("SERIAL,9600", nullptr, 0);
        return value == 0x1;
    }

    bool ledsOn() {
        uint8_t value = readResponse("L,1", nullptr, 0);
        return value == 0x1;
    }

    bool begin() {
        char buffer[20];
        uint8_t value = readResponse("I", buffer, sizeof(buffer));
        return value != 0xff;
    }
};

class AtlasHelper {
private:
    uint8_t addressWeFound = 0;

public:
    void setup() {
        pinMode(0, INPUT);
        pinMode(1, INPUT);

        Wire.begin();

        //  TODO: Investigate. I would see hangs if I used a slower speed.
        Wire.setClock(400000);
    }

    bool test(uint8_t address, const char *name) {
        AtlasScientificBoard sensor(address);
        Serial.print("atlas-helper: ");
        Serial.print(name);
        if (!sensor.begin()) {
            Serial.println(" FAILED");
            return false;
        }
        else {
            addressWeFound = address;
            Serial.println(" PASSED");
        }

        if (!sensor.ledsOn()) {
            Serial.println("atlas-helper: LEDS FAILED");
        }

        sensor.find();

        return true;
    }

    bool ec() {
        return test(ATLAS_SENSOR_EC_DEFAULT_ADDRESS, "EC");
    }

    bool temp() {
        return test(ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS, "TEMP");
    }

    bool ph() {
        return test(ATLAS_SENSOR_PH_DEFAULT_ADDRESS, "PH");
    }

    bool dissolvedOxygen() {
        return test(ATLAS_SENSOR_DO_DEFAULT_ADDRESS, "DO");
    }

    bool orp() {
        return test(ATLAS_SENSOR_ORP_DEFAULT_ADDRESS, "ORP");
    }

    bool findAnySensor() {
        return ec() || temp() || ph() || dissolvedOxygen() || orp() ;
    }

    bool backToUart() {
        AtlasScientificBoard sensor(addressWeFound);
        Serial.println("atlas-helper: Back to UART");
        sensor.uart(9600);
    }

    bool pingDevice() {
        Serial.println("atlas-helper: Ping!");
        return test(addressWeFound, "ATLAS");
    }

    bool changeDeviceToI2c() {
        Serial1.begin(9600);

        addressWeFound = 0;

        while (true) {
            Serial1.print("I\r");

            String reply = Serial1.readStringUntil('\r');

            if (reply.length() > 0) {
                Serial.println(reply);
            }
            if (reply.startsWith("?I")) {
                if (reply.indexOf("EC") >= 0) {
                    addressWeFound = ATLAS_SENSOR_EC_DEFAULT_ADDRESS;
                    Serial.println("atlas-helper: Found EC");
                    break;
                }
                if (reply.indexOf("ORP") >= 0) {
                    addressWeFound = ATLAS_SENSOR_ORP_DEFAULT_ADDRESS;
                    Serial.println("atlas-helper: Found ORP");
                    break;
                }
                if (reply.indexOf("pH") >= 0) {
                    addressWeFound = ATLAS_SENSOR_PH_DEFAULT_ADDRESS;
                    Serial.println("atlas-helper: Found pH");
                    break;
                }
                if (reply.indexOf("DO") >= 0) {
                    addressWeFound = ATLAS_SENSOR_DO_DEFAULT_ADDRESS;
                    Serial.println("atlas-helper: Found DO");
                    break;
                }
                if (reply.indexOf("RTD") >= 0) {
                    addressWeFound = ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS;
                    Serial.println("atlas-helper: Found RTD");
                    break;
                }
            }

            delay(500);
        }

        if (addressWeFound == 0) {
            return false;
        }

        Serial.print("atlas-helper: Configuring with address: ");
        Serial.println(addressWeFound);

        Serial1.print("I2C,");
        Serial1.print(addressWeFound);
        Serial1.print("\r");
        Serial1.flush();

        pinMode(0, INPUT);
        pinMode(1, INPUT);

        return true;
    }
};

void setup() {
    Serial.begin(115200);

    AtlasHelper helper;
    helper.setup();

    while (!Serial /*&& millis() < 2 * 1000*/) {
        delay(100);
    }

    if (false) {
        for (uint8_t i = 0; i < 128; ++i) {
            AtlasScientificBoard sensor(i);
            if (sensor.begin()) {
                Serial.println(i);
            }
        }
    }

    Serial.println("atlas-helper: Begin");

    if (helper.findAnySensor()) {
        // helper.backToUart();
        Serial.println("atlas-helper: Done");
        return;
    }

    if (helper.changeDeviceToI2c()) {
        helper.setup();

        while (millis() < 10000) {
            delay(1000);

            if (helper.pingDevice()) {
                Serial.println("atlas-helper: Done");
                return;
            }
        }
    }

    delay(100);
}

void loop() {
}
