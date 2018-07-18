#include <Arduino.h>
#include <Wire.h>
#include <cstdarg>

const uint8_t ATLAS_SENSOR_EC_DEFAULT_ADDRESS = 0x64;
const uint8_t ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS = 0x66;
const uint8_t ATLAS_SENSOR_PH_DEFAULT_ADDRESS = 0x63;
const uint8_t ATLAS_SENSOR_DO_DEFAULT_ADDRESS = 0x61;
const uint8_t ATLAS_SENSOR_ORP_DEFAULT_ADDRESS = 0x62;

const uint8_t ATLAS_RESPONSE_CODE_NO_DATA = 0xff;
const uint8_t ATLAS_RESPONSE_CODE_NOT_READY = 0xfe;
const uint8_t ATLAS_RESPONSE_CODE_ERROR = 0x2;
const uint8_t ATLAS_RESPONSE_CODE_SUCCESS = 0x1;

void logf(const char *f, ...);

void logfln(const char *f, ...);

class AtlasScientificBoard {
private:
    char buffer[20];
    uint8_t address;

private:
    uint8_t readResponse(const char *str, char *buffer, size_t length, uint32_t readDelay = 200) {
        Wire.beginTransmission(address);
        Wire.write(str);
        Wire.endTransmission();

        delay(readDelay);

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

    bool lock() {
        uint8_t value = readResponse("Plock,1", nullptr, 0);
        return value == 0x1;
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
        uint8_t value = readResponse("I", buffer, sizeof(buffer));
        return value != 0xff;
    }

    bool configure(uint8_t address) {
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "I2C,%d", address);
        uint8_t value = readResponse(buffer, nullptr, 0);
        logfln("Send: %s got %d", buffer, address);
        this->address = address;
        return value == 0x1;
    }

    const char *who() {
        return buffer;
    }
};

uint8_t getExpectedAddressFromWho(String reply) {
    if (reply.startsWith("?I")) {
        if (reply.indexOf("EC") >= 0) {
            return ATLAS_SENSOR_EC_DEFAULT_ADDRESS;
        }
        if (reply.indexOf("ORP") >= 0) {
            return ATLAS_SENSOR_ORP_DEFAULT_ADDRESS;
        }
        if (reply.indexOf("pH") >= 0) {
            return ATLAS_SENSOR_PH_DEFAULT_ADDRESS;
        }
        if (reply.indexOf("DO") >= 0) {
            return ATLAS_SENSOR_DO_DEFAULT_ADDRESS;
        }
        if (reply.indexOf("RTD") >= 0) {
            return ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS;
        }
    }
    return 0x0;
}

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

    bool check(uint8_t address) {
        AtlasScientificBoard sensor(address);
        if (sensor.begin()) {
            Serial.print("atlas-helper: Found I2C device on 0x");
            Serial.print(address, HEX);
            Serial.print(" ");
            Serial.print(sensor.who());
            Serial.println();

            auto expectedAddress = getExpectedAddressFromWho(sensor.who());
            if (expectedAddress != address) {
                logfln("atlas-helper: Wrong address! (expected = 0x%x, actual = 0x%x)", expectedAddress, address);

                if (!sensor.configure(expectedAddress)) {
                    logfln("atlas-helper: Failed to fix address.");
                    return false;
                }

                logfln("atlas-helper: Fixed");

                delay(500);
            }

            if (!sensor.lock()) {
                logfln("atlas-helper: Failed to lock!");
                return false;
            }
            else {
                logfln("atlas-helper: Locked");
            }

            return true;
        }

        return false;
    }

    bool scanExpected() {
        uint8_t addresses[] = {
            ATLAS_SENSOR_EC_DEFAULT_ADDRESS,
            ATLAS_SENSOR_TEMP_DEFAULT_ADDRESS,
            ATLAS_SENSOR_PH_DEFAULT_ADDRESS,
            ATLAS_SENSOR_DO_DEFAULT_ADDRESS,
            ATLAS_SENSOR_ORP_DEFAULT_ADDRESS,
        };

        for (auto i : addresses) {
            // Serial.print("atlas-helper: Check 0x");
            // Serial.println(i, HEX);
            if (check(i)) {
                return true;
            }
        }

        return false;
    }

    bool scanEntireBus() {
        Serial.println("atlas-helper: Scanning entire bus.");

        for (uint8_t i = 128; i > 0; --i) {
            if (check(i)) {
                Serial.println();
                return true;
            }
            else {
                Serial.print(".");
            }
        }

        Serial.println();

        return false;
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

    bool lock() {
        AtlasScientificBoard sensor(addressWeFound);
        return sensor.lock();
    }

    bool changeDeviceToI2c() {
        Serial1.begin(9600);

        addressWeFound = 0;

        auto started = millis();

        while (millis() - started < 5000) {
            Serial1.print("I\r");

            auto reply = Serial1.readStringUntil('\r');

            auto expectedAddress = getExpectedAddressFromWho(reply);
            if (expectedAddress > 0) {
                addressWeFound = expectedAddress;
                break;
            }

            if (reply.length() > 0) {
                Serial.println(reply);
            }

            delay(500);
        }

        if (addressWeFound == 0) {
            Serial.println("atlas-helper: No device on UART...");
            return false;
        }

        Serial.print("atlas-helper: Configuring with address: 0x");
        Serial.println((uint32_t)addressWeFound, HEX);

        Serial1.print("I2C,");
        Serial1.print((uint32_t)addressWeFound);
        Serial1.print("\r");
        Serial1.flush();

        Serial.println("atlas-helper: Configured");

        pinMode(0, INPUT);
        pinMode(1, INPUT);

        return true;
    }
};

void setup() {
    Serial.begin(115200);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);

    while (!Serial && millis() < 2 * 1000) {
        delay(100);
    }

    auto scanWholeBus = false;

    while (true) {
        Serial.println();
        Serial.println();
        Serial.println("atlas-helper: Begin");

        AtlasHelper helper;
        helper.setup();

        Serial.println("atlas-helper: Looking for known sensors on I2C...");

        if (!scanWholeBus && helper.scanExpected()) {
            scanWholeBus = false;
            helper.setup();
            Serial.println("atlas-helper: All done!");
            delay(5000);
        }
        else if (scanWholeBus && helper.scanEntireBus()) {
            scanWholeBus = false;
            helper.setup();
            Serial.println("atlas-helper: All done!");
            delay(5000);
        }
        else {
            Serial.println("atlas-helper: None, looking on Uart...");

            if (!helper.changeDeviceToI2c()) {
                Serial.println("atlas-helper: Failed to change device to I2C...");

                scanWholeBus = true;
            }
            else {
                scanWholeBus = false;

                helper.setup();

                Serial.println("atlas-helper: Scanning, again");

                if (helper.scanExpected()) {
                    Serial.println("atlas-helper: All done!");
                    delay(5000);
                }
            }
        }

        delay(1000);
    }
}

void loop() {
}

#define DEBUG_LINE_MAX 256

void logf(const char *f, ...) {
    char buffer[DEBUG_LINE_MAX];
    va_list args;

    va_start(args, f);
    vsnprintf(buffer, DEBUG_LINE_MAX, f, args);
    va_end(args);

    Serial.print(buffer);
}

void logfln(const char *f, ...) {
    char buffer[DEBUG_LINE_MAX];
    va_list args;

    va_start(args, f);
    auto w = vsnprintf(buffer, DEBUG_LINE_MAX - 2, f, args);
    va_end(args);

    buffer[w] = '\r';
    buffer[w + 1] = '\n';
    buffer[w + 2] = 0;

    Serial.print(buffer);
}
