#include <Arduino.h>
#include <Wire.h>
#include <BleGamepad.h>

#define MPU_ADDR 0x68
#define DEBUG_RAW_ACCEL 1

// If pitch (axis-3 / right stick Y) still doesn't respond correctly,
// flip this to 0 to go back to the original (unswapped) pairing.
#define AXIS_SWAP 1

const int JOY_SW = 5;


const int JOY_X_PIN = 36;
const int JOY_Y_PIN = 34;

// --- Pushbuttons (pulldown: pin reads HIGH when pressed) ---
const int BTN1_PIN = 4;
const int BTN2_PIN = 2;
const int BTN3_PIN = 22;
const int BTN4_PIN = 23;

BleGamepad bleGamepad("ESP32 Gyro Gamepad", "DIY", 100);

int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
int16_t Temp;


const float ANGLE_RANGE     = 45.0;
const float GYRO_DEAD_ZONE  = 4.0;
const float COMP_FILTER_ALPHA = 0.98;
const float GYRO_SENSITIVITY = 131.0;

float pitchAngle = 0;
float rollAngle  = 0;

float accelPitchOffset = 0;
float accelRollOffset  = 0;
float gyroXOffset = 0;
float gyroYOffset = 0;

unsigned long lastUpdateMicros = 0;

const int JOY_DEAD_ZONE = 150;
int joyXCenter = 2048;
int joyYCenter = 2048;

void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void readRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR, count);

    uint8_t i = 0;
    while (Wire.available() && i < count) {
        data[i++] = Wire.read();
    }
}


float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readMPU() {
    uint8_t rawData[14];
    readRegisters(0x3B, 14, rawData);

    AccX = (rawData[0] << 8) | rawData[1];
    AccY = (rawData[2] << 8) | rawData[3];
    AccZ = (rawData[4] << 8) | rawData[5];
    Temp = (rawData[6] << 8) | rawData[7];
    GyroX = (rawData[8] << 8) | rawData[9];
    GyroY = (rawData[10] << 8) | rawData[11];
    GyroZ = (rawData[12] << 8) | rawData[13];
}

const int BTN_PINS[4] = { BTN1_PIN, BTN2_PIN, BTN3_PIN, BTN4_PIN };
int btnStates[4] = { 0, 0, 0, 0 };

// Reads all four pulldown buttons into btnStates[] as 0/1.
void readButtons() {
    for (int i = 0; i < 4; i++) {
        btnStates[i] = (digitalRead(BTN_PINS[i]) == HIGH) ? 1 : 0;
    }
}

void setup() {
    Serial.begin(115200);

    Wire.begin(27, 14);
    delay(100);
    pinMode(JOY_SW, INPUT_PULLUP);
    writeRegister(0x6B, 0x00);
    writeRegister(0x1C, 0x00);
    writeRegister(0x1B, 0x00);

    Serial.println("MPU6500 Initialized");

    pinMode(BTN1_PIN, INPUT_PULLDOWN);
    pinMode(BTN2_PIN, INPUT_PULLDOWN);
    pinMode(BTN3_PIN, INPUT_PULLDOWN);
    pinMode(BTN4_PIN, INPUT_PULLDOWN);


    long sumJX = 0, sumJY = 0;
    const int joySamples = 20;
    for (int i = 0; i < joySamples; i++) {
        sumJX += analogRead(JOY_X_PIN);
        sumJY += analogRead(JOY_Y_PIN);
        delay(5);
    }
    joyXCenter = sumJX / joySamples;
    joyYCenter = sumJY / joySamples;
    Serial.printf("Joystick center calibrated: X=%d Y=%d\n", joyXCenter, joyYCenter);


    Serial.println("Calibrating gyro/accel bias, keep board still...");
    const int gyroSamples = 200;
    double sumPitch = 0, sumRoll = 0, sumGX = 0, sumGY = 0;
    for (int i = 0; i < gyroSamples; i++) {
        readMPU();
        float ax = AccX / 16384.0;
        float ay = AccY / 16384.0;
        float az = AccZ / 16384.0;
        sumPitch += atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
        sumRoll  += atan2(ay, az) * 180.0 / PI;
        sumGX += GyroX;
        sumGY += GyroY;
        delay(5);
    }
    accelPitchOffset = sumPitch / gyroSamples;
    accelRollOffset  = sumRoll / gyroSamples;
    gyroXOffset = sumGX / gyroSamples;
    gyroYOffset = sumGY / gyroSamples;
    Serial.printf("Bias calibrated: pitchOffset=%.2f rollOffset=%.2f gyroXOffset=%.2f gyroYOffset=%.2f\n",
                  accelPitchOffset, accelRollOffset, gyroXOffset, gyroYOffset);

    lastUpdateMicros = micros();

    bleGamepad.begin();
}

void loop() {
    if (!bleGamepad.isConnected()) {
        delay(50);
        return;
    }


    unsigned long now = micros();
    float dt = (now - lastUpdateMicros) / 1000000.0;
    lastUpdateMicros = now;

    readMPU();

    float ax = AccX / 16384.0;
    float ay = AccY / 16384.0;
    float az = AccZ / 16384.0;

    if (DEBUG_RAW_ACCEL) {
        Serial.print("raw ax:"); Serial.print(ax, 3);
        Serial.print(" ay:"); Serial.print(ay, 3);
        Serial.print(" az:"); Serial.print(az, 3);
        Serial.print(" | GyroX:"); Serial.print(GyroX);
        Serial.print(" GyroY:"); Serial.println(GyroY);
    }


    float accelPitch = (atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI) - accelPitchOffset;
    float accelRoll  = (atan2(ay, az) * 180.0 / PI) - accelRollOffset;

    #if AXIS_SWAP
    // Pitch driven by GyroY, roll driven by GyroX (swapped from original pairing)
    float gyroPitchRate = (GyroY - gyroYOffset) / GYRO_SENSITIVITY;
    float gyroRollRate  = (GyroX - gyroXOffset) / GYRO_SENSITIVITY;
    #else
    // Original pairing
    float gyroPitchRate = (GyroX - gyroXOffset) / GYRO_SENSITIVITY;
    float gyroRollRate  = (GyroY - gyroYOffset) / GYRO_SENSITIVITY;
    #endif


    pitchAngle = COMP_FILTER_ALPHA * (pitchAngle + gyroPitchRate * dt) + (1 - COMP_FILTER_ALPHA) * accelPitch;
    rollAngle  = COMP_FILTER_ALPHA * (rollAngle  + gyroRollRate  * dt) + (1 - COMP_FILTER_ALPHA) * accelRoll;


    float pitchAdj = (fabs(pitchAngle) < GYRO_DEAD_ZONE) ? 0 : pitchAngle;
    float rollAdj  = (fabs(rollAngle)  < GYRO_DEAD_ZONE) ? 0 : rollAngle;


    pitchAdj = constrain(pitchAdj, -ANGLE_RANGE, ANGLE_RANGE);
    rollAdj  = constrain(rollAdj,  -ANGLE_RANGE, ANGLE_RANGE);

    int16_t rightX = (int16_t) mapFloat(rollAdj,  -ANGLE_RANGE, ANGLE_RANGE, -32767, 32767);
    int16_t rightY = (int16_t) mapFloat(pitchAdj, -ANGLE_RANGE, ANGLE_RANGE, -32767, 32767);


    int rawX = analogRead(JOY_X_PIN);
    int rawY = analogRead(JOY_Y_PIN);

    int centeredX = rawX - joyXCenter;
    int centeredY = rawY - joyYCenter;

    if (abs(centeredX) < JOY_DEAD_ZONE) centeredX = 0;
    if (abs(centeredY) < JOY_DEAD_ZONE) centeredY = 0;

    int16_t leftX = (int16_t) constrain(map(centeredX, -2048, 2048, -32767, 32767), -32767, 32767);
    int16_t leftY = (int16_t) constrain(map(centeredY, -2048, 2048, -32767, 32767), -32767, 32767);

    Serial.print("  L:"); Serial.print(leftX); Serial.print(","); Serial.print(leftY);
    Serial.print("  R:"); Serial.print(rightX); Serial.print(","); Serial.print(rightY);

    // --- Buttons ---
    readButtons();
    bleGamepad.setLeftThumb(-leftX, leftY);


/*
    int state = 0;

    for (int i = 0; i < 4; i++) {
        state |= btnStates[i] << i;
    }*/

/*
    switch(state){
        case 0b0000:
            break;
        case 0b0011:
            bleGamepad.setRightTrigger(32767);
        case 0b1111:
            break;
    }*/

    if(btnStates[1] == 1){
        bleGamepad.press(BUTTON_2);
    }
    else{
        bleGamepad.release(BUTTON_2);
    }

    Serial.print("pitch:"); Serial.print(pitchAngle, 1);
    Serial.print(" roll:"); Serial.print(rollAngle, 1);

    Serial.print("  BTN:[");
    for (int i = 0; i < 4; i++) {
        Serial.print(btnStates[i]);
        if (i < 3) Serial.print(",");
    }
    Serial.println("]");


    bleGamepad.setRightThumbAndroid(0, rightX);
    bleGamepad.setRY(rightY);



    if (digitalRead(JOY_SW) == LOW)
    {
        bleGamepad.press(BUTTON_1);
    }
    else
    {
        bleGamepad.release(BUTTON_1);
    }

    // bleGamepad.setRightTrigger(0);
    delay(20);
}
