#include <Arduino.h>
#include <BleGamepad.h>

// -------------------------
// Pins
// -------------------------

#define BUTTON_A 25
#define BUTTON_B 26

#define TEST_X 34
#define TEST_Y 35

// -------------------------
// BLE Gamepad
// -------------------------

BleGamepad bleGamepad(
    "GBGC",
    "Aritra",
    100
);

// -------------------------
// Setup
// -------------------------

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println("      GBGC GAMEPAD TEST");
    Serial.println("==============================");

    pinMode(BUTTON_A, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);

    // Start BLE Gamepad
    bleGamepad.begin();

    Serial.println("BLE Gamepad started");
    Serial.println("Device name: GBGC");
    Serial.println("Waiting for connection...");
}

// -------------------------
// Main loop
// -------------------------

void loop()
{
    if (!bleGamepad.isConnected())
    {
        delay(100);
        return;
    }

    // -------------------------
    // BUTTON A
    // -------------------------

    if (digitalRead(BUTTON_A) == LOW)
    {
        bleGamepad.press(BUTTON_1);
    }
    else
    {
        bleGamepad.release(BUTTON_1);
    }

    // -------------------------
    // BUTTON B
    // -------------------------

    if (digitalRead(BUTTON_B) == LOW)
    {
        bleGamepad.press(BUTTON_2);
    }
    else
    {
        bleGamepad.release(BUTTON_2);
    }

    // -------------------------
    // TEST JOYSTICK
    // -------------------------

    int rawX = analogRead(TEST_X);
    int rawY = analogRead(TEST_Y);

    // ESP32 ADC:
    // 0     -> -32767
    // 4095  -> +32767

    int x = map(rawX, 0, 4095, -32767, 32767);
    int y = map(rawY, 0, 4095, -32767, 32767);

    // Right stick:
    // RX = X
    // RY = Y

    bleGamepad.setAxes(
        0,      // Left X
        0,      // Left Y
        0,      // Z
        x,      // Right X / RX
        y,      // Right Y / RY
        0,      // RZ
        0,      // Slider 1
        0       // Slider 2
    );

    // -------------------------
    // Debug
    // -------------------------

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 500)
    {
        lastPrint = millis();

        Serial.print("X: ");
        Serial.print(x);

        Serial.print(" | Y: ");
        Serial.print(y);

        Serial.print(" | A: ");
        Serial.print(digitalRead(BUTTON_A) == LOW);

        Serial.print(" | B: ");
        Serial.println(digitalRead(BUTTON_B) == LOW);
    }

    delay(10);
}
