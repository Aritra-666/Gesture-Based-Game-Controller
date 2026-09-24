#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>
#define SDA_PIN 27
#define SCL_PIN 14
#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define LOOP_PERIOD_US 5000UL // 200 Hz
int16_t AccX, AccY, AccZ;
int16_t GyroX, GyroY, GyroZ;
int16_t MagX = 0, MagY = 0, MagZ = 0;
float gyroBias[3] = {0, 0, 0}; // Saved gyro bias
float gyroBiasOnline[3] = {0, 0, 0}; // Online correction
const float ZETA            = 0.0076f;
const float MAX_ONLINE_BIAS = 0.10f;
bool learnBias = false;
float magOffset[3] = {0, 0, 0}; // Hard-iron offset
float magScale[3]  = {1, 1, 1}; // Soft-iron scale
bool magCalibrated = false;
Preferences prefs;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float beta = 1.0f;
unsigned long previousMicros = 0;
unsigned long startMs = 0;
void mapMagToImuFrame(float &x, float &y, float &z)
{
    float rx = x;
    float ry = y;
    float rz = z;
    x = rx;
    y = ry;
    z = rz;
}
void writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}
bool readRegisters(
    uint8_t address,
    uint8_t reg,
    uint8_t count,
    uint8_t *data)
{
    Wire.beginTransmission(address);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    uint8_t received =
    Wire.requestFrom(address, (uint8_t)count);
    if (received != count)
        return false;
    for (uint8_t i = 0; i < count; i++)
        data[i] = Wire.read();
    return true;
}
bool initMPU()
{
    uint8_t whoami;
    if (!readRegisters(MPU_ADDR, 0x75, 1, &whoami))
        return false;
    Serial.print("MPU WHO_AM_I: 0x");
    Serial.println(whoami, HEX);
    writeRegister(MPU_ADDR, 0x6B, 0x01);
    delay(50);
    writeRegister(MPU_ADDR, 0x1A, 0x03);
    writeRegister(MPU_ADDR, 0x1C, 0x00);
    writeRegister(MPU_ADDR, 0x1B, 0x00);
    delay(100);
    return true;
}
bool readMPU()
{
    uint8_t d[14];
    if (!readRegisters(MPU_ADDR, 0x3B, 14, d))
        return false;
    AccX =
    (int16_t)((d[0] << 8) | d[1]);
    AccY =
    (int16_t)((d[2] << 8) | d[3]);
    AccZ =
    (int16_t)((d[4] << 8) | d[5]);
    GyroX =
    (int16_t)((d[8] << 8) | d[9]);
    GyroY =
    (int16_t)((d[10] << 8) | d[11]);
    GyroZ =
    (int16_t)((d[12] << 8) | d[13]);
    return true;
}
bool initQMC()
{
    writeRegister(QMC_ADDR, 0x0A, 0x80);
    delay(10);
    writeRegister(QMC_ADDR, 0x0B, 0x01);
    writeRegister(QMC_ADDR, 0x09, 0x1D);
    delay(100);
    return true;
}
bool readQMC()
{
    uint8_t status;
    if (!readRegisters(QMC_ADDR, 0x06, 1, &status))
        return false;
    if (!(status & 0x01))
        return false;
    uint8_t d[6];
    if (!readRegisters(QMC_ADDR, 0x00, 6, d))
        return false;
    MagX =
    (int16_t)(d[0] | (d[1] << 8));
    MagY =
    (int16_t)(d[2] | (d[3] << 8));
    MagZ =
    (int16_t)(d[4] | (d[5] << 8));
    return true;
}
#define CAL_SAMPLES 800
#define CAL_WINDOW  50
#define CAL_STEP    10
#define GOOD_RANGE  250
void saveGyroBias()
{
    prefs.begin("imu", false);
    prefs.putBool("gCal", true);
    prefs.putBytes(
        "gBias",
        gyroBias,
        sizeof(gyroBias)
    );
    prefs.end();
}
void foldAndSaveGyroBias()
{
    const float radToLsb =
    (180.0f / PI) * 131.0f;
    for (int k = 0; k < 3; k++)
    {
        gyroBias[k] +=
        gyroBiasOnline[k] * radToLsb;
        gyroBiasOnline[k] = 0;
    }
    saveGyroBias();
    Serial.println("Gyro bias saved.");
}
void calibrateGyro()
{
    static int16_t buf[CAL_SAMPLES][3];
    float saved[3] = {0, 0, 0};
    prefs.begin("imu", true);
    bool haveSaved =
    prefs.getBool("gCal", false);
    if (haveSaved)
    {
        prefs.getBytes(
            "gBias",
            saved,
            sizeof(saved)
        );
    }
    prefs.end();
    Serial.println(
        "Gyro calibration: hold the sensor as steady "
        "as you comfortably can (~4 s)..."
    );
    int n = 0;
    while (n < CAL_SAMPLES)
    {
        if (readMPU())
        {
            buf[n][0] = GyroX;
            buf[n][1] = GyroY;
            buf[n][2] = GyroZ;
            n++;
        }
        delay(5);
    }
    float bestRange = 1e9f;
    float bestMean[3] =
    {
        0,
        0,
        0
    };
    for (
        int start = 0;
    start + CAL_WINDOW <= CAL_SAMPLES;
    start += CAL_STEP)
    {
        long sum[3] =
        {
            0,
            0,
            0
        };
        int16_t lo[3] =
        {
            32767,
            32767,
            32767
        };
        int16_t hi[3] =
        {
            -32768,
            -32768,
            -32768
        };
        for (
            int i = start;
        i < start + CAL_WINDOW;
        i++)
        {
            for (int k = 0; k < 3; k++)
            {
                int16_t v = buf[i][k];
                sum[k] += v;
                if (v < lo[k])
                    lo[k] = v;
                if (v > hi[k])
                    hi[k] = v;
            }
        }
        float worst = 0;
        for (int k = 0; k < 3; k++)
        {
            if ((hi[k] - lo[k]) > worst)
                worst = hi[k] - lo[k];
        }
        if (worst < bestRange)
        {
            bestRange = worst;
            for (int k = 0; k < 3; k++)
            {
                bestMean[k] =
                (float)sum[k] / CAL_WINDOW;
            }
        }
    }
    if (bestRange >= GOOD_RANGE && haveSaved)
    {
        for (int k = 0; k < 3; k++)
            gyroBias[k] = saved[k];
        Serial.println(
            "Too much motion - using SAVED gyro bias "
            "(filter will refine it)."
        );
    }
    else
    {
        for (int k = 0; k < 3; k++)
            gyroBias[k] = bestMean[k];
        if (bestRange < GOOD_RANGE)
        {
            saveGyroBias();
            Serial.println(
                "Gyro bias captured and saved."
            );
        }
        else
        {
            Serial.println(
                "Gyro bias captured with motion - "
                "filter will refine it while you move."
            );
        }
    }
    for (int k = 0; k < 3; k++)
        gyroBiasOnline[k] = 0;
    Serial.print("Gyro bias (LSB): ");
    Serial.print(gyroBias[0]);
    Serial.print(", ");
    Serial.print(gyroBias[1]);
    Serial.print(", ");
    Serial.println(gyroBias[2]);
}
void saveMagCalibration()
{
    prefs.begin("imu", false);
    prefs.putBool("magCal", true);
    prefs.putBytes(
        "mOff",
        magOffset,
        sizeof(magOffset)
    );
    prefs.putBytes(
        "mScl",
        magScale,
        sizeof(magScale)
    );
    prefs.end();
}
void loadMagCalibration()
{
    prefs.begin("imu", true);
    if (prefs.getBool("magCal", false))
    {
        prefs.getBytes(
            "mOff",
            magOffset,
            sizeof(magOffset)
        );
        prefs.getBytes(
            "mScl",
            magScale,
            sizeof(magScale)
        );
        magCalibrated = true;
        Serial.println(
            "Loaded saved magnetometer calibration."
        );
    }
    else
    {
        Serial.println(
            "No magnetometer calibration saved - "
            "send 'c' to calibrate."
        );
    }
    prefs.end();
}
void calibrateMag()
{
    Serial.println(
        "MAG CAL: slowly rotate the sensor through "
        "ALL orientations for 20 s..."
    );
    float lo[3] =
    {
        1e9f,
        1e9f,
        1e9f
    };
    float hi[3] =
    {
        -1e9f,
        -1e9f,
        -1e9f
    };
    unsigned long start =
    millis();
    while (millis() - start < 20000)
    {
        if (readQMC())
        {
            float m[3] =
            {
                (float)MagX,
                (float)MagY,
                (float)MagZ
            };
            for (int k = 0; k < 3; k++)
            {
                if (m[k] < lo[k])
                    lo[k] = m[k];
                if (m[k] > hi[k])
                    hi[k] = m[k];
            }
        }
        delay(5);
    }
    float radius[3];
    float avg = 0;
    for (int k = 0; k < 3; k++)
    {
        magOffset[k] =
        (hi[k] + lo[k]) * 0.5f;
        radius[k] =
        (hi[k] - lo[k]) * 0.5f;
        avg += radius[k];
    }
    avg /= 3.0f;
    for (int k = 0; k < 3; k++)
    {
        if (radius[k] < 1.0f)
        {
            Serial.println(
                "MAG CAL FAILED (not enough rotation). "
                "Try again."
            );
            return;
        }
        magScale[k] =
        avg / radius[k];
    }
    magCalibrated = true;
    saveMagCalibration();
    Serial.print("Offsets: ");
    Serial.print(magOffset[0]);
    Serial.print(", ");
    Serial.print(magOffset[1]);
    Serial.print(", ");
    Serial.println(magOffset[2]);
    Serial.print("Scales:  ");
    Serial.print(magScale[0]);
    Serial.print(", ");
    Serial.print(magScale[1]);
    Serial.print(", ");
    Serial.println(magScale[2]);
    Serial.println(
        "MAG CAL DONE (saved)."
    );
}
void MadgwickUpdate(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float mx,
    float my,
    float mz,
    bool accelValid,
    bool magValid,
    float dt)
{
    float recipNorm;
    float s0 = 0;
    float s1 = 0;
    float s2 = 0;
    float s3 = 0;
    bool haveGradient = false;
    if (accelValid)
    {
        recipNorm =
        1.0f /
        sqrtf(
            ax * ax +
            ay * ay +
            az * az
        );
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        float q0q0 = q0 * q0;
        float q0q1 = q0 * q1;
        float q0q2 = q0 * q2;
        float q0q3 = q0 * q3;
        float q1q1 = q1 * q1;
        float q1q2 = q1 * q2;
        float q1q3 = q1 * q3;
        float q2q2 = q2 * q2;
        float q2q3 = q2 * q3;
        float q3q3 = q3 * q3;
        float _2q0 = 2.0f * q0;
        float _2q1 = 2.0f * q1;
        float _2q2 = 2.0f * q2;
        float _2q3 = 2.0f * q3;
        float fx =
        2.0f * (q1q3 - q0q2) - ax;
        float fy =
        2.0f * (q0q1 + q2q3) - ay;
        float fz =
        1.0f - 2.0f * (q1q1 + q2q2) - az;
        bool useMag = false;
        if (magValid)
        {
            float mn =
            sqrtf(
                mx * mx +
                my * my +
                mz * mz
            );
            if (mn > 0.0f)
            {
                recipNorm = 1.0f / mn;
                mx *= recipNorm;
                my *= recipNorm;
                mz *= recipNorm;
                useMag = true;
            }
        }
        if (useMag)
        {
            float _2q0mx =
            2.0f * q0 * mx;
            float _2q0my =
            2.0f * q0 * my;
            float _2q0mz =
            2.0f * q0 * mz;
            float _2q1mx =
            2.0f * q1 * mx;
            float hx =
            mx * q0q0
            - _2q0my * q3
            + _2q0mz * q2
            + mx * q1q1
            + _2q1 * my * q2
            + _2q1 * mz * q3
            - mx * q2q2
            - mx * q3q3;
            float hy =
            _2q0mx * q3
            + my * q0q0
            - _2q0mz * q1
            + _2q1mx * q2
            - my * q1q1
            + my * q2q2
            + _2q2 * mz * q3
            - my * q3q3;
            float _2bx =
            sqrtf(
                hx * hx +
                hy * hy
            );
            float _2bz =
            -_2q0mx * q2
            + _2q0my * q1
            + mz * q0q0
            + _2q1mx * q3
            - mz * q1q1
            + _2q2 * my * q3
            - mz * q2q2
            + mz * q3q3;
            float _4bx =
            2.0f * _2bx;
            float _4bz =
            2.0f * _2bz;
            float ex =
            _2bx *
            (0.5f - q2q2 - q3q3)
            + _2bz *
            (q1q3 - q0q2)
            - mx;
            float ey =
            _2bx *
            (q1q2 - q0q3)
            + _2bz *
            (q0q1 + q2q3)
            - my;
            float ez =
            _2bx *
            (q0q2 + q1q3)
            + _2bz *
            (0.5f - q1q1 - q2q2)
            - mz;
            s0 =
            -_2q2 * fx
            + _2q1 * fy
            - _2bz * q2 * ex
            + (-_2bx * q3 + _2bz * q1) * ey
            + _2bx * q2 * ez;
            s1 =
            _2q3 * fx
            + _2q0 * fy
            - 4.0f * q1 * fz
            + _2bz * q3 * ex
            + (_2bx * q2 + _2bz * q0) * ey
            + (_2bx * q3 - _4bz * q1) * ez;
            s2 =
            -_2q0 * fx
            + _2q3 * fy
            - 4.0f * q2 * fz
            + (-_4bx * q2 - _2bz * q0) * ex
            + (_2bx * q1 + _2bz * q3) * ey
            + (_2bx * q0 - _4bz * q2) * ez;
            s3 =
            _2q1 * fx
            + _2q2 * fy
            + (-_4bx * q3 + _2bz * q1) * ex
            + (-_2bx * q0 + _2bz * q2) * ey
            + _2bx * q1 * ez;
        }
        else
        {
            s0 =
            -_2q2 * fx
            + _2q1 * fy;
            s1 =
            _2q3 * fx
            + _2q0 * fy
            - 4.0f * q1 * fz;
            s2 =
            -_2q0 * fx
            + _2q3 * fy
            - 4.0f * q2 * fz;
            s3 =
            _2q1 * fx
            + _2q2 * fy;
        }
        recipNorm =
        sqrtf(
            s0 * s0 +
            s1 * s1 +
            s2 * s2 +
            s3 * s3
        );
        if (recipNorm > 0.0f)
        {
            recipNorm = 1.0f / recipNorm;
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;
            haveGradient = true;
            if (learnBias)
            {
                float wex =
                2.0f *
                (
                    q0 * s1
                    - q1 * s0
                    - q2 * s3
                    + q3 * s2
                );
                float wey =
                2.0f *
                (
                    q0 * s2
                    + q1 * s3
                    - q2 * s0
                    - q3 * s1
                );
                float wez =
                2.0f *
                (
                    q0 * s3
                    - q1 * s2
                    + q2 * s1
                    - q3 * s0
                );
                gyroBiasOnline[0] =
                constrain(
                    gyroBiasOnline[0]
                    + wex * dt * ZETA,
                    -MAX_ONLINE_BIAS,
                    MAX_ONLINE_BIAS
                );
                gyroBiasOnline[1] =
                constrain(
                    gyroBiasOnline[1]
                    + wey * dt * ZETA,
                    -MAX_ONLINE_BIAS,
                    MAX_ONLINE_BIAS
                );
                gyroBiasOnline[2] =
                constrain(
                    gyroBiasOnline[2]
                    + wez * dt * ZETA,
                    -MAX_ONLINE_BIAS,
                    MAX_ONLINE_BIAS
                );
            }
        }
    }
    gx -= gyroBiasOnline[0];
    gy -= gyroBiasOnline[1];
    gz -= gyroBiasOnline[2];
    float qDot1 =
    0.5f *
    (
        -q1 * gx
        - q2 * gy
        - q3 * gz
    );
    float qDot2 =
    0.5f *
    (
        q0 * gx
        + q2 * gz
        - q3 * gy
    );
    float qDot3 =
    0.5f *
    (
        q0 * gy
        - q1 * gz
        + q3 * gx
    );
    float qDot4 =
    0.5f *
    (
        q0 * gz
        + q1 * gy
        - q2 * gx
    );
    if (haveGradient)
    {
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;
    recipNorm =
    1.0f /
    sqrtf(
        q0 * q0 +
        q1 * q1 +
        q2 * q2 +
        q3 * q3
    );
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}
void getEuler(
    float &yaw,
    float &pitch,
    float &roll)
{
    yaw =
    atan2f(
        2.0f *
        (q0 * q3 + q1 * q2),
           1.0f -
           2.0f *
           (q2 * q2 + q3 * q3)
    )
    * 180.0f / PI;
    pitch =
    asinf(
        constrain(
            2.0f *
            (q0 * q2 - q3 * q1),
                  -1.0f,
                  1.0f
        )
    )
    * 180.0f / PI;
    roll =
    atan2f(
        2.0f *
        (q0 * q1 + q2 * q3),
           1.0f -
           2.0f *
           (q1 * q1 + q2 * q2)
    )
    * 180.0f / PI;
}
void printStartupMessage()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("    GBGC MADGWICK 9-DOF TEST");
    Serial.println("================================");
    Serial.println(
        "Serial: 'c' = calibrate mag, "
        "'g' = redo gyro capture, "
        "'s' = save learned gyro bias"
    );
}
void initI2C()
{
    Wire.begin(
        SDA_PIN,
        SCL_PIN
    );
    Wire.setClock(400000);
    delay(100);
}
void handleSerialCommands()
{
    if (Serial.available())
    {
        char c = Serial.read();
        if (c == 'c')
        {
            calibrateMag();
            previousMicros =
            micros();
            startMs =
            millis();
        }
        else if (c == 'g')
        {
            calibrateGyro();
            previousMicros =
            micros();
            startMs =
            millis();
        }
        else if (c == 's')
        {
            foldAndSaveGyroBias();
        }
    }
}
bool updateSensorFusion()
{
    unsigned long now =
    micros();
    if (
        now - previousMicros
        < LOOP_PERIOD_US
    )
    {
        return false;
    }
    float dt =
    (now - previousMicros)
    / 1000000.0f;
    previousMicros =
    now;
    if (
        dt <= 0 ||
        dt > 0.1f
    )
    {
        dt =
        LOOP_PERIOD_US
        / 1000000.0f;
    }
    if (!readMPU())
        return false;
    readQMC();
    float ax =
    AccX / 16384.0f;
    float ay =
    AccY / 16384.0f;
    float az =
    AccZ / 16384.0f;
    float accNorm =
    sqrtf(
        ax * ax +
        ay * ay +
        az * az
    );
    const float DEG2RAD =
    PI / 180.0f;
    float gx =
    (
        (GyroX - gyroBias[0])
        / 131.0f
    )
    * DEG2RAD;
    float gy =
    (
        (GyroY - gyroBias[1])
        / 131.0f
    )
    * DEG2RAD;
    float gz =
    (
        (GyroZ - gyroBias[2])
        / 131.0f
    )
    * DEG2RAD;
    float mx =
    (MagX - magOffset[0])
    * magScale[0];
    float my =
    (MagY - magOffset[1])
    * magScale[1];
    float mz =
    (MagZ - magOffset[2])
    * magScale[2];
    mapMagToImuFrame(
        mx,
        my,
        mz
    );
    bool accelValid =
    (
        accNorm > 0.8f &&
        accNorm < 1.2f
    );
    bool magValid =
    magCalibrated;
    bool converging =
    (
        millis() - startMs
        < 3000
    );
    beta =
    converging
    ? 1.0f
    : 0.05f;
    learnBias =
    !converging;
    MadgwickUpdate(
        gx,
        gy,
        gz,
        ax,
        ay,
        az,
        mx,
        my,
        mz,
        accelValid,
        magValid,
        dt
    );
    return true;
}
void outputOrientation()
{
    static unsigned long lastPrint = 0;
    if (
        millis() - lastPrint
        < 50
    )
    {
        return;
    }
    lastPrint =
    millis();
    float yaw;
    float pitch;
    float roll;
    getEuler(
        yaw,
        pitch,
        roll
    );
    Serial.print("Yaw: ");
    Serial.print(yaw, 2);
    Serial.print(" | Pitch: ");
    Serial.print(pitch, 2);
    Serial.print(" | Roll: ");
    Serial.print(roll, 2);
    Serial.print("\n");
}
void setup()
{
    Serial.begin(115200);
    delay(1000);
    printStartupMessage();
    initI2C();
    Serial.println(
        initMPU()
        ? "MPU OK"
        : "MPU ERROR"
    );
    Serial.println(
        initQMC()
        ? "QMC OK"
        : "QMC ERROR"
    );
    loadMagCalibration();
    calibrateGyro();
    previousMicros =
    micros();
    startMs =
    millis();
}
void loop()
{
    handleSerialCommands();
    if (!updateSensorFusion())
        return;
    outputOrientation();
}
