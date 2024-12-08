// Basic demo for accelerometer readings from Adafruit MPU6050

// ESP32 Guide: https://RandomNerdTutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/
// ESP8266 Guide: https://RandomNerdTutorials.com/esp8266-nodemcu-mpu-6050-accelerometer-gyroscope-arduino/
// Arduino Guide: https://RandomNerdTutorials.com/arduino-mpu-6050-accelerometer-gyroscope/
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <pins_arduino.h>
#include <BleMouse.h>
#include <LowFilter.hpp>
#include <math.h>

BleMouse bleMouse("ESP32 MOUSE by SHJ", "Espressif", 100);
Adafruit_MPU6050 mpu;

// Sensitivity settings for mouse control
float sensitivity = 20.0;  // Adjust mouse movement speed
float previousAccelerationX = 0, previousAccelerationY = 0, previousAccelerationZ = 0;
float velocityX = 0, velocityY = 0, velocityZ = 0;
float deltaTime = 0.05; // Time interval in seconds, adjust based on loop delay (time between sensor reads)

void init_mpu()
{
  // Try to initialize!
  if (!mpu.begin())
  {
    Serial0.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial0.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
}

void setup(void)
{
  Serial0.begin(115200);
  Serial0.println("Starting BLE work!");
  bleMouse.begin();
  init_mpu();
}

void acc2Mouse(BleMouse &mouse, float dx, float dy, float dz);

void accGyro2Mouse(BleMouse &mouse, float dx, float gx, float dz);

void normalAttack(BleMouse &mouse, float gy);

void loop()
{
  sensors_event_t a, g, temp;
  sensors_event_t ao, go, tempo;

  if (bleMouse.isConnected())
  {
    mpu.getEvent(&a, &g, &temp);
    a.acceleration.x = filter(ao.acceleration.x, a.acceleration.x);
    a.acceleration.y = filter(ao.acceleration.y, a.acceleration.y);
    a.acceleration.z = filter(ao.acceleration.z, a.acceleration.z);
    g.gyro.x = filter(go.gyro.x, g.gyro.x);
    g.gyro.y = filter(go.gyro.y, g.gyro.y);
    g.gyro.z = filter(go.gyro.z, g.gyro.z);

    // Calculate the difference in acceleration (delta) for mouse movement
    float deltaX = a.acceleration.x - previousAccelerationX;
    float deltaY = a.acceleration.y - previousAccelerationY;
    float deltaZ = a.acceleration.z - previousAccelerationZ;

    // Simulate the mouse movement using the acceleration change (delta)
    // acc2Mouse(bleMouse, deltaX, deltaY, deltaZ);
    accGyro2Mouse(bleMouse, g.gyro.z, g.gyro.x, deltaZ);
    normalAttack(bleMouse,g.gyro.y);

    // Serial0.print("Delta Acceleration X: ");
    // Serial0.print(deltaX);
    // Serial0.print(", Y: ");
    // Serial0.print(deltaY);
    // Serial0.print(", Z: ");
    // Serial0.print(deltaZ);
    // Serial0.println(" m/s^2");

    // Serial0.print("Delta Rotation X: ");
    // Serial0.print(g.gyro.x);
    // Serial0.print(", Y: ");
    // Serial0.print(g.gyro.y);
    // Serial0.print(", Z: ");
    // Serial0.print(g.gyro.z);
    // Serial0.println(" m/s^2");

    Serial0.println("");

    ao = a;
    go = g;
    tempo = temp;

       // Save the current acceleration values for the next loop
    previousAccelerationX = a.acceleration.x;
    previousAccelerationY = a.acceleration.y;
    previousAccelerationZ = a.acceleration.z;
  }
  delay(deltaTime * 500);
}

void acc2Mouse(BleMouse &mouse, float dx, float dy, float dz) {
  // Use the difference in acceleration (delta) to control mouse movement
  // dx, dy, dz represent changes in acceleration in x, y, and z axes

  int moveX = int(dx * sensitivity);
  int moveY = int(dy * sensitivity);
  int moveWheel = int(dz * sensitivity); // Optional: use z-axis for scroll

  mouse.move(-moveX, moveY, 0, 0); // Move the mouse
}

void accGyro2Mouse(BleMouse &mouse, float gz, float gx, float dz) {
  // Use the difference in acceleration (delta) to control mouse movement
  // dx, dy, dz represent changes in acceleration in x, y, and z axes

  int moveX = int(gz * sensitivity);
  int moveY = int(gx * sensitivity);
  int moveWheel = int(dz * sensitivity); // Optional: use z-axis for scroll
  if (abs(gz) < 0.1)
  {
    moveX = 0;
  }

  if (abs(gx) < 0.1)
  {
    moveY = 0;
  }

  mouse.move(-moveX, -moveY, 0, 0); // Move the mouse
}

void normalAttack(BleMouse &mouse, float gy){
  Serial0.print("Rotation Y: ");
  Serial0.print(gy);
  if (gy > 7)
  {
    mouse.click(MOUSE_LEFT);
  }
}
