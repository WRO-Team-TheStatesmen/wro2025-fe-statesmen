# Commit - 20 August 2025
This commit updates the ESP32 and RPI communication protocol. Now, there is an ultrarsonic sensor for the front and also an IR sensor.
5 values are sent from the ESP32 to the Raspberry Pi:
1. Angle (BNO055)
2. Left Distance (TF Luna)
3. Front Distance (Ultrasonic)
4. Right Distance (TF Luna)
5. IR Value (0 or 1)