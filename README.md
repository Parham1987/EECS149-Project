Self-Balancing Vehicle with NRF52+Buckler
=====
## TX
The board reads data from analog joystick and update the BLE buffer every 20ms and send the data to the Receiver.</Br>
Using A0 as X_input, A1 as Y_input

## RX
The car would maintain its balance when switch is on by moving in the correct direction to avoid tilt. It uses a PID method to calculate the necessary speed according to the digital accelerometer angle read using I2C and using a PWM it would make proper output to maintain the balance. In addition, when we are near balance position the car would follow the BLE received command updated every 20ms and move according to the received parameters, if it enters a critical degree of tilt then it would ignore BLE and maintain its balance first then continue using commands.</Br>
Using A0 as Enable1, A1 as Direction1, D0 as Enable2, D1 as Direction2
