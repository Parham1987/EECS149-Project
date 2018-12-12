Self Balancing Vehicle with NRF52+Buckler
=====
##TX
The board reads data from analog joystick and update the BLE buffer every 20ms and send the data to the Reciver.
Using A0 as X_input, A1 as Y_input

##RX
The car would maintain it's balance when switch is on by moving in the correct direction to avoid tilt when we are near balance position the car would follow the BLE recived command updated every 20ms and move according to the recived parameters, if it enters a critical degree of tilt then it would ignore BLE and maintain it's balance first then continue using commands.
