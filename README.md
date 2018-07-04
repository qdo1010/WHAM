# WHAM

## Project History
[What we did and are currently working on](https://github.com/qhdo1010/WHAM/tree/master/Previous%20Codes)


## Overview 
* [RF Network](https://github.com/qhdo1010/WHAM/tree/master/RF_Network) contains Code to connect multiple Arduinos to 1 via RF with RF24 Library. (This is Work in Progress)

* [LSM9DS0](https://github.com/qhdo1010/WHAM/tree/master/LSM9DS0) contains code to read data from the current 9DOF IMU being used. (This is for reference)

* [Previous Codes](https://github.com/qhdo1010/WHAM/tree/master/Previous%20Codes) contain Interfacing with Bluetooth Module, Sending data to Phone, Sending Data to Matlab, and Code for previous IMU no longer being used 

## Current Progress

* Currently, all the code to read 9DOF IMU data, inititiating and receiving RF connection are there.

* 9DOF IMU Data can be sent through RF, but due to some bugs described later inside the RF Network Folder, the data read from master are different from what is expected.

* Once clean data can be received by the Master, we want to integrate Bluetooth Code, to send those datapoints from Master to a SmartPhone via Bluetooth.

* Once all the steps above are completed, we can then add more sensors to collect more useful data, instead of just Motion/Movement Info
