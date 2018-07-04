## Description

This contains code/attempts to combine RF network code with Sensor Data from the 9DOF IMU LSM9DS0

* We want to deploy RF24Mesh_Example to an Arduino Flora connected to a LSM9DS0, with an RF module

* We also want to deploy RF24Mesh_Example_Master to an Arduino Uno connected to a RF Module and a Bluetooth Module as well.

* Basically, RF24Mesh_Example will read 9 datapoints from the LSM9DS0 (acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, mag_x, mag_y, mag_z)

* Then, this code will send those datapoints to a Master via RF Network.

* The Master once receive those datapoint, will send it on via Bluetooth to a phone (Code for sending data to phone via Bluetooth can be found in Previous Code section)

## Note

For each Slave Code deployed, change the Slave ID so that they are not identical 

* #define nodeID 1

To deploy code on the Flora, use pin 6 and 12

To deploy on the Uno, change to pin 9 and 10

* RF24 radio(6, 12);

## Bugs
* RF Slave send a message to the Master. However, the data type expected is an uint32_t

* We want to send Accel and Gyro data, but those are float. Therefore, data sent might be corrupted due to datatype mismatched?
