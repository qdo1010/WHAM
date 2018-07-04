# Overview

**1) In the past, we used only 1 Arduino Nano, and we have only 1 sensor, called MPU6050 that we use to collect movement information.**

* The user would put on a wristband that houses the Nano and MPU6050, and there would be a Bluetooth Module inside that send the movement info to an iPhone.

* We also have the wristband send those data to MATLAB, so we can plot what's going on, but that's through an USB cable.

**2) We then decided to integrate more sensors throughout the body, not just on the wrist, so we can capture more relevant info from the users.**

*Thus, we decided to make a Sensor Network on a T-Shirt, and our first attempt was one connected via I2C, inspired by this project: https://github.com/ResEnv/SensorTape*

* 1 Master would send out a query asking who is in the network sequentially, and the slave would respond with their ID by incrementing the count started at 0 from the Master.

* Slave Microcontroller then send all the data it gathers from its connected sensor to the master, and the Master would send those data on to a Computer via an USB cable or through Bluetooh.

* This was very time comsuming and complicated to pull off, but we got it to work,

* The code is now in the Master and Slave code, and everything works out fine.

*However, we soon realized it's very time consuming to connect those sensors with conductive threads, and it's very bulky and user-unfriendly with just regular wires. This would be extremely difficult to scale with limited resources.*

**3) We decided to move on to building a Mesh RF Network, and this is what we have been trying to work on ever since.**

*Basically, we just want a Small Button or Pin that users can buy and attach to anywhere on their body, and with multiple of those Button/Pins containing various sensors, we can have full body tracking at the user's convienence.*

* This would follow the same concept as part 2, where we would have 1 Master and multiple Slave Nodes. Except there will be no wire, and no sequential query from Master to automatically assign ID.

* Everything will be done through the air with a cheap RF module attached to each Sensor node.

* Each Slave will be hardcoded with an ID called NodeID, and they will constantly be broadcasting their data, similar to a Radio Station, and the Master will act as a Radio Receiver, tuning in to those data.

*With this approach, we can add as many sensors as we would like, and whereever we want, since we are not restricted to just a Sensors being wired up on a T-Shirt.*
