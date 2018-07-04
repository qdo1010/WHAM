## Note

To deploy code on the Flora, use pin 6 and 12

To deploy on the Uno, change to pin 9 and 10

RF24 radio(6, 12);

## Bugs
RF Slave send a message to the Master.
However, the data type expected is an uint32_t
We want to send Accel and Gyro data, but those are float
Therefore, data sent might be corrupted due to datatype mismatched?
