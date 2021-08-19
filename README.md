# MPU6050 Accelerometer and Gyroscope driver

## Functionalities

This driver provides functions to 
- Configure the **MPU6050** into one of **4 operation modes** ```FULL, FULL WITH ITERRUPTS, ACCELEROMETER ONLY, ACCELEROMETER ONLY WITH INTERRUPTS```
- Retreive data from the **MPU6050** module
- Put the **MPU6050** into sleep 
- And since all registers are defined, and a **write** and a **write and read** functions are provided to interact with the module, any additional functionality is easy to implement.

## TODO

- Every sensor needs calibration since no two modules are exactly the same, and they all come with inperfections, a calibration routine must be implemented.

- The read upon interrupt routine hasen't been implemented yet, although the "put into mode" functionality is ready.
