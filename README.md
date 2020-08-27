# micropython-pybd-expansion
Micropython sources for a Custom PYBD Expansion PCB to be used with a PYBD-SF6-W4F2.

Compatible layout to be used with https://github.com/bensherlock/micropython-ota-updater

The custom PYBD Expansion board includes two I2C sensors: BME280 and LSM3030AGR, both powered by the secondary 3V3 regulator on the PYBD. I2C lines are X9 (SCL) and X10 (SDA). 

## NB
The library code included in this repository is currently working to an absolute minimum feature set required to get something meaningful from the sensors such that we can get units out to sea for field trials ASAP whilst working in the current global situation. It should be written in such a way that you could add functions to access the other registers should you wish to extend these source files and add interrupt capability to the LSM303AGR for example. 
