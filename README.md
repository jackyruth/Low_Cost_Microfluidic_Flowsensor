# Low Cost Flowsensor Development

# Data to Sensor Binding
## Near Sensor Prototype (connecting tygon: 2mm)
Data: 10.txt, 20.txt ...
## Far Sensor Prototype (connecting tygon: 5mm)
Data: 30_farsensor.txt, 50_farsensor.txt ...

# TODO
[ ] Implement code on Arduino using TMP117 arduino library to make sure it's not Zephyr

# Progress Log
## 2022-01-18
[x] Implement Data Ready (Didn't solve glitches, also slows down program unneccessarily, I may remove this)
[x] Implement I2C Broadcast Reset for Synchronous Temperature Reading (Working, but didn't solve glitches)
