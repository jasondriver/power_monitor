# power_monitor

This is a home automation project that monitors 2 Peacefair PZEM-004T Energy monitor devices.  Currently saves data to a
sqlite3 database on the disk and writes to cout.

### Serial communication

The PZEM-004 is equipped with TTL serial data communication interface, you can read and set the relevant parameters via the 
serial port.   To communicate with the PZEM-004 device you need to use a TTL to USB or RS232 on the monitoring computer. 

# include 
### for c++
```
apt-get install libsqlite3-dev 
```

