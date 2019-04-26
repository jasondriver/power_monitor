# Power Monitor

This is a home automation project that monitors up to 2 Peacefair PZEM-004T Energy monitor devices.  Currently saves data to a
sqlite3 database on the disk.  

### Serial communication

The PZEM-004 is equipped with TTL serial data communication interface, you can read and set the relevant parameters via the 
serial port.   To communicate with the PZEM-004 device you need to use a TTL to USB or RS232 on the monitoring computer. 

### Prerequisites
```
apt-get install libsqlite3-dev 
```

### Installing

Plug in your PZEM-004 to a ttl-to-usb device and plug into the usb ports.  Clone this project onto your raspberry pi.
```
git clone https://github.com/jasondriver/power_monitor
cd power_monitor
```

### make the database
```
sqlite3 bin/power_monitor.db < make_table.sql
```

### compile
```
make
```

### run
$ program  db  device0  device1  ...
Example with 2 devices.
```
cd bin
./output power_monitor.db /dev/ttyUSB0 /dev/ttyUSB1
```

You can also save to a csv by redirecting cout to a file
```
./output power_monitor.db /dev/ttyUSB0 /dev/ttyUSB1 > file.csv
```

### quit

Type 'q'+enter into terminal to quit the program.

### Example Output
```
starting at current time: 2019-01-26 13:55:27
Opened database successfully
2019-01-26 13:55:30, 0, 118.01, 10.48, 1228, 1473304
2019-01-26 13:55:30, 1, 118.02, 1.71, 200, 581427
```