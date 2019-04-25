//#include <wiringPi.h>

#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "serial_bytestream.h"

using namespace std; 

// define max message length
#define MSG_LEN 7
//// define delay between sent packet and receive packet
//#define SEND_RECEIVE_DELAY 10000

//#include <uapi/linux/usbdevice_fs.h>
// the usbdevice library is not on Mac so here is the workaround..
#define USBDEVFS_RESET _IO('U', 20)

// time of day to turn gpio pin on/off
int threshhold_hour = 12;
int threshhold_minute = 30;

/* 
 *  Sets usb information using the open file descriptor
 *  help with set_interface_attribs and set_blocking from
 *  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 */ 
int set_interface_attribs (int fd, int speed, int parity) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        printf("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

/* 
 *  Sets blocking on open file descriptor
 */
int set_blocking (int fd, int should_block) {
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        printf("error %d from tggetattr", errno);
        return -1;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0) {
        printf("error %d setting term attributes", errno);
        return -1;
    }
    return 0;
}

/*
 *  Sends packet to open file descriptor of USB device
 *  Caller must make sure that the length of the packet is correct
 */
int send_packet(int fd, int size, uint8_t packet[]) {
    for(int i = 0; i < size; i++) {
        write (fd, &packet[i], 1);
        // sleep enough to transmit the 1 byte (#bytes+25)*100
        // receive 25:  approx 100 uS per char transmit
        usleep ((1 + 25) * 100);
    }
    return 0;
}

/*
 *  Recieves and returns packet from open file descriptor of USB device
 *  Caller must make sure that the length of the packet is correct
 */
int recieve_packet(int fd, int size, uint8_t *packet) {
    for(int i = 0; i < size; i++) {
        int n = read (fd, &packet[i], 1);
        if (n == 1)
            continue;
        else
            break;
        usleep (1000);
    }
    return 0;
}

/*
 *  Calculates checksum by adding together all of the bytes
 */
static uint8_t calc_checksum(int size, uint8_t packet[]) {
    uint8_t ret = 0;
    for(int i = 0; i < size; i++)
        ret += packet[i];
    return ret;
}

/*
 *  Sets the communication address of the device, needs to be of the form ###.###.###.### i.e. 192.168.1.1
 *  Caller must make sure that the length of the addr is 4 integers, and the address isn't already in use
 */
int set_com_addr(int fd, uint8_t addr[]) {
    // package address info for device
    uint8_t set_com_addr_cmd[] = {0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for(int i = 1; i < 5; i++)
        set_com_addr_cmd[i] = addr[i-1];

    // calculate checksum for packet using the first 6 bytes
    set_com_addr_cmd[6] = calc_checksum(6, set_com_addr_cmd);

    // send packet to device
    send_packet(fd, MSG_LEN, set_com_addr_cmd);
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof(buf));
    recieve_packet(fd, MSG_LEN, buf);
    usleep(10000);
    recieve_packet(fd, MSG_LEN, buf);
    return 0;
}

/* return time as vector<int>, firt element ret[0]  is hour (24hr format, 0-23)  second element ret[1]  is minutes (0-59)*/
vector<int> current_time_hour_minutes() {
    time_t raw_time = time(nullptr);
    vector<int> ret(2, 0);
    tm* t = localtime(&raw_time);
    ret[0] = t->tm_hour;
    ret[1] = t->tm_min; 
    return ret;
}

/* return date and time as string, works on linux */
string current_time_and_date() {
    time_t raw_time = time(nullptr);
    stringstream ss;
    ss << put_time(localtime(&raw_time), "%Y-%m-%d %X");
    return ss.str();
}

/* 1st argument is output string, need to use string c_str() to convert to c style char* for sqlite3 */
void return_formated_sql_insert_string(string* out, string date_time, int device_id, double voltage, double current, int power, int energy) {
    stringstream ss;
    ss << "INSERT INTO DAILY_POWER (DATE,DEVICE_ID,VOLTAGE,CURRENT,POWER,ENERGY) ";
    ss << "VALUES ('" << date_time << "', " << device_id << ", " << setw(5) << voltage;
    ss << ", " << setw(4) << current << ", " << power << ", " << energy << " );";
    
    *out = ss.str();

    return;
}

/* formatted print to cout for sql info */
void sql_cout_msg(string the_time, int id, double voltage, double current, int power, int energy) {
    cout << the_time << ", " << id << ", " << voltage << ", " << current << ", " << power << ", " << energy << endl;
    return;
}

/* sqlite3 exec callback function */
int callback(void *not_used, int argc, char **argv, char **az_col_name) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", az_col_name[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}