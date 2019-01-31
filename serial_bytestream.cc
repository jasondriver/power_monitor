#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <sqlite3.h> 
#include <stdint.h>
#include <string.h>
#include <sstream>

#include <ctime>
#include <iostream>
#include <iomanip>
#include <signal.h>
#include <sys/time.h>

using namespace std; 

// define max message length
#define MSG_LEN 7
// define delay between sent packet and receive packet
#define SEND_RECEIVE_DELAY 10000

//#include <uapi/linux/usbdevice_fs.h>
// the usbdevice library is not on Mac so here is the workaround..
#define USBDEVFS_RESET _IO('U', 20)

// global predefined packets to be sent for device by manufacturer
uint8_t READ_VOLTAGE[] = {0xB0, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1A};
uint8_t READ_CURRENT[] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};
uint8_t READ_WATTAGE[] = {0XB2, 0XC0, 0XA8, 0X01, 0X01, 0X00, 0X1C};
uint8_t READ_ENERGY[] = {0xB3, 0XC0, 0XA8, 0X01, 0X01, 0X00, 0X1D};

/*
 *  Serial class for dealing with usb connections
 */
class Serial {
    private:
        // variables
        char* port;
        int baudrate;
        int parity;
        int fd;
        uint8_t com_addr[4] = {0x00, 0x00, 0x00, 0x00};

        // init messages for construct of predefined packets set by the device manufacturer to pass to the device
        uint8_t read_voltage[MSG_LEN] = {0xB0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint8_t read_current[MSG_LEN] = {0xB1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        uint8_t read_wattage[MSG_LEN] = {0XB2, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        uint8_t read_energy[MSG_LEN] = {0xB3, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00};
        uint8_t set_addr_packet[MSG_LEN] = {0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

        /* helper functions */
        int _set_interface_attribs (int speed, int parity);
        int _set_blocking (int should_block);
        void _set_packet_helper(int size, int offset, uint8_t const get_addr[], uint8_t set_addr[]);
        uint8_t _calc_checksum(int size, uint8_t const packet[]);
        double _convert_voltage(uint8_t packet[]);
        double _convert_current(uint8_t packet[]);
        int _hex_to_int(int size, int offset, uint8_t packet[]);
        int _convert_power(uint8_t packet[]);
        int _convert_energy(uint8_t packet[]);
    public:
        Serial(char* p, int b = B9600, int pa = 0);
        ~Serial();

        void open_port(char* p);
        void close_port();
        void set_packets(int size, uint8_t addr[]);
        int send_packet(int size, uint8_t packet[]);
        int recieve_packet(int size, uint8_t *packet);

        int receive_power();
        double receive_voltage();
        double receive_current();
        int receive_energy();
        void set_com_addr();
        void line_wait();

        int print_packet(int size, uint8_t packet[]);
        void print_com_addr();
        void print_all();

	int get_fd();
};

/*
 *  Constructor
 */
Serial::Serial(char* p, int b, int pa) {
    port = p;
    baudrate = b;
    parity = pa;

    int f = open (port, O_RDWR | O_NOCTTY | O_SYNC);

    if (f < 0) {
            printf("error %d opening %s: %s", errno, port, strerror (errno));
            f = -1;
    }
    else {
        fd = f;
        _set_interface_attribs (b, pa);
        _set_blocking (0);
    }
}

/* 
 *  Deconstructor
 */
Serial::~Serial() {
    return;
}

/*
 *  Setters
 */
/* opens port */
void Serial::open_port(char* p) {
    port = p;
    int f = open (p, O_RDWR | O_NOCTTY | O_SYNC);

    if (f < 0) {
            printf("error %d opening %s: %s", errno, port, strerror (errno));
            f = -1;
    }
    else
        fd = f;
    return;
}

/* closes the port if open */
void Serial::close_port() {
    if (fd != -1)
        close(fd);
    return;
}

/* helper that sets a packet with the device address */
void Serial::_set_packet_helper(int size, int offset, uint8_t const get_addr[], uint8_t set_addr[]) {
    for(int i = offset; i < offset + size; i++)
        set_addr[i] = get_addr[i-1];
    set_addr[MSG_LEN - 1] = _calc_checksum(MSG_LEN, set_addr); 
    return;
}

/* sets all sendable packets with the device address */
void Serial::set_packets(int size, uint8_t addr[]) {
    for(int i = 0; i < size; i++)
        com_addr[i] = addr[i];
    _set_packet_helper(size, 1, addr, read_voltage);
    _set_packet_helper(size, 1, addr, read_current);
    _set_packet_helper(size, 1, addr, read_wattage);
    _set_packet_helper(size, 1, addr, read_energy);
    _set_packet_helper(size, 1, addr, set_addr_packet);
    return;
}

/* 
 *  Serial helper functions
 */
/* 
 *  Sets usb information using the open file descriptor
 *  Thanks to wallyk for the set_interface_attribs and set_blocking methods from this post on stack overflow
 *  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 *  added some slight modifications to them
 */
int Serial::_set_interface_attribs (int speed, int parity) {
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

/* Sets blocking on open file descriptor */
int Serial::_set_blocking (int should_block) {
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

/* calculates checksum by device standards, sum all preceeding packets allowing overflow of uint for checksum*/
uint8_t Serial::_calc_checksum(int size, uint8_t const packet[]) {
    uint8_t ret = 0;
    for(int i = 0; i < size; i++) {
        ret += packet[i];
    }
    return ret;
}

/*  Convert voltage packet to voltage value, i.e. voltage = packet[1] packet[2] . packet[3] */
double Serial::_convert_voltage(uint8_t packet[]) {
    int voltage_int = packet[1] * 100 + packet[2];
    double voltage_decimal = (double) packet[3] /100.0;
    double ret = (double) voltage_int + voltage_decimal;
    return ret;
}

/* Convert current packet to value */
double Serial::_convert_current(uint8_t packet[]) {
    return (double) packet[2] + ((double) packet[3] / 100.0);
}

/*  Helper function for convert_power and convert_energy
 *  size = how many elements to use, offset = where in the array to start, packet = array of #s */
int Serial::_hex_to_int(int size, int offset, uint8_t packet[]) {
    uint8_t* arr =  (uint8_t*) malloc(2 * size * sizeof(uint8_t));
    if(!arr) {
        printf("error %d from malloc", errno);
    return -1;
    }
    int j = offset;
    for (int i = 0; i < 2 * size; i++){
        if((i % 2) == 0) {
            arr[i] = (packet[j] & 0xF0) >> 4;
    } else {
            arr[i] = packet[j] & 0x0F;
            j++;
    }
    }

    int base = 1;
    int ret = 0;

    for (int i = (2 * size) - 1; i > -1; i--) {
        ret += arr[i] * base;
    base *= 16;
    }

    free(arr);
    return ret;
}

/*  Convert average power packet to power value, i.e. power = (packet[1] packet[2])base16 */
int Serial::_convert_power(uint8_t packet[]) {
    return _hex_to_int(2, 1, packet);
}

/* Convert energy packet to value */
int Serial::_convert_energy(uint8_t packet[]) {
    return _hex_to_int(3, 1, packet);
}

/* 
 * Main send methods 
 */
/* Sends packet to open file descriptor of USB device */
int Serial::send_packet(int size, uint8_t packet[]) {
    for(int i = 0; i < size; i++) {
        write (fd, &packet[i], 1);
        // sleep enough to transmit the 1 byte (#bytes+25)*100
        // receive 25:  approx 100 uS per char transmit
        usleep ((1 + 25) * 100);
    }
    return 0;
}

/* Recieves and returns packet from open file descriptor of USB device */
int Serial::recieve_packet(int size, uint8_t *packet) {
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
 * Send the receive packet (needs to run set_address first), recieves device message, and decodes 
 * Output: Power in W 
 */
int Serial::receive_power() {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(MSG_LEN, READ_WATTAGE);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(MSG_LEN, buf);
    return _convert_power(buf);
}

/* 
 * Send the receive packet (needs to run set_address first), recieves device message, and decodes 
 * Output: Voltage in V 
 */
double Serial::receive_voltage() {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(MSG_LEN, READ_VOLTAGE);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(MSG_LEN, buf);
    return _convert_voltage(buf);
}

/* 
 * Send the receive packet (needs to run set_address first), recieves device message, and decodes 
 * Output: Current in A 
 */
double Serial::receive_current() {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(MSG_LEN, READ_CURRENT);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(MSG_LEN, buf);
    return _convert_current(buf);
}

/* 
 * Send the receive packet (needs to run set_address first), recieves device message, and decodes 
 * Output: Energy in J 
 */
int Serial::receive_energy() {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(MSG_LEN, READ_ENERGY);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(MSG_LEN, buf);
    return _convert_energy(buf);
}

/*  TODO: fix this method
 *  Sets the communication address of the device, needs to be of the form ###.###.###.### i.e. 192.168.1.1
 *
 */
void Serial::set_com_addr() {
    // package address info for device
    uint8_t set_com_addr_cmd[] = {0xB4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    for(int i = 1; i < 5; i++)
        set_com_addr_cmd[i] = com_addr[i-1];
    set_com_addr_cmd[6] = _calc_checksum(6, set_com_addr_cmd);
    // send packet to device
    send_packet(MSG_LEN, set_com_addr_cmd);
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof(buf));
    recieve_packet(MSG_LEN, buf);
    usleep(10000);
    recieve_packet(MSG_LEN, buf);
    return;
}

/*
 *  Waitline by sending blank packet and consuming packets from device
 */
void Serial::line_wait() {
    int num_times_wait = 4;
    uint8_t blank_packet[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t buf[MSG_LEN];
    for(int i = 0; i < num_times_wait; i++) {
        send_packet(MSG_LEN, blank_packet);
        usleep(SEND_RECEIVE_DELAY);
        recieve_packet(MSG_LEN, buf);
    }
    return;
}

/*
 *  Print methods
 */
/* Print packet on one line */
int Serial::print_packet(int size, uint8_t packet[]) {
    printf("packet: 0x");
    for(int i = 0; i < size; i++)
        printf("%.2x", packet[i]);
    printf("\n");
    return 0;
}

/* Prints the com address */
void Serial::print_com_addr() {
    for(int i = 0; i < 3; i++)
        cout << (int) com_addr[i] << ".";
    cout << (int) com_addr[3] << "\n";
    return;
}

/* Prints portname, baudrate, parity, file descriptor, address, and the packets */
void Serial::print_all() {
    cout << "port name: " << port << "\n";
    cout << "baudrate: " << baudrate << "\n";
    cout << "parity: " << parity << "\n";
    cout << "fd: " << fd << "\n";
    cout << "comm addr ";
    print_com_addr();
    cout << "read_voltage ";
    print_packet(MSG_LEN, read_voltage);
    cout << "read_current ";
    print_packet(MSG_LEN, read_current);
    cout << "read_wattage ";
    print_packet(MSG_LEN, read_wattage);
    cout << "read_energy ";
    print_packet(MSG_LEN, read_energy);
    cout << "set_addr ";
    print_packet(MSG_LEN, set_addr_packet);
    return;
}

/*
 *  Getters
 */
int Serial::get_fd() {
    return fd;
}

/* global methods */

/* 
 *  Sets usb information using the open file descriptor
 *  help with set_interface_attribs and set_blocking from
 *  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 */ 
static int set_interface_attribs (int fd, int speed, int parity) {
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
static int set_blocking (int fd, int should_block) {
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
static int send_packet(int fd, int size, uint8_t packet[]) { 
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
static int recieve_packet(int fd, int size, uint8_t *packet) {
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
static int callback(void *not_used, int argc, char **argv, char **az_col_name) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", az_col_name[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

int main() 
{
    /* get time */
    string the_time = current_time_and_date();
    cerr << "starting at current time: " << current_time_and_date() << "\n";

    // open usb device
    char portname[] = "/dev/ttyUSB0";
    char portname1[] = "/dev/ttyUSB1";

    int fd  = open (portname,  O_RDWR | O_NOCTTY | O_SYNC);
    int fd1 = open (portname1, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
            return -1;
    }

    // set speed to 9600 baud rate, 8n1 (no parity)
    set_interface_attribs (fd , B9600, 0);
    set_interface_attribs (fd1, B9600, 0);
    // set no blocking 
    set_blocking (fd , 0);
    set_blocking (fd1, 0);

    // set communication address as 192.168.1.2 (not for LAN, but for usb to device)
    uint8_t com_addr[] = {192, 168, 1, 1};

    // set serial class
    Serial s = Serial(portname , B9600, 0);
    Serial s1= Serial(portname1, B9600, 0);
    // calculate all packets 
    s.set_packets(4, com_addr);
    s1.set_packets(4, com_addr);
    // set the com address in the device
    set_com_addr(s.get_fd(), com_addr);
    set_com_addr(s1.get_fd(), com_addr);
    s.set_com_addr();
    s1.set_com_addr();
    // print all
    //s.print_all();
    //s1.print_all();

    /* sqlite3 database variables */
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc, rc2;

    /* Open database */
    rc = sqlite3_open("power_monitor.db", &db);
   
    if( rc ) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return(0);
    } else {
        fprintf(stderr, "Opened database successfully\n");
    }

    /* Get info */
    double voltage = 0.0,   v2=0;
    double  current =  0.0, c2=0;
    int power = 0,  p2=0;
    int energy = 0, e2=0;

    /* continuous loop */
    for (uint k = 0;; k++) {

        the_time = current_time_and_date();

        /* Query device */
        s.line_wait();
        voltage = s.receive_voltage();
        s.line_wait();
        current =  s.receive_current();
        s.line_wait();
        power = s.receive_power();
        s.line_wait();
        energy =  s.receive_energy();

        s1.line_wait();
        v2 = s1.receive_voltage();
        s1.line_wait();
        c2 =  s1.receive_current();
        s1.line_wait();
        p2 = s1.receive_power();
        s1.line_wait();
        e2 =  s1.receive_energy();

        /* Create SQL statement */
        string* sql = new string[180];
	return_formated_sql_insert_string(sql, the_time, 0, voltage, current, power, energy);
	string* sql2 = new string[180];
        return_formated_sql_insert_string(sql2, the_time, 1, v2, c2, p2, e2);

        /* Execute SQL statement */
        rc = sqlite3_exec(db, sql->c_str(), callback, 0, &zErrMsg);

        //Catch the error
        if( rc != SQLITE_OK ){
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            sql_cout_msg(the_time, 0, voltage, current, power, energy);
        }
       
        rc2 = sqlite3_exec(db, sql2->c_str(), callback, 0, &zErrMsg);
        // Catch the error
        if( rc2 != SQLITE_OK ){
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            sql_cout_msg(the_time, 1, v2, c2, p2, e2);
	}
       

    }

    sqlite3_close(db);

    printf("ending...\n");
    fflush(stdout);
 
    s.close_port();
 
    return 0;
}
