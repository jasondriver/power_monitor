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
 *  Sets usb information using the open file descriptor
 *  Thanks to wallyk for the set_interface_attribs and set_blocking methods from this post on stack overflow
 *  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
 *  added some slight modifications to them
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
 *  Prints array elements
 *  Caller must make sure that the length of the packet is correct
 */
static int print_packet(int size, uint8_t packet[]) {
    for(int i = 0; i < size; i++) {
        printf("%d) 0x%.2x\n", i, packet[i]);
    }
    return 0;
}

/*
 *  Print packet on one line
 */
static int print_packet_oneline(int size, uint8_t packet[]) {
    printf("packet: 0x");
    for(int i = 0; i < size; i++)
        printf("%.2x", packet[i]);
    printf("\n");
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
    printf("Sent packet:\n");
    print_packet_oneline(MSG_LEN, set_com_addr_cmd);
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof(buf));
    recieve_packet(fd, MSG_LEN, buf);
    usleep(10000);
    recieve_packet(fd, MSG_LEN, buf);
    printf("Recieved packet:\n");
    print_packet_oneline(MSG_LEN, buf);
    return 0;
}

/*
 *  Convert voltage packet to voltage value
 *  voltage = packet[1] packet[2] . packet[3]
 */
static double convert_voltage(uint8_t packet[]) {
    int voltage_int = packet[1] * 100 + packet[2];
    double voltage_decimal = (double) packet[3] /100.0;
    double ret = (double) voltage_int + voltage_decimal;
    return ret;
}

/*
 *  Convert current packet to value
 */
static double convert_current(uint8_t packet[]) {
    return (double) packet[2] + ((double) packet[3] / 100.0);
}

/*
 *  Helper function for convert_power and convert_energy
 *  size = how many elements to use, offset = where in the array to start, packet = array of #s
 */
static int hex_to_int(int size, int offset, uint8_t packet[]) {
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

/*
 *  Convert average power packet to power value
 *  power = (packet[1] packet[2])base16
 */
static int convert_power(uint8_t packet[]) {
    return hex_to_int(2, 1, packet);
}

/*
 *  Convert energy packet to value
 */
static int convert_energy(uint8_t packet[]) {
    return hex_to_int(3, 1, packet);
}

/*
 *  
 *  Main methods
 *
 */

static int receive_power(int fd) {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(fd, MSG_LEN, READ_WATTAGE);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(fd, MSG_LEN, buf);
    return convert_power(buf);
}

static double receive_voltage(int fd) {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(fd, MSG_LEN, READ_VOLTAGE);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(fd, MSG_LEN, buf);
    return convert_voltage(buf);
}

static double receive_current(int fd) {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(fd, MSG_LEN, READ_CURRENT);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(fd, MSG_LEN, buf);
    return convert_current(buf);
}

static int receive_energy(int fd) {
    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    send_packet(fd, MSG_LEN, READ_ENERGY);
    usleep(SEND_RECEIVE_DELAY);
    recieve_packet(fd, MSG_LEN, buf);
    return convert_energy(buf);
}

static void flush_line(int fd) {
    int num_times_flush = 10;
    uint8_t flush_packet[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t buf[MSG_LEN];
    for(int i = 0; i < num_times_flush; i++) {
        send_packet(fd, MSG_LEN, flush_packet);
        usleep(SEND_RECEIVE_DELAY);
        recieve_packet(fd, MSG_LEN, buf);
    }
    return;
}

/* 
 *  TODO: resets usb device
 */
static int reset_device(int fd) {
    // doesn't work
    int rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return -1;
    }
    return 0;
}

string current_time_and_date() {
    time_t raw_time = time(nullptr);
    stringstream ss;
    ss << put_time(localtime(&raw_time), "%Y-%m-%d %X");
    return ss.str();
}

string return_formated_sql_insert_string(int device_id, double voltage, double current, int power, int energy) {
    //string sql = "INSERT INTO DAILY_POWER (DATE,DEVICE_ID,VOLTAGE,CURRENT,POWER,ENERGY) "  \
                 //"VALUES ('"+current_time_and_date()+"', "+to_string(device_id)+", "+to_string(voltage)+", "+to_string(current)+", "+to_string(power)+", "+to_string(energy)+" );\0";
                       //"VALUES ('today', 1, 120.0, 0.1, 10, 10 );";
    stringstream ss;
    ss << "INSERT INTO DAILY_POWER (DATE,DEVICE_ID,VOLTAGE,CURRENT,POWER,ENERGY) ";
    ss << "VALUES ('" << current_time_and_date() << "', " << device_id << ", " << setw(5) << voltage;
    ss << ", " << setw(4) << current << ", " << power << ", " << energy << " );";
    return ss.str();
}

static int callback(void *NotUsed, int argc, char **argv, char **azColName) {
   int i;
   for(i = 0; i<argc; i++) {
      printf("%s = %s\n", azColName[i], argv[i] ? argv[i] : "NULL");
   }
   printf("\n");
   return 0;
}

int main() 
{
    printf("starting...\n");
    fflush(stdout);
   
    /*
    * get time
    */
    string the_time = current_time_and_date();
    cout << current_time_and_date() << "\n";

    // calc checksum for READ_VOLTAGE
    READ_VOLTAGE[6] = calc_checksum(6, READ_VOLTAGE);
    print_packet_oneline(MSG_LEN, READ_VOLTAGE);

    // open usb device
    char portname[] = "/dev/ttyUSB0";
    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);

    if (fd < 0)
    {
            printf("error %d opening %s: %s", errno, portname, strerror (errno));
            return -1;
    }

    // set speed to 9600 baud rate, 8n1 (no parity)
    set_interface_attribs (fd, B9600, 0);
    // set no blocking 
    set_blocking (fd, 0);

    // set communication address as 192.168.1.1 (not for LAN, but for usb to device)
    uint8_t com_addr[] = {192, 168, 1, 1};
    set_com_addr(fd, com_addr);

    send_packet(fd, MSG_LEN, READ_VOLTAGE);
    flush_line(fd);

    for (int i = 0; i < 2; i++) {
        flush_line(fd);
        printf("Wattage is: %d\n", receive_power(fd));
    }

    for (int i = 0; i < 2; i++) {
        flush_line(fd);
        printf("Voltage is: %f\n", receive_voltage(fd));
    }

    for (int i = 0; i < 2; i++) {
        flush_line(fd);
        printf("Current is: %f\n", receive_current(fd));
    }

    for (int i = 0; i < 2; i++) {
        flush_line(fd);
        printf("Energy is: %d\n", receive_energy(fd));
    }

    /*
     * open and add to sqlite3 database
     */
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    /* Open database */
    rc = sqlite3_open("power_monitor.db", &db);
   
    if( rc ) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return(0);
    } else {
        fprintf(stderr, "Opened database successfully\n");
    }

    /* Create SQL statement */
    //sql = "INSERT INTO DAILY_POWER (DATE,DEVICE_ID,VOLTAGE,CURRENT,POWER,ENERGY) "  \
    //      "VALUES ('today', 1, 120.0, 0.1, 10, 10 );";

    int device_id = 1;
    double voltage = 0.0;
    double  current =  0.0;
    voltage = receive_voltage(fd);
    current =  receive_current(fd);
    int power = 0;
    int energy = 0;
    power = receive_power(fd);
    energy =  receive_energy(fd);

    char const* sql = return_formated_sql_insert_string(device_id, voltage, current, power, energy).c_str();
    cout << sql << "\n";
    printf("%f %f %d %d", voltage, current, power, energy);

    /* Execute SQL statement */
    rc = sqlite3_exec(db, sql, callback, 0, &zErrMsg);
   
    if( rc != SQLITE_OK ){
        fprintf(stderr, "SQL error: %s\n", zErrMsg);
        sqlite3_free(zErrMsg);
    } else {
        fprintf(stdout, "Records created successfully\n");
    }
    sqlite3_close(db);



    printf("ending...\n");
    fflush(stdout);
  
    //reset_device(fd);
    close(fd);
    return 0;
}
