#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>

#include "serial.h"

// define delay between sent packet and receive packet
#define SEND_RECEIVE_DELAY 10000

// global predefined packets to be sent for device by manufacturer
uint8_t READ_VOLTAGE[] = {0xB0, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1A};
uint8_t READ_CURRENT[] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};
uint8_t READ_WATTAGE[] = {0XB2, 0XC0, 0XA8, 0X01, 0X01, 0X00, 0X1C};
uint8_t READ_ENERGY[] = {0xB3, 0XC0, 0XA8, 0X01, 0X01, 0X00, 0X1D};

using namespace std;

/*
 *  Constructor
 */
Serial::Serial(char* p, int b, int pa) : port(p), baudrate(b), parity(pa) {

    // open device at port
    int f = open (port, O_RDWR | O_NOCTTY | O_SYNC);

    if (f < 0) {
            printf("error %d opening %s: %s", errno, port, strerror (errno));
            fd = -1;
    }
    else {
        fd = f;
        // set speed to 9600 baud rate, 8n1 (no parity)
        _set_interface_attribs (b, pa);
        // set no blocking
        _set_blocking (0);
    }
}

/*
 *  Deconstructor
 */
Serial::~Serial() {
    return;
}

/* opens port */
void Serial::open_port(char* p) {
    port = p;
    int f = open (p, O_RDWR | O_NOCTTY | O_SYNC);

    if (f < 0) {
            printf("error %d opening %s: %s", errno, port, strerror (errno));
            fd = -1;
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