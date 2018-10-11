#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

//#include <sqlite3.h>
#include <stdint.h>

#define MSG_LEN 7

//#include <uapi/linux/usbdevice_fs.h>
// the usbdevice library is not on Mac so here is the workaround..
#define USBDEVFS_RESET _IO('U', 20)

/* 
 *  Sets usb information using the open file descriptor
 *  Thanks to wallyk for the set interface and blocking methods from this post on stack overflow
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
 *  Convert read voltage packet to voltage value
 *  voltage = packet[1] packet[2] . packet[3]
 *  TODO: add decimal place to return value
 */
static double convert_voltage(uint8_t packet[]) {
    //int voltage_int = (packet[1] << 8) + packet[2];
    int voltage_int = packet[1] * 100 + packet[2];
    double voltage_decimal = (double) packet[3] /100.0;
    double ret = (double) voltage_int + voltage_decimal;
    return ret;
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

int main() 
{
    uint8_t READ_VOLTAGE[] = {0xB4, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1E};
    //uint8_t READ_CURRENT[] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};

    printf("starting...");
    fflush(stdout);

    READ_VOLTAGE[6] = calc_checksum(6, READ_VOLTAGE);
    print_packet_oneline(MSG_LEN, READ_VOLTAGE);

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

    uint8_t buf[MSG_LEN];
    memset(&buf, 0, sizeof buf);
    
    for (int i = 0; i < 10; i++) {
        send_packet(fd, MSG_LEN, READ_VOLTAGE);
        usleep(10000);
        recieve_packet(fd, MSG_LEN, buf);
        printf("recieved packet\n");
        print_packet_oneline(MSG_LEN, buf);
    }

    printf("%f", convert_voltage(buf));

    printf("ending...");
    fflush(stdout);
  
    reset_device(fd);
    close(fd);
    return 0;
}
