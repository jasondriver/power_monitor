#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>

#include <stdint.h>

#define MSG_LEN 7

//#include <uapi/linux/usbdevice_fs.h>
// the usbdevice library is not on Mac so here is the workaround..
#define USBDEVFS_RESET _IO('U', 20)

/*
/  Thanks to wallyk for the set interface and blocking methods from this post on stack overflow
/  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
*/ 
int set_interface_attribs (int fd, int speed, int parity)
{
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

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
                printf("error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
                printf("error %d setting term attributes", errno);
}

// caller must make sure that the length of the packet is correct
int print_packet(int size, uint8_t packet[]) {
    for(int i = 0; i < size; i++) {
        printf("%d) 0x%x\n", i, packet[i]);
    }
    return 0;
}

int main() 
{
    uint8_t READ_VOLTAGE[] = {0xB4, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1E};
    //uint8_t READ_CURRENT[] = {0xB1, 0xC0, 0xA8, 0x01, 0x01, 0x00, 0x1B};

    printf("starting...");
    fflush(stdout);
    char *portname = "/dev/tty.usbserial-A601SWKZ";
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

    // commands for use with the device
    uint8_t read_voltage[MSG_LEN];
    read_voltage[0] = 0xB4;
    read_voltage[1] = 0xC0;
    read_voltage[2] = 0xA8;
    read_voltage[3] = 0x01;
    read_voltage[4] = 0x01;
    read_voltage[5] = 0x00;
    read_voltage[6] = 0x1E;

    uint8_t read_current[MSG_LEN];
    read_current[0] = 0xB1;
    read_current[1] = 0xC0;
    read_current[2] = 0xA8;
    read_current[3] = 0x01;
    read_current[4] = 0x01;
    read_current[5] = 0x00;
    read_current[6] = 0x1B;

    print_packet(MSG_LEN, READ_VOLTAGE);

    for(int i = 0; i < MSG_LEN; i++) {
        printf("%d) 0x%d\n", i, read_voltage[i]); 
    }

    for(int i = 0; i < MSG_LEN; i++) {
        write (fd, &read_voltage[i], 1);
        // sleep enough to transmit the 1 byte (#bytes+25)*100
        // receive 25:  approx 100 uS per char transmit
        usleep ((1 + 25) * 100);
    }

    uint8_t buf [MSG_LEN];
    memset(&buf, -1, sizeof buf);
    for(int i = 0; i < MSG_LEN; i++) {
        int n = read (fd, &buf[i], 1); 
        if (n == 1) 
            continue;
        else
            break;
        usleep (1000);
    }

    for(int i = 0; i < MSG_LEN; i++) {                                             
        printf("%d) 0x%d\n", i, buf[i]);                             
    } 

    printf("ending...");
    fflush(stdout);
    
    // doesn't work
    int rc = ioctl(fd, USBDEVFS_RESET, 0);
    if (rc < 0) {
        perror("Error in ioctl");
        return 1;
    }

    close(fd);
    return 0;
}
