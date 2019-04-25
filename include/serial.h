#ifndef POWER_MONITOR_SERIAL_H
#define POWER_MONITOR_SERIAL_H

// define max message length
#define MSG_LEN 7

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

#endif //POWER_MONITOR_SERIAL_H
