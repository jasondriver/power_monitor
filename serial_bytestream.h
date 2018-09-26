#ifndef _SERIAL_BYTESTREAM_H
#define _SERIAL_BYTESTREAM_H

#include <stdint.h>

static int set_interface_attribs (int fd, int speed, int parity);
static int set_blocking (int fd, int should_block);
static int print_packet(int size, uint8_t packet[]);
static int print_packet_oneline(int size, uint8_t packet[]);
static int send_packet(int fd, int size, uint8_t packet[]);
static int recieve_packet(int fd, int size, uint8_t *packet);
static int set_com_addr(int fd, uint8_t addr[]);
static double convert_voltage(uint8_t packet[]);
static int reset_device(int fd);

#endif /* _SERIAL_BYTESTREAM_H */
