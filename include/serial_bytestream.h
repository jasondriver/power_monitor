#ifndef _SERIAL_BYTESTREAM_H
#define _SERIAL_BYTESTREAM_H

#include <vector>
#include <string>

using namespace std;

int set_interface_attribs (int fd, int speed, int parity);
int set_blocking (int fd, int should_block);
int send_packet(int fd, int size, uint8_t packet[]);
int recieve_packet(int fd, int size, uint8_t *packet);
int set_com_addr(int fd, uint8_t addr[]);
vector<int> current_time_hour_minutes();
string current_time_and_date();
void return_formated_sql_insert_string(string* out, string date_time, int device_id, double voltage, double current, int power, int energy);
void sql_cout_msg(string the_time, int id, double voltage, double current, int power, int energy);
int callback(void *not_used, int argc, char **argv, char **az_col_name);


#endif /* _SERIAL_BYTESTREAM_H */
