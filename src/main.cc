#include <cerrno>
#include <fcntl.h>
#include <cstring>
#include <termios.h>
#include <iostream>
#include <sqlite3.h>
#include <thread>
#include <deque>

#include "main.h"

using namespace std;

static volatile bool keep_running = true;
// all usb devices held in array
deque<Serial> dq;

void poll(string the_time, sqlite3 *db, char *zErrMsg, int rc) {
    // variables for
    double voltage = 0.0;
    double current = 0.0;
    int power = 0;
    int energy = 0;

    while(keep_running) {

        the_time = current_time_and_date();

        // Execute SQL transaction
        sqlite3_exec(db, "BEGIN TRANSACTION;", NULL, NULL, NULL);
        int j = 0;
        for(Serial s : dq) {
            // query device for info
            s.line_wait();
            voltage = s.receive_voltage();
            s.line_wait();
            current =  s.receive_current();
            s.line_wait();
            power = s.receive_power();
            s.line_wait();
            energy =  s.receive_energy();

            // Create SQL statement
            string* sql = new string[180];
            return_formated_sql_insert_string(sql, the_time, j, voltage, current, power, energy);
            // Insertion SQL statement
            rc = sqlite3_exec(db, sql->c_str(), callback, 0, &zErrMsg);

            // Catch the error
            if( rc != SQLITE_OK ){
                fprintf(stderr, "SQL error: %s\n", zErrMsg);
                sqlite3_free(zErrMsg);
            } else {
                sql_cout_msg(the_time, j, voltage, current, power, energy);
            }
            j++;
        }
        sqlite3_exec(db, "END TRANSACTION;", NULL, NULL, NULL);

    }
}

int main(int argc, char *argv[]) {

    char *path_to_db = argv[1];
    /* get time */
    string the_time = current_time_and_date();
    cerr << "starting at current time: " << current_time_and_date() << "\n";

    // open usb device
    for(int i = 2; i < argc; i++ ) {
        // get portname
        char *portname = argv[i];
        // set attributes (baudrate+parity)
        Serial s = Serial(portname , B9600, 0);
        dq.push_back(s);
    }

    // set communication address as 192.168.1.2 (not for LAN, but for usb to device)
    uint8_t com_addr[] = {192, 168, 1, 1};

    for(Serial s : dq) {
        // calculate all packets
        s.set_packets(4, com_addr);
        // set the com address
        set_com_addr(s.get_fd(), com_addr);
        s.set_com_addr();
    }

    /* sqlite3 database variables */
    sqlite3 *db;
    char *zErrMsg = 0;
    int rc;

    /* Open database */
    rc = sqlite3_open(path_to_db, &db);

    if( rc ) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        return(0);
    } else {
        fprintf(stderr, "Opened database successfully\n");
    }

    // start polling
    thread t(poll, the_time, db, zErrMsg, rc);

    // exit by pressing 'q'
    if(cin.get() == 'q') {
        keep_running = false;
        t.join();
    }

    sqlite3_close(db);

    printf("ending...\n");
    fflush(stdout);

    for(Serial s : dq) {
        s.close_port();
    }

    return 0;
}