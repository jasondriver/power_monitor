#include <cerrno>
#include <fcntl.h>
#include <cstring>
#include <termios.h>
#include <iostream>
#include <sqlite3.h>
#include <vector>

#include "main.h"

using namespace std;

int main(int argc, char *argv[]) {
    /* get time */
    string the_time = current_time_and_date();
    cerr << "starting at current time: " << current_time_and_date() << "\n";

    // open usb device
    char portname[] = "/dev/ttyUSB0";
    char portname1[] = "/dev/ttyUSB1";

    // set serial class
    Serial s = Serial(portname , B9600, 0);
    Serial s1= Serial(portname1, B9600, 0);

    // set speed to 9600 baud rate, 8n1 (no parity)
    set_interface_attribs (s.get_fd() , B9600, 0);
    set_interface_attribs (s1.get_fd(), B9600, 0);
    // set no blocking
    set_blocking (s.get_fd() , 0);
    set_blocking (s1.get_fd(), 0);

    // set communication address as 192.168.1.2 (not for LAN, but for usb to device)
    uint8_t com_addr[] = {192, 168, 1, 1};

    // calculate all packets
    s.set_packets(4, com_addr);
    s1.set_packets(4, com_addr);
    // set the com address in the device
    set_com_addr(s.get_fd(), com_addr);
    set_com_addr(s1.get_fd(), com_addr);
    s.set_com_addr();
    s1.set_com_addr();
//    print all
//    s.print_all();
//    s1.print_all();

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

//    // flag for triggering io pins
//    bool pin_triggered = false;
//    // setup gpio pins for output
//    wiringPiSetup();
//    pinMode(0, OUTPUT);
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


//	// trigger io pin
//        vector<int> time = current_time_hour_minutes();
//        if(!pin_triggered && time[0] == threshhold_hour && (threshhold_minute >= time[1])) {
//	    pin_triggered = true;
//	    digitalWrite(0, HIGH);
//	}
//	// reset trigger at midnight
//	if(time[0] == 0) {
//	    pin_triggered = false;
//	    digitalWrite(0, LOW);
//	}

        /* Create SQL statement */
        string* sql = new string[180];
        return_formated_sql_insert_string(sql, the_time, 0, voltage, current, power, energy);
        string* sql2 = new string[180];
        return_formated_sql_insert_string(sql2, the_time, 1, v2, c2, p2, e2);

        /* Execute SQL transaction */
        sqlite3_exec(db, "BEGIN TRANSACTION;", NULL, NULL, NULL);

        // insertion
        rc = sqlite3_exec(db, sql->c_str(), callback, 0, &zErrMsg);

        //Catch the error
        if( rc != SQLITE_OK ){
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            sql_cout_msg(the_time, 0, voltage, current, power, energy);
        }

        // insertion
        rc2 = sqlite3_exec(db, sql2->c_str(), callback, 0, &zErrMsg);

        // Catch the error
        if( rc2 != SQLITE_OK ){
            fprintf(stderr, "SQL error: %s\n", zErrMsg);
            sqlite3_free(zErrMsg);
        } else {
            sql_cout_msg(the_time, 1, v2, c2, p2, e2);
	    }
        sqlite3_exec(db, "END TRANSACTION;", NULL, NULL, NULL);

    }

    sqlite3_close(db);

    printf("ending...\n");
    fflush(stdout);

    s.close_port();

    return 0;
}