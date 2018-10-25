var http = require('http');
var fs = require('fs');
const sqlite3 = require('sqlite3').verbose();
 
// open the database
var dbpath = './power_monitor.db'

let db = new sqlite3.Database(dbpath, sqlite3.OPEN_READWRITE, (err) => {
    if (err) {
        console.error(err.message);
    }
    console.log('Connected to the power monitoring database.');
});

// query statement
var sql = `SELECT date as d, device_id as id, power as p FROM (SELECT * FROM DAILY_POWER ORDER BY date DESC) LIMIT 3`
var arr = [];

// query database
db.serialize(function() {
    db.all(sql, [], (err, rows) => {
        if (err) {
            throw err;
        }
        rows.forEach((row) => {
            var item = {d: row.d, id: row.id, p: row.p};
            arr.push(item);
            console.log(row.d + "\t" + row.id + "\t" + row.p);
        });
    });
});

//console.log(arr.toString());

// close database
db.close((err) => {
    if (err) {
        console.error(err.message);
    }
    console.log('Close the database connection.');
});

console.log(arr.toString());

// serve webpages
fs.readFile('templates/index.html', function (err, html) {
    if (err) {
        throw err; 
    }
    http.createServer(function(request, response) { 
        response.writeHeader(200, {"Content-Type": "text/html"});
        response.write(html); 
        response.end();   
    }).listen(8080);
});
