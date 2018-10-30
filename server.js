var http = require('http');
var fs = require('fs');
var url = require('url');

// Create a server
http.createServer( function (request, response) {  
   // Parse the request containing file name
   var pathname = url.parse(request.url).pathname;
   
   // Print the name of the file for which request is made.
   console.log("Request for " + pathname + " received.");
   
   // Read the requested file content from file system
   fs.readFile(pathname.substr(1), function (err, data) {
      if (err) {
         console.log(err);
         
         // HTTP Status: 404 : NOT FOUND
         // Content Type: text/plain
         response.writeHead(404, {'Content-Type': 'text/html'});
      } else {	
         //Page found	  
         // HTTP Status: 200 : OK
         // Content Type: text/plain
         response.writeHead(200, {'Content-Type': 'text/html'});	
         
         // Write the content of the file to response body
         response.write(data.toString());		
      }
      
      // Send the response body 
      response.end();
   });   
}).listen(8080);

// Console will print the message
console.log('Server running at http://127.0.0.1:8080/');

/*
var express = require('express');
var router = express.Router();
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
router.get('templates/index.html', function(req, res) {
    var yourCities =  ["stockholm", "moscow", "barcelona", "bordeaux", "helsinki", "london"];
    var cityList = yourCities.reduce(function(cityList, city) {
      cityList.push(require('../public/cityData/' + city))
      return cityList;
    }, [])

    //TODO::need to update this to send an array
    res.render('map', {
        cityList : JSON.stringify(cityList),
    });
});

module.exports = router;

// serve webpages
fs.readFile('templates/index.html', function (err, html) {
    if (err) {
        throw err; 
    }
    http.createServer(function(request, response) { 
        res.render('map', {
            powerdata : JSON.stringify(arr),
        });
        response.writeHeader(200, {"Content-Type": "text/html"});
        response.write(html); 
        response.end();   
    }).listen(8080);
});
*/
