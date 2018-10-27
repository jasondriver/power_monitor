var http = require('http');

// Options to be used by request 
var options = {
   host: 'localhost',
   port: '8081',
   path: 'templates/index.html'  
};

// Callback function is used to deal with response
var callback = function(response) {
   // Continuously update stream with data
   var body = '';
   response.on('data', function(data) {
      body += data;
   });
   
   response.on('end', function() {
      // Data received completely.
      console.log(body);
   });
}
// Make a request to the server
var req = http.request(options, callback);
req.end();


/*
// store a bunch of time values for the graph
times = []

function createGraph() {
  // this is how time would be stored on the server
  var now = Date.now()
  // add datum
  times.push({ 
    milliseconds: parseInt(now.toString().slice(-3)),
    time: now
  })

  console.log("times: " + times[0].milliseconds + " now: " + now)

  // remove old data
  if (times.length > 120)
    times.shift()
  // define plot boundaries
  var width = 300,
      height = 100
  var margin = {
    top: 0, 
    right: 10, 
    bottom: 5, 
    left: 50
  }
  var plot = {
    width: width - margin.right - margin.left,
    height: height - margin.top - margin.bottom
  }
  // x-axis is time
  var x = d3.time.scale()
    .range([0, plot.width])
  // y-axis is numerical
  var y = d3.scale.linear()
    .range([plot.height, 0])
  // set axis scales
  var xAxis = d3.svg.axis()
    .scale(x)
    .orient('bottom')
    .tickFormat('')
    .tickSize(0, 0)

  var yAxis = d3.svg.axis()
    .scale(y)
    .orient('left')
    .tickSize(0, 0).ticks(3)
  // set time span to show
  var timeCap = width * 40 // 12s
  var latest = times.length
    ? times[times.length - 1].time
    : 0
  var data = times.filter(function(d) {
    return d.time >= latest - timeCap
  })
  
  x.domain([latest - timeCap, latest])
  y.domain([0, 1000])

  var line = d3.svg.line()
    .x(function(d) { return x(parseInt(d.time)) })
    .y(function(d) { return y(d.milliseconds) })  
  // make the graph
  var svg = d3.select('#dashboard_graph')
  var graph = undefined

  if (d3.select('.graph-g').empty()) {
    graph = svg.append('g')
        .attr('class', 'graph-g')
        .attr('width', plot.width)
        .attr('height', plot.height)
        .attr('transform', 'translate(' + margin.left + ',' + margin.top + ')')
    //add axes
    graph.append('g')
        .attr('class', 'x axis')
        .attr('transform', 'translate(0,' + plot.height + ')')
        .call(xAxis)
      .append('text')
        .attr('dx', (plot.width / 2))
        .attr('dy', '1em')
        .style('text-anchor', 'middle')
        .text('Time')

    graph.append('g')
        .attr('class', 'y axis')
        .call(yAxis)
      .append('text')
        .attr('transform', 'rotate(-90)')
        .attr('dx', (0 - plot.height / 2))
        .attr('dy', '-2.8em')
        .style('text-anchor', 'middle')
        .text('ms');
  } else {
    graph = d3.select('.graph-g')
  }

  // remove old line
  graph.select('.line').remove()
  //add data line
  graph.append('path')
    .datum(data)
    .attr('class', 'line')
    .attr('d', line)
}

test_values = [5.5, 8.9, 2.0, 4.1, 3.3]

function draw_dashboard_graph() {
  var width = 300
  var height = 100
  var svg = d3.select('#dashboard')
  test_values.push(random())
  path
     .attr("d", line)
     .attr("transform", null)
   .transition()
     .attr("transform", "translate(" + x(-1) + ")");

  // remove old data
  if (test_values.length > 100)
    test_values.shift()
}

// draw graph every 1 second
//var startGraph = window.setInterval(createGraph, 1000)
var startGraph = window.setInterval(draw_dashboard_graph, 1000)
*/
