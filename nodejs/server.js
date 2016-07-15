// import HTTP module
var http = require('http');

// define the listening port
const PORT=8080; 

// define a function to handle requests and send a response
function handleRequest(request, response)
{
    response.end('Request arrived at path: ' + request.url);
    console.log('Response sent');
}

// create a server
var server = http.createServer(handleRequest);

// start the server
server.listen(PORT, function()
{
    // callback triggered when server is successfully listening
    console.log("Server listening on: http://localhost:%s", PORT);
}
);
