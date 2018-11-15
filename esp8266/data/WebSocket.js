var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);

connection.onopen = function () {
    console.log('Connect ' + new Date());
};
connection.onerror = function (error) {
    console.log('WebSocket Error ', error);
};
connection.onmessage = function (e) {  
    console.log('Server: ', e.data);
};
connection.onclose = function(){
    console.log('WebSocket connection closed');
};

function send(cmd) {
    console.log('>>> ' + cmd); 
    connection.send(cmd);
}
