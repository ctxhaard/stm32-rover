var connection = new WebSocket('ws://'+location.hostname+':81/', ['arduino']);
//var connection = new WebSocket('ws://192.168.4.1:81/', ['arduino']);

var last_dir = 'up'; // up, left, down, right
var last_vplain = 'right'; // left, right
var last_hplain = 'up'; // up, down
var last_force = 0.0;  

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

function sendUpdate() {
	var idir = 1;
	if (last_dir === 'up' ) {
		if (last_vplain === 'right') {
			idir = 1;
		} else {
			idir = 8;
		}
	} else if (last_dir === 'right') {
		if (last_hplain === 'up') {
			idir = 2;
		} else {
			idir = 3;
		}

	} else if (last_dir === 'down') {
		if (last_vplain === 'right') {
			idir = 4;
		} else {
			idir = 5;
		}		
	} else if (last_dir === 'left') {
		if (last_hplain === 'up') {
			idir = 7;
		} else {
			idir = 6;
		}
	}
	send( '\r\n' + idir + ':' + this.last_force + '\r\n' );
}
