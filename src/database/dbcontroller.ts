require('./models/events');

var mongoose = require('mongoose');
var event = mongoose.model('Event');

export async function connectToDB(){
    mongoose.connect("mongodb+srv://rideDB:nl3fP1uYDI9DW8Cx@cluster0.xroyxn3.mongodb.net/?retryWrites=true&w=majority");
}

export async function closeConnection(){
    mongoose.connection.close();
}

export async function addNewEvent(desc: String, success: boolean){
    var newEvent = new event({
        event: desc,
        success
    });

    newEvent.save();
}

