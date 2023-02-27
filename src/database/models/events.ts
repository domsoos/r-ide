var mongoose = require('mongoose');


const eventSchema = new mongoose.Schema({
    event: {
        type: String
        //required: true
    },
    success:{
        type: Boolean
        //required: true
    },
    createddate:{
        type: Date, 
        default: Date.now
    }
});

mongoose.model('Event', eventSchema);