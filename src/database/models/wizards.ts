
var mongoose = require('mongoose');


const wizardSchema = new mongoose.Schema({
    wizardName: {
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

mongoose.model('Wizard', wizardSchema);

