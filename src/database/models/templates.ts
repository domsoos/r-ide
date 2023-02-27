var mongoose = require('mongoose');


const templateSchema = new mongoose.Schema({
    templateName: {
        type: String,
        required: true
    },
    templateText: {
        type: String,
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

mongoose.model('Template', templateSchema);