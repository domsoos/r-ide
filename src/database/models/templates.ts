var mongoose = require('mongoose');


const templateSchema = new mongoose.Schema({
    template: {
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

mongoose.model('Template', templateSchema);