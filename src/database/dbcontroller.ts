require('./models/events');
require('./models/templates');
require('./models/wizards');


var mongoose = require('mongoose');
var event = mongoose.model('Event');
var template = mongoose.model('Template');
var wizard = mongoose.model('Wizard');

export async function connectToDB(){
    mongoose.connect("mongodb+srv://rideDB:nl3fP1uYDI9DW8Cx@cluster0.xroyxn3.mongodb.net/?retryWrites=true&w=majority");

    const db = mongoose.connection;
    db.on('error', console.error.bind(console, 'connection error: '));
    db.once('open', () => {
        
    });
}

export async function closeConnection(){
    mongoose.connection.close();
}

export async function addNewEvent(desc: String, success: boolean){
    var newEvent = new event({
        event: desc,
        success
    });

    try {
        await newEvent.save();
        
    } catch(error) {
        console.error(error);
    }
}

async function uploadTemplate(name:String, text:String, success: boolean): Promise<void> {
    const newTemplate = new template({
        templateName: name,
        templateText: text,
        success: success,
    });

    try {
        await newTemplate.save();
        
    } catch(error) {
        console.error(error);
    }
}


export async function getTemplateByName(name: string): Promise<string | null> {
    try {
        const templateText = await template.findOne({ name: name }).select('template').lean().exec();

        if(template) {
            return template.template;
        } else {
            return null;
        }
    } catch(error) {
        console.error(error);
        return null;
    }
}
