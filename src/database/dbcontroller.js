"use strict";
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.getTemplateByName = exports.addNewEvent = exports.closeConnection = exports.connectToDB = void 0;
require('./models/events');
require('./models/templates');
require('./models/wizards');
var mongoose = require('mongoose');
var event = mongoose.model('Event');
var template = mongoose.model('Template');
var wizard = mongoose.model('Wizard');
function connectToDB() {
    return __awaiter(this, void 0, void 0, function* () {
        mongoose.connect("mongodb+srv://rideDB:nl3fP1uYDI9DW8Cx@cluster0.xroyxn3.mongodb.net/?retryWrites=true&w=majority");
        const db = mongoose.connection();
        db.on('error', console.error.bind(console, 'connection error: '));
        db.once('open', () => {
            console.log('connected to mongodb');
        });
    });
}
exports.connectToDB = connectToDB;
function closeConnection() {
    return __awaiter(this, void 0, void 0, function* () {
        mongoose.connection.close();
    });
}
exports.closeConnection = closeConnection;
function addNewEvent(desc, success) {
    return __awaiter(this, void 0, void 0, function* () {
        var newEvent = new event({
            event: desc,
            success
        });
        try {
            yield newEvent.save();
            console.log(`Created new event: ${desc}`);
        }
        catch (error) {
            console.error(error);
        }
    });
}
exports.addNewEvent = addNewEvent;
function uploadTemplate(name, text, success) {
    return __awaiter(this, void 0, void 0, function* () {
        const newTemplate = new template({
            templateName: name,
            templateText: text,
            success: success,
        });
        try {
            yield newTemplate.save();
            console.log(`Created new template: ${name}`);
        }
        catch (error) {
            console.error(error);
        }
    });
}
function getTemplateByName(name) {
    return __awaiter(this, void 0, void 0, function* () {
        try {
            const templateText = yield template.findOne({ name: name }).select('template').lean().exec();
            if (template) {
                return template.template;
            }
            else {
                return null;
            }
        }
        catch (error) {
            console.error(error);
            return null;
        }
    });
}
exports.getTemplateByName = getTemplateByName;
//# sourceMappingURL=dbcontroller.js.map