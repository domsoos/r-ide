CREATE DATABASE IF NOT EXISTS `ridedb`;
USE `ridedb`;

CREATE TABLE IF NOT EXISTS Wizards (
    wizardid int PRIMARY KEY,
    templateid int NULL,
    type varchar(50) NOT NULL
);
CREATE TABLE IF NOT EXISTS Template (
    templateid int PRIMARY KEY,
    type varchar(50) NOT NULL,
    wizardid int NOT NULL
);
CREATE TABLE IF NOT EXISTS Events (
    eventid int PRIMARY KEY,
    wizardid int NULL,
    templateid int NOT NULL,
    type varchar(50) NOT NULL,
    action varchar(50) NOT NULL,
    actiondate datetime NOT NULL
);
ALTER TABLE Events ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);
ALTER TABLE Events ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Wizards ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Template ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);

INSERT INTO Wizards(wizardid, templateid) VALUES(1000, 10000);
INSERT INTO Template(templateid, eventid, type, wizardid) VALUES(10000,100,"button click",1000);
INSERT INTO Events(eventid, wizardid, templateid, type, action, actiondate) VALUES(100, 1000,10000,"button click", "open wizard", NOW());



