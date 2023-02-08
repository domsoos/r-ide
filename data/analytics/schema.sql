CREATE DATABASE IF NOT EXISTS `ridedb`;
USE `ridedb`;

CREATE TABLE Wizards (
    wizardid int PRIMARY KEY,
    templateid int NULL,
    type varchar(50) NOT NULL
);

CREATE TABLE Template (
    templateid int PRIMARY KEY,
    eventid int NULL,
    type varchar(50) NOT NULL,
    wizardid int NOT NULL,
);

CREATE TABLE Event (
    shipmentid int PRIMARY KEY,
    wizardid int NULL,
    templateid int NOT NULL,
    type varchar(50) NOT NULL,
    action varchar(50) NOT NULL,
    actiondate datetime NOT NULL,
);


ALTER TABLE Event ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);
ALTER TABLE Event ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Wizards ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Template ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);
ALTER TABLE Template ADD FOREIGN KEY (eventid) REFERENCES Event(shipmentid);


INSERT INTO Event(shipmentid, wizardid, templateid, type, action, actiondate) VALUES(1, 1,1,"button click", "open wizard", NOW());
INSERT INTO Wizards(wizardid, templateid) VALUES(1, 1);
INSERT INTO Template(templateid, eventid, type, wizardid) VALUES(1,1,"button click",1);

