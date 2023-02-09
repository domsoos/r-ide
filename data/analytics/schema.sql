CREATE DATABASE IF NOT EXISTS `ridedb`;
USE `ridedb`;

CREATE TABLE IF NOT EXISTS Wizards (
    wizardid INT PRIMARY KEY AUTO_INCREMENT,
    templateid INT DEFAULT 0 UNIQUE NULL,
    type VARCHAR(50) DEFAULT 'Node' NOT NULL
);
CREATE TABLE IF NOT EXISTS Template (
    templateid INT PRIMARY KEY AUTO_INCREMENT,
    wizardid INT DEFAULT 0 UNIQUE NOT NULL,
    type VARCHAR(50) NOT NULL
);
CREATE TABLE IF NOT EXISTS Events (
    eventid INT PRIMARY KEY,
    wizardid int DEFAULT 0 UNIQUE NOT NULL,
    templateid INT DEFAULT 0 UNIQUE NOT NULL,
    type VARCHAR(50) DEFAULT 'button click' NOT NULL,
    action VARCHAR(50) DEFAULT 'start ROS bag' NOT NULL,
    actiondate datetime NOT NULL
);
ALTER TABLE Events ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);
ALTER TABLE Events ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Wizards ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Template ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);

INSERT INTO Wizards(wizardid, templateid) VALUES(1000, 10000);
INSERT INTO Template(templateid, eventid, type, wizardid) VALUES(10000,100,"button click",1000);
INSERT INTO Events(eventid, wizardid, templateid, type, action, actiondate) VALUES(100, 1000,10000,"button click", "open wizard", NOW());



