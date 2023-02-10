CREATE DATABASE IF NOT EXISTS `ridedb`;
USE `ridedb`;

CREATE TABLE IF NOT EXISTS Wizards (
    wizardid INT PRIMARY KEY AUTO_INCREMENT,
    templateid INT NOT NULL,
    type VARCHAR(50) DEFAULT 'Node' NOT NULL
);
CREATE TABLE IF NOT EXISTS Template (
    templateid INT PRIMARY KEY AUTO_INCREMENT,
    type VARCHAR(50) DEFAULT 'message' NOT NULL
);
CREATE TABLE IF NOT EXISTS Events (
    eventid INT PRIMARY KEY AUTO_INCREMENT,
    wizardid INT NULL,
    templateid INT NULL,
    type VARCHAR(100) DEFAULT 'button click' NOT NULL,
    action VARCHAR(100) DEFAULT 'start ROS bag' NOT NULL,
    actiondate datetime NOT NULL
);
ALTER TABLE Events ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);
ALTER TABLE Events ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
ALTER TABLE Wizards ADD FOREIGN KEY (templateid) REFERENCES Template(templateid);
-- ALTER TABLE Template ADD FOREIGN KEY (wizardid) REFERENCES Wizards(wizardid);

INSERT INTO Template(templateid,type) VALUES(1, "message");
INSERT INTO Template(templateid,type) VALUES(2, "service");
INSERT INTO Template(templateid,type) VALUES(3, "ROS Topic");
INSERT INTO Template(templateid,type) VALUES(4, "ROS Subscriber");

INSERT INTO Wizards(wizardid, templateid, type) VALUES(1, 1, "Node");
INSERT INTO Wizards(wizardid, templateid, type) VALUES(2, 2, "Node");
INSERT INTO Wizards(wizardid, templateid, type) VALUES(3, 3, "Node");
INSERT INTO Wizards(wizardid, templateid, type) VALUES(4, 4, "Node");
INSERT INTO Wizards(wizardid, templateid, type) VALUES(5, 1, "Node");

INSERT INTO Events(wizardid, templateid, type, action, actiondate) VALUES(1,2,"button click", "open wizard", NOW());
