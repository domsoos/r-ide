DROP DATABASE IF EXISTS `ridedb`;
CREATE DATABASE `ridedb`;
USE `ridedb`;

CREATE TABLE Wizards (
wizard_id int NOT NULL PRIMARY KEY,
template_id int NULL,
);

CREATE TABLE Template (
template_id int NOT NULL PRIMARY KEY,
event_id int NULL,
type varchar(50) NOT NULL,
wizard_id int NOT NULL,
);

CREATE TABLE Event (
shipment_id int NOT NULL PRIMARY KEY,
wizard_id int NULL,
template_id int NOT NULL,
type varchar(50) NOT NULL,
action varchar(50) NOT NULL,
action_date datetime NOT NULL,
);


