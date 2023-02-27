-- MySQL dump 10.13  Distrib 8.0.32, for Linux (x86_64)
--
-- Host: localhost    Database: ridedb
-- ------------------------------------------------------
-- Server version	8.0.32

/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!50503 SET NAMES utf8mb4 */;
/*!40103 SET @OLD_TIME_ZONE=@@TIME_ZONE */;
/*!40103 SET TIME_ZONE='+00:00' */;
/*!40014 SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0 */;
/*!40014 SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0 */;
/*!40101 SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='NO_AUTO_VALUE_ON_ZERO' */;
/*!40111 SET @OLD_SQL_NOTES=@@SQL_NOTES, SQL_NOTES=0 */;

--
-- Table structure for table `Events`
--

DROP TABLE IF EXISTS `Events`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Events` (
  `eventid` int NOT NULL AUTO_INCREMENT,
  `wizardid` int DEFAULT NULL,
  `templateid` int DEFAULT NULL,
  `type` varchar(100) NOT NULL DEFAULT 'button click',
  `action` varchar(100) NOT NULL DEFAULT 'start ROS bag',
  `actiondate` datetime NOT NULL,
  PRIMARY KEY (`eventid`),
  KEY `wizardid` (`wizardid`),
  KEY `templateid` (`templateid`),
  CONSTRAINT `Events_ibfk_1` FOREIGN KEY (`wizardid`) REFERENCES `Wizards` (`wizardid`),
  CONSTRAINT `Events_ibfk_2` FOREIGN KEY (`templateid`) REFERENCES `Template` (`templateid`)
) ENGINE=InnoDB AUTO_INCREMENT=2 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Events`
--

LOCK TABLES `Events` WRITE;
/*!40000 ALTER TABLE `Events` DISABLE KEYS */;
INSERT INTO `Events` VALUES (1,1,2,'button click','open wizard','2023-02-14 01:24:17');
/*!40000 ALTER TABLE `Events` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Template`
--

DROP TABLE IF EXISTS `Template`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Template` (
  `templateid` int NOT NULL AUTO_INCREMENT,
  `type` varchar(50) NOT NULL DEFAULT 'message',
  PRIMARY KEY (`templateid`)
) ENGINE=InnoDB AUTO_INCREMENT=5 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Template`
--

LOCK TABLES `Template` WRITE;
/*!40000 ALTER TABLE `Template` DISABLE KEYS */;
INSERT INTO `Template` VALUES (1,'message'),(2,'service'),(3,'ROS Topic'),(4,'ROS Subscriber');
/*!40000 ALTER TABLE `Template` ENABLE KEYS */;
UNLOCK TABLES;

--
-- Table structure for table `Wizards`
--

DROP TABLE IF EXISTS `Wizards`;
/*!40101 SET @saved_cs_client     = @@character_set_client */;
/*!50503 SET character_set_client = utf8mb4 */;
CREATE TABLE `Wizards` (
  `wizardid` int NOT NULL AUTO_INCREMENT,
  `templateid` int NOT NULL,
  `type` varchar(50) NOT NULL DEFAULT 'Node',
  PRIMARY KEY (`wizardid`),
  KEY `templateid` (`templateid`),
  CONSTRAINT `Wizards_ibfk_1` FOREIGN KEY (`templateid`) REFERENCES `Template` (`templateid`)
) ENGINE=InnoDB AUTO_INCREMENT=6 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;
/*!40101 SET character_set_client = @saved_cs_client */;

--
-- Dumping data for table `Wizards`
--

LOCK TABLES `Wizards` WRITE;
/*!40000 ALTER TABLE `Wizards` DISABLE KEYS */;
INSERT INTO `Wizards` VALUES (1,1,'Node'),(2,2,'Node'),(3,3,'Node'),(4,4,'Node'),(5,1,'Node');
/*!40000 ALTER TABLE `Wizards` ENABLE KEYS */;
UNLOCK TABLES;
/*!40103 SET TIME_ZONE=@OLD_TIME_ZONE */;

/*!40101 SET SQL_MODE=@OLD_SQL_MODE */;
/*!40014 SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS */;
/*!40014 SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS */;
/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
/*!40111 SET SQL_NOTES=@OLD_SQL_NOTES */;

-- Dump completed on 2023-02-14  1:37:44
