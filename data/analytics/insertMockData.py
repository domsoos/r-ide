import sys, json
import mysql.connector

import config

def readFile(file_name):
    pass

def insertData():
    connection = mysql.connector.connect(
    host=config.host,
    port=config.port,
    user=config.user,
    password=config.password,
    database="ridedb"
    )

    cursor = connection.cursor()
    cursor.execute("INSERT * ")


if __name__ == '__main__':
    readFile(sys.argv[1])
    insertData()
