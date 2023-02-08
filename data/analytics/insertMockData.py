import sys, json
import docker
import mysql.connector

import config

def getCursor():
    connection = mysql.connector.connect(
        host='localhost',
        user="root",
        password=config.password,
        #unix_socket = config.socket,
        database="ridedb"
    )
    if connection:
        print("Connection established")
    else:
        print("No connection..")
    cursor = connection.cursor()
    return cursor

def readFile(file_name):
    dataToRead = []
    with open(file_name, 'r') as in_file:
        data = json.load(in_file)
    if "events.json" in file_name:
        # we know the file to read is an event
        for line in data:
            query = "INSERT INTO Event (shipmentid, wizardid, templateid, type, action, actiondate) VALUES(%s, %s, %s, %s, %s, %s)"
            values = (int(line['shipment_id']), int(line['wizard_id']), line['template_id'], line[3], line[4], line[5]                                 )
            cursor = getCursor(query, values)
    elif "templates.json" in file_name:
        # it is a template file
        for line in data.values():
            templateid = line['template_id']
    elif "wizards.json" in file_name:
        # it is a wizard to read
        
        #print(data)
        for line in data.values():
            #print(line)
            wizardid = line['wizard_id']
            templateid = line['template_id']
            wtype = line['type']
            #print(f"wizardi: {wizardid} templateid: {templateid} type: {wtype}")
            if templateid and wtype:
                query = "INSERT INTO Wizards (wizardid, templateid) VALUES(%s, %s)"
                values = (wizardid, templateid)
                connection = mysql.connector.connect(
                    host='localhost',
                    user="root",
                    port = 3306,
                    password=config.password,
                    database="ridedb"
                )
                if connection:
                    print("Connection established")
                else:
                    print("No connection..")
                cursor = connection.cursor()
                #cursor.execute("SHOW TABLES")
                #cursor.commit()
                #results = cursor.fetchall()
                #for result in results:
                #    print(result[0])
                cursor.execute(query, values)
                cursor.commit()
                cursor.close()
                connection.close()

if __name__ == '__main__':
    if len(sys.argv) > 1:
        readFile(sys.argv(1))
    readFile("mock-wizards.json")
