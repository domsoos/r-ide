import sys, json
import docker
import mysql.connector

import config

def getConnection():
    connection = mysql.connector.connect(
        host='localhost',
        user="root",
        port = 3306,
        password=config.password,
        database="ridedb"
    )
    if connection:
        pass
    else:
        print("No connection..")
    return connection

def execute(connection, query, values):
    cursor = connection.cursor()
    cursor.execute(query, values)
    connection.commit()
    print("Insert successful")
    cursor.close()
    connection.close()

def readFile(file_name):
    dataToRead = []
    with open(file_name, 'r') as in_file:
        data = json.load(in_file)
    if "events.json" in file_name:
        # we know the file to read is an event
        for line in data.values():
            wizardid = int(line['wizard_id'])
            templateid = int(line['template_id'])
            isWizard=False
            if wizardid:
                connection = getConnection()
                cursor = connection.cursor()
                cursor.execute("SELECT * FROM Template WHERE templateid=%s", (templateid, ))
                result = cursor.fetchone()
                if result is not None:
                    print("Template entry already exist")
                else:
                    query = "INSERT INTO Template (templateid, type) VALUES(%s, %s)"
                    values = (templateid, "ROS monitor")
                    execute(connection, query, values)

                connection = getConnection()
                cursor = connection.cursor()
                cursor.execute("SELECT * FROM Wizards WHERE wizardid=%s", (wizardid,))
                result = cursor.fetchall()
                if result is not None:
                    print("Wizard entry already exist")
                    isWizard=True
                else:
                    query = "INSERT INTO Wizards (wizardid, templateid) VALUES(%s, %s)"
                    values = (wizardid, templateid)
                    connection = getConnection()
                    execute(connection, query, values)

                connection = getConnection()
                cursor = connection.cursor()
                cursor.execute("SELECT * FROM Events WHERE eventid=%s", (int(line['event_id']),))
                result = cursor.fetchone()
                if result is not None:
                    print("Event entry aleready exist")
                else:
                    if isWizard:
                        query = "INSERT INTO Events (eventid, type, action, actiondate) VALUES(%s, %s, %s, NOW())"
                        values = (int(line['event_id']), str(line['event_type']), str(line['event_action']))
                        connection = getConnection()
                        execute(connection, query, values)
                    else:
                        query = "INSERT INTO Events (eventid, wizardid, templateid, type, action, actiondate) VALUES(%s, %s, %s, %s, %s, NOW())"
                        values = (int(line['event_id']), int(line['wizard_id']), int(line['template_id']), str(line['event_type']), str(line['event_action']))
                        connection = getConnection()
                        execute(connection, query, values)

            else:
                if templateid:
                    query = "INSERT INTO Events (eventid, templateid, type, action, actiondate) VALUES(%s, %s, %s, %s, NOW())"
                    values = (int(line['event_id']), templateid, str(line['event_type']), str(line['event_action']))
                    connection = getConnection()
                    execute(connection, query, values)
                else:
                    query = "INSERT INTO Events (eventid, type, action, actiondate) VALUES(%s, %s, %s, NOW())"
                    values = (int(line['event_id']), str(line['event_type']), str(line['event_action']))
                    connection = getConnection()
                    execute(connection, query, values)
    elif "templates.json" in file_name:
        for line in data.values():
            query = "INSERT INTO Template (templateid, type) VALUES(%s, %s)"
            values = (int(line['template_id']), str(line['type']))
            connection = getConnection()
            execute(connection, query, values)
    elif "wizards.json" in file_name:
        for line in data.values():
            wizardid = line['wizard_id']
            templateid = line['template_id']
            wtype = line['type']
            #print(f"wizardi: {wizardid} templateid: {templateid} type: {wtype}")
            if templateid:
                if wtype:
                    query = "INSERT INTO Wizards (wizardid, templateid, type) VALUES(%s, %s, %s)"
                    values = (int(wizardid), 1, wtype)
                    connection = getConnection()
                    execute(connection, query, values)
                else:
                    query = "INSERT INTO Wizards (wizardid, templateid) VALUES(%s, %s)"
                    values = (int(wizardid), 1)
                    connection = getConnection()
                    execute(connection, query, values)

if __name__ == '__main__':
    readFile("mock-events.json")
