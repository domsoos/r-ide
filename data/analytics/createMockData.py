import json
import random
from datetime import datetime, timedelta
from sys import argv
from uuid import uuid4

wizards = {}
templates = {}
events = {}

wizardList = ["message node", "service node", "ROS Topic","ROS subscriber", "ROS monitor"]
templateList = ["service", "ROS topic", "ROS node", "urdf"]


def createWizardData(wizard_id, wizard_type, template_id=None):
    wizard = {
        'wizard_id': wizard_id,
        'template_id': template_id,
        'type': wizard_type
    }
    return wizard



def createTemplateData(template_id, template_type):
    template = {
        'template_id': template_id,
        'type': template_type
    }
    return template


def createEventData(event_id, template_id, event_type, event_action, event_action_time, wizard_id=None,
                    parent_id=None):
    event = {
        'event_id': event_id,
        'wizard_id': wizard_id,
        #'parent_id': parent_id,
        'template_id': template_id,
        'event_type': event_type,
        'event_action': event_action,
        'event_action_time': event_action_time.timestamp()
    }
    return event

def generateMockWizards(total):
    for _ in range(total):
        wizard = createWizardData(random.randint(1,4),random.choice(wizardList), random.randint(1,5))
        wizards[wizard['wizard_id']] = wizard


def generateMockTemplates(total):
    for i in range(total):
        newTemplate = createTemplateData(i+1, random.choice(wizardList))
        templates[newTemplate['template_id']] = newTemplate
"""
def generateMockTemplates(total):
    for i in range(total):
        newTemplate = createTemplateData(i+1, random.choice(wizardList))
        templates.append(newTemplate)
"""

def generateMockEvents(total):
    for i in range(total):
        event_id = i
        template_id = random.randint(5, 8)
        wizard_id = random.randint(6, 10)
        event_type = ''.join(chr(random.randrange(65, 123)) for _ in range(random.randrange(10, 45)))
        action = ''.join(chr(random.randrange(65, 123)) for _ in range(random.randrange(10, 45)))
        event_action = ''.join(chr(random.randrange(65, 123)) for _ in range(random.randrange(10, 45)))
        action_date = datetime.now()
        event_time = datetime.now()

        events[event_id] = createEventData(
            event_id,
            template_id,
            event_type,
            event_action,
            event_time,
            wizard_id
            #parent_id
        )


"""
def generateMockEvents(total):
    for _ in range(total):
        shipment_id = str(uuid4())
        template_id = random.choice(tuple(templates))
        if random.random() > 0.8:
            try:
                parent_id = random.choice(events)
            except KeyError:
                parent_id = None
        else:
            parent_id = None
        wizard = templates[template_id]['wizard_id']
        event_type = ''.join(chr(random.randrange(65, 123)) for _ in range(random.randrange(10, 45)))
        event_action = ''.join(chr(random.randrange(65, 123)) for _ in range(random.randrange(10, 45)))
        event_time = datetime.now() - timedelta(days=random.randrange(3), hours=random.randrange(24), minutes=random.randrange(60), seconds=random.randrange(60))

        events[shipment_id] = createEventData(
            shipment_id,
            template_id,
            event_type,
            event_action,
            event_time,
            wizard,
            parent_id
        )
"""

def saveToJSON(file_name):
    try:
        name, extension = file_name.split('.')
    except ValueError:
        name = file_name
        extension = 'json'

    with open(name + '-wizards.' + extension, 'w') as wizardFile:
        wizardFile.write(json.dumps(wizards, indent=4))

    with open(name + '-templates.' + extension, 'w') as templateFile:
        templateFile.write(json.dumps(templates, indent=4))

    with open(name + '-events.' + extension, 'w') as eventsFile:
        eventsFile.write(json.dumps(events, indent=4))


if __name__ == '__main__':
    generateMockWizards(int(argv[2]))
    generateMockTemplates(int(argv[3]))
    generateMockEvents(int(argv[4]))
    saveToJSON(argv[1])
