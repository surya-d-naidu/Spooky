from servo import setFromDict
import time
import json

stand_path = 'animations/stand.json'
walk_path = 'animations/walk.json'
rotate_path = 'animations/rotate.json'
sit_path = 'animations/sit.json'

def json_to_dict(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def animate_servos(data):
    for entry in data:
        positions = entry["positions"]
        duration = entry["duration"]

        setFromDict(positions)

        time.sleep(duration / 100.0)

stand = True
walk = False
sit = False

while True:
    if stand:
        animate_servos(json_to_dict(stand_path))
        time.sleep(5)
        stand = False
        walk = True
    if walk:
        for i in range(30):
            animate_servos(json_to_dict(walk_path))
        walk = False
        sit = True
    if sit:
        animate_servos(json_to_dict(sit_path))
        time.sleep(5)
        stand = True
        sit = False
