from servo import setFromDict as s
import json
import time

def walk():
    
    with open('animation_j/walk.json') as f:
        walk_frames = json.load(f)

    for i in range(len(walk_frames) - 1):
        x = (walk_frames[i])
        s(x)
        time.sleep(x['duration']/100)


def end_walk():
    with open('animation_j/walk.json') as f:
        walk_frames = json.load(f)

    x = (walk_frames[0])
    s(x)

def turn_left():
    with open('animation_j/left.json') as f:
        walk_frames = json.load(f)

    for i in range(len(walk_frames) - 1):
        x = (walk_frames[i])
        s(x)
        time.sleep(x['duration']/100)

def turn_right():
    with open('animation_j/right.json') as f:
        walk_frames = json.load(f)

    for i in range(len(walk_frames) - 1):
        x = (walk_frames[i])
        s(x)
        time.sleep(x['duration']/100)