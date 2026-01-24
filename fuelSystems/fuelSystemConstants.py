from enum import Enum

#Constants file

class shooterTargetCmd(Enum): #These are just temporary/example names. We should do better when we decide on actual positions.
    MANUALMODE = 0 #Basically, tell turret to be under the direct control of the operator joysticks.
    CORNERONE = 1 
    CORNERTWO = 2

SHOOTEROFFSET = 0

SHOOTER_WHEEL_RADIUS = 0.0254 # meters converted from 2 inches  

GRAVITY = -9.8 #Meters/second