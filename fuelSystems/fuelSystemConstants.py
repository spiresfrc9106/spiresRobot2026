from enum import Enum

#Constants file

class shooterTargetCmd(Enum): #These are just temporary/example names. We should do better when we decide on actual positions.
    MANUALMODE = 0 #Basically, tell turret to be under the direct control of the operator joysticks.
    CORNERONE = 1 
    CORNERTWO = 2

SHOOTER_OFFSET = .5 #How far the shooter is from the center of the robot in meters.
#My current unerstanding is that the turret will be in the back center of the robot.

SHOOTER_WHEEL_RADIUS = 0.0254 # meters converted from 2 inches  

GRAVITY = -9.8 #Meters/second