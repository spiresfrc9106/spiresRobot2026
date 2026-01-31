from utils.singleton import Singleton
import hal
import threading
from .driverstation import DriverStation
from wpilib import DriverStation

class hubLightColor(metaclass=Singleton):

    def __init__(self):
      ourTurn(self)=True
      DriverStation.getAlliance()
      #feeds back blue or red
      data = wpilib.DriverStation.getGameSpecificMessage()
      pass  

    def getMatchTime(self):
       #The FMS does not currently send the official match time to the robots.
        #This returns the time since the enable signal sent from the Driver
        #Station.
        #At the beginning of autonomous, the time is reset to 0.0 seconds.
        #At the beginning of teleop, the time is reset to +15.0 seconds.
        #If the robot is disabled, this returns 0.0 seconds.

        return DriverStation.getInstance().getMatchTime()
    
    def update(self):

     
     ally = DriverStation.getAlliance()
     if ally is not None:
       if ally == DriverStation.Alliance.kRed:
       #RED ACTION
        if data:
         match data:
           case "R":
             if currentTime > 15 and < 25::
              ourTurn=True

             else currentTime > 50 and < 75:
             ourTurn=True

             elif currentTime > 100:
             ourTurn=True
             
             elif:
             ourTurn=False
            ...
           case "B":
             if currentTime > 15 and < 50:
              ourTurn=True

             else currentTime > 75 and < 100:
             ourTurn=True

             elif currentTime > 125:
             ourTurn=True

             elif:
             ourTurn=False
       elif ally == DriverStation.Alliance.kBlue:
       # <BLUE ACTION>
        if data:
         match data:
           case "B":
             if currentTime > 15 and < 25::
              ourTurn=True

             else currentTime > 50 and < 75:
             ourTurn=True

             elif currentTime > 100:
             ourTurn=True
             
             elif:
             ourTurn=False
            ...
           case "R":
             if currentTime > 15 and < 50:
              ourTurn=True

             else currentTime > 75 and < 100:
             ourTurn=True

             elif currentTime > 125:
             ourTurn=True

             elif:
             ourTurn=False
     else:
     #<NO COLOR YET ACTION>
      
        pass