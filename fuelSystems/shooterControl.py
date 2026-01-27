from utils.singleton import Singleton
from fuelSystems.fuelSystemConstants import shooterTargetCmd
from utils.signalLogging import addLog
from utils.constants import TURRET_PITCH_CANID, TURRET_YAW_CANID, TOP_SHOOTER_CANID, BOTTOM_SHOOTER_CANID, blueHubLocation, redHubLocation
from wrappers.wrapperedSparkMax import WrapperedSparkMax
from wrappers.wrapperedKraken import WrapperedKraken
from drivetrain.poseEstimation.drivetrainPoseEstimator import DrivetrainPoseEstimator
from drivetrain.drivetrainControl import DrivetrainControl
import math 
from fuelSystems.fuelSystemConstants import GRAVITY, SHOOTER_WHEEL_RADIUS

class ShooterController(metaclass=Singleton):

    def __init__(self): 
        #TODO -- ADD A CHECK TO PREVENT US FROM TRYING TO GO PAST OUR MAXIMUM ANGLES. from 5 to 67 degrees. 
        #
        
        # motor declerations here
        #2 krakens for the shooter wheels
        self.shooterTopMotor = WrapperedKraken(TOP_SHOOTER_CANID, "ShooterMotorTop", brakeMode=False)
        self.shooterBottomMotor = WrapperedKraken(BOTTOM_SHOOTER_CANID, "ShooterMotorBottom", brakeMode=False)

        # 2 neo 550s (Controlled by SparkMaxes) for pitch/yaw
        self.pitchMotor = WrapperedSparkMax(TURRET_PITCH_CANID, "TurretMotorPitch", brakeMode=True)
        self.yawMotor = WrapperedSparkMax(TURRET_YAW_CANID, "TurretMotorYaw", brakeMode=True)

        self.curPos = self.robotPosEst.getCurEstPose()

        self.toldToShoot = True

        self.currentTargetCommand = shooterTargetCmd.CORNERONE

        self.hubTrajectoryMaxHeight = 2.25 #Currently meters? rounded after converting the 7 ft example in elliots equations
        self.hubTrajectoryVertexOffset = .304 #Also in meters

        self.driveTrainCtrl = DrivetrainControl()
        self.robotPosEst = DrivetrainPoseEstimator((self.driveTrainCtrl.getModulePositions()))

        pass

    def update(self):
        
        #Right now software is assuming that we will only move the turret when the shoot button is held down
        if self.toldToShoot: 
            
            #Need to know robot rotation in the future, for now we assume ideal robot that never turns (kinda). 
            #Also currently assuming super strong ideal motors that have no gearboxes. 
            #We all love ideal software land

            self.robotRelTurretYaw = self.yawMotor.getMotorPositionRad()
            self.robotRelTurretPitch = self.pitchMotor.getMotorPositionRad()

            #Calculate the ideal ball velocity Magnitude and Direction so it will make it to our target
            #This is "Traejctory Relative," X axis is the line from the base of the robot at the center of the turret to
            #the base of the hub at the center  

            self.oldPos = self.curPos 
            self.curPos = self.robotPosEst.getCurEstPose()
            self.curHubPos = blueHubLocation #THIS DOESN'T CHECK WHICH ALLIANCE WE ARE I NEED TO IMPLEMENT THAT LATER

            #Oh here calculate the turret's position relative to the feild (for if the turret isn't in the center of our robot). 
            #Right now ignoring this to have simpler starting code.
            #Relative to the feild
            self.turretPosX = self.curPos.X() 
            self.turretPosY = self.curPos.Y()

            #Get distance to hub
            self.hubTurretXDiff = self.curHubPos.X() - self.turretPosX
            self.hubTurretYDiff = self.curHubPos.Y() - self.turretPosY

            self.distToHub = math.sqrt((self.hubTurretXDiff) ** 2 + (self.hubTurretYDiff) ** 2)

            #Use distance to hub to calculate desired velocity and angle -- 
            self.desTrajVelo = math.sqrt((2*GRAVITY*self.hubTrajectoryMaxHeight)/(2*abs(GRAVITY)))
            self.desTrajPitch = ((2*self.hubTrajectoryMaxHeight)/(self.desTrajVelo))

            # Get robots velocity by measuring distance traveled since last cycle and
            # dividing it by time.

            self.robotCycleTime = 0.02

            self.robotXVelo = (self.curPos.X() - self.oldPos.X()) / self.robotCycleTime
            self.robotYVelo = (self.curPos.Y() - self.oldPos.Y()) / self.robotCycleTime

            #Convert the robot's velocity to be relative to our trajectory-freindly axis from its own relative one.
            #The angle difference between the field axis and the trajectory one.

            self.robotToTrajAxisDiff = math.atan(self.turretPosX / self.turretPosY) #Task for my future self -- Check if this
            self.robotTrajRelVeloX = 1 / (math.sin(self.robotToTrajAxisDiff) * self.robotXVelo) #code accidentally flips
            self.robotTrajRelVeloY = 1 / (math.cos(self.robotToTrajAxisDiff) * self.robotYVelo) #X and Y axis

            #All of the components of the vector for the needed ball velocity to score
            self.neededBallXVelo = math.cos(self.desTrajPitch) * self.desTrajVelo - self.robotTrajRelVeloX
            self.neededBallZVelo = math.sin(self.desTrajPitch) * self.desTrajVelo 
            self.neededBallYVelo = self.robotTrajRelVeloY

            #Convert the components of the needed ball velocity vector to a magnitude, yaw and pitch. 
            self.neededBallVelo = math.sqrt((self.neededBallXVelo) ** 2 + (self.neededBallYVelo) ** 2 + (self.neededBallZVelo) ** 2)
            self.neededBallYaw = math.atan((-1 * self.robotTrajRelVeloY) / (self.neededBallXVelo)) 
            self.neededBallPitch = math.atan((self.neededBallZVelo) / (self.neededBallXVelo)) 

            #Now we correct the yaw so it is relative to robot's current direction instead of our ideal trajectory axis
            self.neededTurretYaw = (self.neededBallPitch - self.robotToTrajAxisDiff) + self.robotPosEst.getCurEstPose().rotation().radians()
            self.neededTurretpitch = self.neededBallPitch

            #Now all thats left is figure out the rotational velocity of the wheels:
            self.neededShooterRotVelo = self.neededBallVelo / SHOOTER_WHEEL_RADIUS 
            
            #so by this point hopefully all we need to do is point turret to self.neededTurretYaw and self.neededTurretPitch
            #and set the rotational velocity of the motors to self.neededShooterRotVelo (After compensating for gear of course)
            #and we'll be golden.

            #i think?

            #hopefully

            self.shooterTopMotor.setVelCmd(self.neededShooterRotVelo)
            self.shooterBottomMotor.setVelCmd(self.neededShooterRotVelo)

            
        pass

    def setTargetCmd(self, targetCommand):
        pass

    def setShooting(self, shooterCommand):
        self.toldToShoot = True
        pass

    def getIdealTrajectoryPitch(self):
        #return information
        pass
    
    def getIdealTrajectoryYaw(self):
        #return information
        pass

    def getIdealTopWheelSpeed(self):
        #return information
        pass
    
    def getIdealBottomWheelSpeed(self):
        #return information
        pass