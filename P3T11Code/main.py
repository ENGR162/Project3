from __future__ import print_function
from __future__ import division
import brickpi3
import grovepi
import time
import math

#BrickPi_______________________________________________________________________
gyroNumReads = 5 #number of gyro read attempts until averaging
initialGyroAngle = "No Initial" #initial angle of gyro

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_GYRO_ABS) #set up Gyro

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #reset positions
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #reset positions
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #reset positions
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #reset positions

#TEST
print(BP.get_sensor(BP.PORT_4))
#GitTest Yolo
#TEST

#get initial angle
while initialGyroAngle == "No Initial":
    try:
        initialGyroAngle = BP.get_sensor(BP.PORT_4)
        
    except TypeError:
        initialGyroAngle = "No Initial"
        print ("Error")
            
    except IOError:
        initialGyroAngle = "No Initial"
        print ("Error")

#BrickPi_Functions_____________________________________________________________
def MeasureAngle():
    measurements = 0 #variable to store all measurements
    
    for i in range(gyroNumReads):
        
        successfulMeasures = 0 #variable to count all successful measures
        
        try:
            read = BP.get_sensor(BP.PORT_4) #storing angle from gyro
            measurements += read #adding angle to total variable
            successfulMeasures += 1 #registering as successful measure
            
        except TypeError:
            print ("Error")
            
        except IOError:
            print ("Error")
        
    return(measurements / successfulMeasures) #returns average value of angle

#______________________________________________________________________________

#GrovePi_______________________________________________________________________
ultraPort = 4 #ultrasonic sensor port number
ultraNumReads = 5 #number of ultrasonic read attempts until averaging

#GrovePi_Functions_____________________________________________________________
def MeasureDistance():
    measurements = 0 #variable to store all measurements
    
    for i in range(ultraNumReads):
        
        successfulMeasures = 0 #variable to count all successful measures
        
        try:
            read = grovepi.ultrasonicRead(ultraPort) #storing distance from ultrasonic
            measurements += read #adding distance to total variable
            successfulMeasures += 1 #registering as successful measure
            
        except TypeError:
            print ("Error")
            
        except IOError:
            print ("Error")
        
    return(measurements / successfulMeasures) #returns average value of distance
    
#______________________________________________________________________________
    
#Movement______________________________________________________________________
wheelDiameter = 25 #wheel diameter in cm

def ConvertSpeedToDps(linearSpeed):
    angularSpeed = 360 * linearSpeed / (wheelDiameter * math.pi)
    return(angularSpeed) #returns angular speed in dps
    
def ConvertDistanceToAngle(linearDistance):
    angle = 360 * linearDistance / (wheelDiameter * math.pi)
    return(angle) #returns distance in degrees with reference to wheeldiameter
    
def DriveStraightDistance(distance, travelTime):
    angularSpeed = ConvertSpeedToDps(distance / travelTime) #calculated desired speed
    
    initialMotorAngle = BP.get_motor_encoder(BP.PORT_B) #use motor port B as initial
    finalMotorAngle = ConvertDistanceToAngle(distance) + initialMotorAngle #final desired angle
    
    while ((math.fabs(finalMotorAngle) - math.fabs(BP.get_motor_encoder(BP.PORT_B))) > 0): #keep moving until moved set distance
        BP.set_motor_dps(BP.PORT_B, angularSpeed)
        BP.set_motor_dps(BP.PORT_C, angularSpeed)
        time.sleep(0.02)
        
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)
    
def DriveAtSpeed(linearSpeed, travelTime):
    angularSpeed = ConvertSpeedToDps(linearSpeed)
    
    BP.set_motor_dps(BP.PORT_B, angularSpeed)
    BP.set_motor_dps(BP.PORT_C, angularSpeed)
    
    time.sleep(travelTime)
    
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)

#______________________________________________________________________________

#Mapping_______________________________________________________________________
mapWidth = 5
mapHeight = 5

teamNumber = 11
mapNumber = 0
lengthOfGridUnit = 10
unit = "cm"
origin = [2,0]
notes = "This is an example map"

mapData = []

for i in range(mapWidth):
    mapData.append([])
    for j in range(mapHeight):
        mapData[i].append(0)

def WriteMapData():
    mapFile = open("team" + str(teamNumber) + "_map.txt", "w")
    mapFile.write("Team: " + str(teamNumber) + "\n")
    mapFile.write("Map: " + str(mapNumber) + "\n")
    mapFile.write("Unit Length: " + str(lengthOfGridUnit) + "\n")
    mapFile.write("Unit: " + unit + "\n")
    mapFile.write("Origin: (" + str(origin[0]) + "," + str(origin[1]) + ")" + "\n")
    mapFile.write("Notes: " + str(notes) + "\n")
    
    for i in range(len(mapData)):
        for j in range(len(mapData[0]) - 1):
            mapFile.write(str(mapData[i][j]) + ",")
        mapFile.write(str(mapData[i][-1]))
        mapFile.write("\n")
        
    mapFile.close()
        
def WriteMapHazard():
    pass
#______________________________________________________________________________