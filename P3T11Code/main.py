from __future__ import print_function
from __future__ import division
import brickpi3
import grovepi
import time
import math

#Setup_Instructions____________________________________________________________
"""
1. Face bot north in relation to the map when starting program
2. Face ultrasonic sensor to the front of the bot before starting
3. Determine if wheelDiameter variable is accurate for wheels/tracks
"""

#BrickPi_______________________________________________________________________
gyroNumReads = 5 #number of gyro read attempts until averaging
ultraNumReads = 5 #number of ultrasonic read attempts until averaging
initialGyroAngle = 0

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS) #set up Gyro sensor
BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM) #set up Ultrasonic sensor

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #reset positions
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #reset positions
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #reset positions
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #reset positions

#Initial Angle and Measure Setup_______________________________________________

gyroTestBool = True
while gyroTestBool:
    try:
        temp = BP.get_sensor(BP.PORT_4) #test sensor
        initialGyroAngle = temp[0]
        gyroTestBool = False
    
    except TypeError:
        gyroTestBool = True #set error variable
        print ("Initial Angle TypeError") #print error
    
    except IOError:
        gyroTestBool = True #set error variable
        print ("Initial Angle IOError") #print error

    except brickpi3.SensorError:
        gyroTestBool = True #set error variable
        print("Initial Gyro Sensor Error") #print error

ultraTestBool = True
while ultraTestBool:
    try:
        ultraTest = BP.get_sensor(BP.PORT_3) #test sensor
        ultraTestBool = False
    
    except TypeError:
        ultraTestBool = True #set error variable
        print ("Initial Distance TypeError") #print error
    
    except IOError:
        ultraTestBool = True #set error variable
        print ("Initial Distance IOError") #print error

    except brickpi3.SensorError:
        ultraTestBool = True #set error variable
        print("Initial Ultra Sensor Error") #print error

#______________________________________________________________________________

#BrickPi_Functions_____________________________________________________________
def MeasureAngle():
    measurements = 0 #variable to store all measurements
    successfulMeasures = 0 #variable to count all successful measures
    
    for i in range(gyroNumReads):
        try:
            read = BP.get_sensor(BP.PORT_4)[0] #storing angle from gyro
            measurements += read #adding angle to total variable
            successfulMeasures += 1 #registering as successful measure
            
        except TypeError as error:
            print ("Gyro Read TypeError") #print read error
            
        except IOError:
            print ("Gyro Read IOError") #print read error
            
        except brickpi3.SensorError:
            print("Gyro Read Sensor Error") #print read error

    if(successfulMeasures == 0): #if no successful measures
        print("No Successful Angle Measures")
        return("No Successful Angle Measures")
    else:
        return((measurements / successfulMeasures) - initialGyroAngle) #returns average value of angle
        

def MeasureDistance():
    measurements = 0 #variable to store all measurements
    successfulMeasures = 0 #variable to count all successful measures
    
    for i in range(ultraNumReads):
        try:
            read = BP.get_sensor(BP.PORT_3) #storing distance from ultrasonic
            measurements += read #adding distance to total variable
            successfulMeasures += 1 #registering as successful measure
            
        except TypeError:
            print ("TypeError") #print read error
            
        except IOError:
            print ("IOError") #print read error
            
        except brickpi3.SensorError:
            print("Sensor Error") #print read error

    if(successfulMeasures == 0): #if no successful measures
        print("No Successful Distance Measures")
        return("No Successful Distance Measures")
    else:
        return(measurements / successfulMeasures) #returns average value of distance
    
#______________________________________________________________________________

#GrovePi_______________________________________________________________________
#ultraPort = 3 #ultrasonic sensor port number

#GrovePi_Functions_____________________________________________________________
"""
def MeasureDistanceGrove():
    measurements = 0 #variable to store all measurements
    successfulMeasures = 0 #variable to count all successful measures
    
    for i in range(ultraNumReads):
        try:
            read = grovepi.ultrasonicRead(ultraPort) #storing distance from ultrasonic
            measurements += read #adding distance to total variable
            successfulMeasures += 1 #registering as successful measure
            
        except TypeError:
            print ("TypeError")
            
        except IOError:
            print ("IOError")

    print(successfulMeasures)
    print(measurements)

    if(successfulMeasures == 0):
        return(math.inf)
    else:
        return(measurements / successfulMeasures) #returns average value of distance
""" 
#______________________________________________________________________________
    
#Movement_And_Bot_Mechanics____________________________________________________
wheelDiameter = 25 #wheel diameter in cm
currentHeading = 0

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

def TurnToAngle(desiredAngle):
    currentAngle = MeasureAngle()
    kP = 0.5
    kI = 0.5
    angleThreshold = 0.1
    
    while not((currentAngle < desiredAngle + angleThreshold) and (currentAngle > desiredAngle - angleThreshold)):
        angularSpeedPorp = -kP * (desiredAngle - currentAngle)
        angularSpeedIntegral = -kI * (desiredAngle - currentAngle) * 0.02
        angularSpeedTotal = angularSpeedPorp + angularSpeedIntegral

        print(str(currentAngle) + " cur")
        print(str(angularSpeedPorp) + " prop")
        print(str(angularSpeedIntegral) + " int")
        
        BP.set_motor_dps(BP.PORT_B, -angularSpeedTotal)
        BP.set_motor_dps(BP.PORT_C, angularSpeedTotal)
        currentAngle = MeasureAngle()
        time.sleep(0.02)

    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)

def TurnByAngle(angle):
    targetAngle = MeasureAngle() + angle
    while (targetAngle > 360 or targetAngle < -360):
        if(targetAngle > 360):
            targetAngle -= 360
        if(targetAngle < -360):
            targetAngle += 360
    
    TurnToAngle(targetAngle)

def RotateVisorToAngle(angle):
    BP.set_motor_position(BP.PORT_D, angle)

def ScanFront():
    RotateVisorToAngle(0)
    time.sleep(0.05)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Front")
    return(dis)

def ScanRear():
    RotateVisorToAngle(-180)
    time.sleep(0.05)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Rear")
    return(dis)

def ScanRight():
    RotateVisorToAngle(-270)
    time.sleep(0.05)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Right")
    return(dis)

def ScanLeft():
    RotateVisorToAngle(-90)
    time.sleep(0.05)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Left")
    return(dis)

def ReturnVisorToFrontView():
    RotateVisorToAngle(0)
    print("Returned view to front")

def ScanAllSides():
    distanceScans = [0, 0, 0, 0]
    distanceScans[0] = ScanFront()
    distanceScans[1] = ScanLeft()
    distanceScans[2] = ScanRear()
    distanceScans[3] = ScanRight()
    ReturnVisorToFrontView()
    return(distanceScans)
    
#______________________________________________________________________________

#Mapping_______________________________________________________________________

#Utility_Mapping_Key___________________________________________________________
"""
GEARS Operation Key #Use during operation

0 = Unknown
1 = Open but untraveled path
2 = Traveled path and clear
3 = Origin
4 = IR Danger grid location
5 = Magnetic Danger grid location
6 = Exit point

GEARS Output Key #Use for output

0 = Not part of the path
1 = Path GEARS took
2 = Heat Source
3 = Magnetic Source
4 = Exit point
5 = Origin(starting point of GEARS)
"""
#______________________________________________________________________________

mapWidth = 5
mapHeight = 5

teamNumber = 11
mapNumber = 0
GridUnit = 10
unit = "cm"
origin = [2,0]
mapNotes = "This is an example map"

hazardsNotes = "This is an example of resource information"

mapData = []

#fixes unit mismatches
if(unit == "cm"):
    lengthOfGridUnit = GridUnit
else:
    lengthOfGridUnit = GridUnit * 2.54

#MapArraySetup_________________________________________________________________
#Initialize mapData as 0's
for i in range(mapWidth):
    mapData.append([])
    for j in range(mapHeight):
        mapData[i].append(0)

hazardsFile = open("team" + str(teamNumber) + "_hazards.txt", "w") #write initial hazard file
hazardsFile.write("Team: " + str(teamNumber) + "\n") #write team num
hazardsFile.write("Map: " + str(mapNumber) + "\n") #write map num
hazardsFile.write("Notes: " + str(hazardsNotes) + "\n") #write hazard notes
hazardsFile.write("\nResource Type, Parameter of Interest, Parameter, Resource X Coordinate, Resource Y Coordinate\n") #write headers
hazardsFile.close() #close hazard file

#MapArrayFunctions_____________________________________________________________
        
def WriteMapData():
    mapFile = open("team" + str(teamNumber) + "_map.txt", "w") #create map file
    mapFile.write("Team: " + str(teamNumber) + "\n") #write team number
    mapFile.write("Map: " + str(mapNumber) + "\n") #write map number
    mapFile.write("Unit Length: " + str(GridUnit) + "\n") #write grid unit
    mapFile.write("Unit: " + unit + "\n") #write unit of grid
    mapFile.write("Origin: (" + str(origin[0]) + "," + str(origin[1]) + ")" + "\n") #write origin point
    mapFile.write("Notes: " + str(mapNotes) + "\n") #write map notes
    
    for i in range(mapWidth):
        for j in range(mapHeight - 1):
            mapFile.write(str(mapData[i][j]) + ",") #write single data point
        mapFile.write(str(mapData[i][-1])) #write last data point
        mapFile.write("\n") #skip to next line for writing
        
    mapFile.close() #close map file
        
def WriteMapHazard(hazardType, parameterValue, gridX, gridY):
    hazardsFile = open("team" + str(teamNumber) + "_hazards.txt", "a") #append to add new hazard

    if(hazardType == "IR"): #write heat source hazard
        hazardsFile.write("High Temperature Heat Source, ") #write hazard type
        hazardsFile.write("Test Unit(unit), ") #write parameter unit
        hazardsFile.write(str(parameterValue) + ", ") #write parameter
        hazardsFile.write(str((gridX - origin[0]) * GridUnit) + unit + ", ") #write xLoc with repect to origin
        hazardsFile.write(str((gridY - origin[1]) * GridUnit) + unit + "\n") #write yLoc with repect to origin
        
    elif(hazardType == "Magnet"): #write magnet field hazard
        hazardsFile.write("Electrical Activity Source, ") #write hazard type
        hazardsFile.write("Test Unit(unit), ") #write parameter unit
        hazardsFile.write(str(parameterValue) + ", ") #write parameter
        hazardsFile.write(str((gridX - origin[0]) * GridUnit) + unit + ", ") #write xLoc with repect to origin
        hazardsFile.write(str((gridY - origin[1]) * GridUnit) + unit + "\n") #write yLoc with repect to origin
    
    hazardsFile.close()

def ConvertWorkMapToFinalMap():
    #converts first value to second value
    #per element in conversionArray
    conversionArray = [[0, 0], [1, 0], [2, 1], [3, 5], [4, 2], [5, 3], [6, 4]]
    
    for i in range(mapWidth):
        for j in range(mapHeight):
            for k in range(len(conversionArray)):
                if(mapData[i][j] == conversionArray[k][0]):
                    mapData[i][j] = conversionArray[k][1]
                    break
    
#______________________________________________________________________________

try:
    #pass
    ScanAllSides()
    while True:
        #print(MeasureAngle())
        time.sleep(0.02)
        
except KeyboardInterrupt:
    BP.reset_all()
