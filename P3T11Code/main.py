from __future__ import print_function
from __future__ import division
import MPU9250
import brickpi3
import grovepi
import IR_Functions
import time
import math

#Setup_Instructions____________________________________________________________
"""
# 1. Face bot north in relation to the map when starting program
# 2. Face ultrasonic sensor to the front of the bot before starting
# 3. Determine if wheelDiameter variable is accurate for wheels/tracks
"""
#MPU___________________________________________________________________________
mpu = MPU9250.MPU9250()

#BrickPi_______________________________________________________________________
gyroNumReads = 5 #number of gyro reads until averaging
ultraNumReads = 5 #number of ultrasonic reads until averaging
hallNumReads = 5 #number of hall reads until averaging
irNumReads = 5 #number of IR sensor reads until averaging
magNumReads = 5 #number of IMU magnetic sensor reads until averaging
initialGyroAngle = 0 #maybe unesscessary var

ultraRightPort = 8 #port number of right side ultrasonic sensor
ultraLeftPort = 3 #port number of left side ultrasonic sensor

BP = brickpi3.BrickPi3()
BP.set_sensor_type(BP.PORT_4, BP.SENSOR_TYPE.EV3_GYRO_ABS_DPS) #set up Gyro sensor
BP.set_sensor_type(BP.PORT_3, BP.SENSOR_TYPE.EV3_ULTRASONIC_CM) #set up Ultrasonic sensor
BP.set_sensor_type(BP.PORT_1, BP.SENSOR_TYPE.CUSTOM, [(BP.SENSOR_CUSTOM.PIN1_ADC)]) #set up Hall sensor

BP.offset_motor_encoder(BP.PORT_A, BP.get_motor_encoder(BP.PORT_A)) #reset positions
BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #reset positions
BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #reset positions
BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #reset positions

#Grovepi_

IR_Functions.IR_setup(grovepi)

#Initial_Setup_________________________________________________________________

#Gyro sensor
gyroTestBool = True
while gyroTestBool:
    try:
        temp = BP.get_sensor(BP.PORT_4) #test sensor
        initialGyroAngle = temp[0]
        gyroTestBool = False
    
    except TypeError:
        print ("Initial Angle TypeError") #print error
    
    except IOError:
        print ("Initial Angle IOError") #print error

    except brickpi3.SensorError:
        print("Initial Gyro Sensor Error") #print error

#Ultrasonic visor sensor
ultraTestBool = True
while ultraTestBool:
    try:
        test = BP.get_sensor(BP.PORT_3) #test sensor
        ultraTestBool = False
    
    except TypeError:
        print ("Initial Distance TypeError") #print error
    
    except IOError:
        print ("Initial Distance IOError") #print error

    except brickpi3.SensorError:
        print("Initial Ultra Sensor Error") #print error

#Ultrasonic right sensor
ultraTestRightBool = True
while ultraTestRightBool:
    try:
        test = grovepi.ultrasonicRead(ultraRightPort) #test sensor
        ultraTestRightBool = False
    
    except TypeError:
        print ("Initial Distance Right TypeError") #print error
    
    except IOError:
        print ("Initial Distance Right IOError") #print error

#Ultrasonic left sensor
ultraTestLeftBool = True
while ultraTestLeftBool:
    try:
        test = grovepi.ultrasonicRead(ultraLeftPort) #test sensor
        ultraTestLeftBool = False
    
    except TypeError:
        print ("Initial Distance Left TypeError") #print error
    
    except IOError:
        print ("Initial Distance Left IOError") #print error

#IR sensor
irTestBool = True
while irTestBool:
    try:
        test = IR_Functions.IR_Read(grovepi) #test sensor
        irTestBool = False
    
    except TypeError:
        print ("IR TypeError") #print error
    
    except IOError:
        print ("IR IOError") #print error

#IMU sensor
imuTestBool = True
while imuTestBool:
    try:
        test = mpu.readMagnet() #test sensor
        imuTestBool = False
    
    except TypeError:
        print ("IMU TypeError") #print error
    
    except IOError:
        print ("IMU IOError") #print error
#______________________________________________________________________________

#BrickPi_Functions_____________________________________________________________
def MeasureAngle():
    measurements = 0.0 #variable to store all measurements
    successfulMeasures = 0.0 #variable to count all successful measures
    
    while successfulMeasures < gyroNumReads:
        try:
            read = BP.get_sensor(BP.PORT_4)[0] #storing angle from gyro
            measurements += read #adding angle to total variable
            successfulMeasures += 1.0 #registering as successful measure
            
        except TypeError:
            print ("Gyro Read TypeError") #print read error
            
        except IOError:
            print ("Gyro Read IOError") #print read error
            
        except brickpi3.SensorError:
            print ("Gyro Read Sensor Error") #print read error

    #print((measurements / successfulMeasures) - initialGyroAngle)
    return((measurements / successfulMeasures) - initialGyroAngle) #returns average value of angle

def MeasureDistance():
    measurements = 0.0 #variable to store all measurements
    successfulMeasures = 0.0 #variable to count all successful measures
    
    while successfulMeasures < ultraNumReads:
        try:
            read = BP.get_sensor(BP.PORT_3) #storing distance from ultrasonic
            measurements += read #adding distance to total variable
            successfulMeasures += 1.0 #registering as successful measure
            
        except TypeError:
            print ("Ultra Read TypeError") #print read error
            
        except IOError:
            print ("Ultra Read IOError") #print read error
            
        except brickpi3.SensorError:
            print ("Ultra Read Sensor Error") #print read error

    return(measurements / successfulMeasures) #returns average value of distance
#______________________________________________________________________________

#GrovePi_Functions_____________________________________________________________
def MeasureSideDistance(port):
    measurements = 0.0 #variable to store all measurements
    successfulMeasures = 0.0 #variable to count all successful measures
    
    while successfulMeasures < ultraNumReads:
        try:
            read = grovepi.ultrasonicRead(port)#storing distance from ultrasonic
            measurements += read #adding distance to total variable
            successfulMeasures += 1.0 #registering as successful measure
            
        except TypeError:
            print ("Ultra Read Side TypeError") #print read error
            
        except IOError:
            print ("Ultra Read Side IOError") #print read error

    sideDis = measurements / successfulMeasures
    #clamping value
    if(sideDis > 10.0):
        sideDis = 10.0
    elif(sideDis < -10.0):
        sideDis = -10.0

    #print("Port: " + str(port) + " Dist: " + str(sideDis))
    return(sideDis) #returns average value of distance

def MeasureRightLeftSideDis():
    #Left distance is first in array, then Right distance
    return ([MeasureSideDistance(ultraLeftPort), MeasureSideDistance(ultraRightPort)])

def MeasureIR():
    measurements = [0.0, 0.0] #variable to store all measurements
    successfulMeasures = 0.0 #variable to count all successful measures
    
    while successfulMeasures < irNumReads:
        try:
            read = IR_Functions.IR_Read(grovepi) #storing IR from IR sensor
            measurements[0] += read[0] #adding IR one to total variable
            measurements[1] += read[1] #adding IR two to total variable
            successfulMeasures += 1.0 #registering as successful measure
            
        except TypeError:
            print ("IR Read TypeError") #print read error
            
        except IOError:
            print ("IR Read IOError") #print read error

    return([measurements[0] / successfulMeasures, measurements[1] / successfulMeasures]) #returns average value of IR

#______________________________________________________________________________

#IMU_Functions_________________________________________________________________

def MeasureMagneticFields():

    fields = [0.0, 0.0, 0.0] #variable to store all measurements
    successfulMeasures = 0.0 #variable to count all successful measures
    
    while successfulMeasures < magNumReads:
        try:
            read = mpu.readMagnet() #IMU magnetic reading
            fields[0] += read["x"]
            fields[1] += read["y"]
            fields[2] += read["z"]
            successfulMeasures += 1.0 #registering as successful measure
            
        except TypeError:
            print ("IMU Magnetic Read TypeError") #print read error
            
        except IOError:
            print ("IMU Magnetic Read IOError") #print read error

    return([fields[0] / successfulMeasures, fields[1] / successfulMeasures, fields[2] / successfulMeasures]) #returns average value of Magnetic Field Directions as list

#______________________________________________________________________________

#Mapping_______________________________________________________________________

#Utility_Mapping_Key___________________________________________________________
"""
# GEARS Operation Key #Use during operation
#
# 0 = Unknown
# 1 = Open but untraveled path
# 2 = Traveled path and clear
# 3 = Origin
# 4 = IR Danger grid location
# 5 = Magnetic Danger grid location
# 6 = Exit point untraveled
# 7 = Exit point traveled
#
# GEARS Output Key #Use for output
#
# 0 = Not part of the path
# 1 = Path GEARS took
# 2 = Heat Source
# 3 = Magnetic Source
# 4 = Exit point
# 5 = Origin(starting point of GEARS)
"""
#______________________________________________________________________________

mapWidth = 5
mapHeight = 5

teamNumber = 11
mapNumber = 0
GridUnit = 40
unit = "cm"
origin = [2,2]
mapNotes = "This is an example map"
hazardsNotes = "This is an example of resource information"

mapData = [[0,0,0,0,0],[0,0,1,0,0],[0,0,0,0,0],[0,0,0,0,0],[0,0,0,0,0]]

currentHeading = "North" #Setting inital heading
gearsCurrentGridLocation = origin

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

mapData[origin[0]][origin[1]] = 3 #change the mapdata to have origin location

hazardsFile = open("team" + str(teamNumber) + "_hazards.txt", "w") #write initial hazard file
hazardsFile.write("Team: " + str(teamNumber) + "\n") #write team num
hazardsFile.write("Map: " + str(mapNumber) + "\n") #write map num
hazardsFile.write("Notes: " + str(hazardsNotes) + "\n") #write hazard notes
hazardsFile.write("\nResource Type, Parameter of Interest, Parameter, Resource X Coordinate, Resource Y Coordinate\n") #write headers
hazardsFile.close() #close hazard file

#MapArrayFunctions_____________________________________________________________

def WriteUtilityMapData():
    mapFile = open("team" + str(teamNumber) + "_map_utility.txt", "w") #create map file
    mapFile.write("Team: " + str(teamNumber) + "\n") #write team number
    mapFile.write("Map: " + str(mapNumber) + "\n") #write map number
    mapFile.write("Unit Length: " + str(GridUnit) + "\n") #write grid unit
    mapFile.write("Unit: " + unit + "\n") #write unit of grid
    mapFile.write("Origin: (" + str(origin[0]) + "," + str(origin[1]) + ")" + "\n") #write origin point
    mapFile.write("Notes: " + str(mapNotes) + "\n") #write map notes

    for j in range(mapHeight - 1, -1, -1):
        for i in range(mapWidth - 1):
            mapFile.write(str(mapData[i][j]) + ",") #write single data point
        mapFile.write(str(mapData[-1][j])) #write last data point
        mapFile.write("\n") #skip to next line for writing
        
    mapFile.close() #close map file
    
def WriteMapData():
    mapFile = open("team" + str(teamNumber) + "_map.txt", "w") #create map file
    mapFile.write("Team: " + str(teamNumber) + "\n") #write team number
    mapFile.write("Map: " + str(mapNumber) + "\n") #write map number
    mapFile.write("Unit Length: " + str(GridUnit) + "\n") #write grid unit
    mapFile.write("Unit: " + unit + "\n") #write unit of grid
    mapFile.write("Origin: (" + str(origin[0]) + "," + str(origin[1]) + ")" + "\n") #write origin point
    mapFile.write("Notes: " + str(mapNotes) + "\n") #write map notes

    for j in range(mapHeight - 1, -1, -1):
        for i in range(mapWidth - 1):
            mapFile.write(str(mapData[i][j]) + ",") #write single data point
        mapFile.write(str(mapData[-1][j])) #write last data point
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
    conversionArray = [[0, 0], [1, 0], [2, 1], [3, 5], [4, 2], [5, 3], [6, 4], [7, 4]]
    
    for i in range(mapWidth):
        for j in range(mapHeight):
            for k in range(len(conversionArray)):
                if(mapData[i][j] == conversionArray[k][0]):
                    mapData[i][j] = conversionArray[k][1]
                    break


def UpdateMapBasedOnScans(distances):

    tileDistances = [0, 0, 0, 0]

    for i in range(len(distances)):
        tileDistances[i] = round((distances[i] - (lengthOfGridUnit / 4)) / lengthOfGridUnit)

    print(tileDistances)

    #structure for coordinate transformations
    if(currentHeading == "North"):
        UpdateSingleDirectionOfMapAsOpen(tileDistances[0], "North")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[1], "West")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[2], "South")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[3], "East")
        
    elif(currentHeading == "West"):
        UpdateSingleDirectionOfMapAsOpen(tileDistances[3], "North")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[0], "West")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[1], "South")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[2], "East")
        
    elif(currentHeading == "South"):
        UpdateSingleDirectionOfMapAsOpen(tileDistances[2], "North")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[3], "West")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[0], "South")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[1], "East")
        
    elif(currentHeading == "East"):
        UpdateSingleDirectionOfMapAsOpen(tileDistances[1], "North")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[2], "West")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[3], "South")
        UpdateSingleDirectionOfMapAsOpen(tileDistances[0], "East")

def UpdateSingleDirectionOfMapAsOpen(tileDistance, direction):
    
    if(direction == "North"):
        for i in range(1, tileDistance + 1):
            if(SetGridLocationToClear([gearsCurrentGridLocation[0], gearsCurrentGridLocation[1] + i])):
                break
            
    elif(direction == "West"):
        for i in range(1, tileDistance + 1):
            if(SetGridLocationToClear([gearsCurrentGridLocation[0] - i, gearsCurrentGridLocation[1]])):
                break
            
    elif(direction == "South"):
        for i in range(1, tileDistance + 1):
            if(SetGridLocationToClear([gearsCurrentGridLocation[0], gearsCurrentGridLocation[1] - i])):
                break
        
    elif(direction == "East"):
        for i in range(1, tileDistance + 1):
            if(SetGridLocationToClear([gearsCurrentGridLocation[0] + i, gearsCurrentGridLocation[1]])):
                break

def SetGridLocationValue(location, value):
    if(value == 2):
        if(mapData[location[0]][location[1]] == 1):
            mapData[location[0]][location[1]] = 2
        elif(mapData[location[0]][location[1]] == 6):
            mapData[location[0]][location[1]] = 7

    else:
        mapData[location[0]][location[1]] = value

def SetGridLocationToClear(location): #returns True if found exit

    desiredValue = -1
    desiredLocation = location
    
    try: #if location is on map

        #need fixing for index out of bounds errors
        if(not(location[0] > (mapWidth - 1)) and not(location[1] > (mapHeight - 1))):
            pointValue = mapData[location[0]][location[1]]            

        #Only change unknown tiles
        desiredValue = 1
        desiredLocation = location

        if(location[0] < 0):
            desiredValue = 6
            desiredLocation = [0, location[1]]
        
        elif(location[0] > (mapWidth - 1)):    
            desiredValue = 6
            desiredLocation = [mapWidth - 1, location[1]]
        
        elif(location[1] < 0):
            desiredValue = 6
            desiredLocation = [location[0], 0]
        
        elif(location[1] > (mapHeight - 1)):
            desiredValue = 6
            desiredLocation = [location[0], mapHeight - 1]

    except IndexError: #if location is off map
        print("Index Error")
        pointValue = -1
        desiredValue = -1

    if(desiredValue == 6):
        if(mapData[desiredLocation[0]][desiredLocation[1]] == 0):
            SetGridLocationValue(desiredLocation, desiredValue)
            return True

        elif(mapData[desiredLocation[0]][desiredLocation[1]] == 1):
            SetGridLocationValue(desiredLocation, desiredValue)
            return True

        elif(mapData[desiredLocation[0]][desiredLocation[1]] == 2):
            SetGridLocationValue(desiredLocation, desiredValue)
            return True

    elif(desiredValue == 1):
        if(pointValue == 0): #Only change unknown tiles
            SetGridLocationValue(desiredLocation, desiredValue)

    return False
        
#______________________________________________________________________________
                
#Movement_And_Bot_Mechanics____________________________________________________
wheelDiameter = 2.480 #wheel diameter in cm

def ConvertSpeedToDps(linearSpeed):
    angularSpeed = 360 * linearSpeed / (wheelDiameter * math.pi)
    return(angularSpeed) #returns angular speed in dps
    
def DriveStraightDistance(distance, travelTime):

    if(travelTime == 0):
        return
    
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #reset positions
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #reset positions
    
    angularSpeed = ConvertSpeedToDps(distance / travelTime) #calculated desired speed
    
    initialMotorAngle = BP.get_motor_encoder(BP.PORT_B) #use motor port B as initial
    finalMotorAngle = ConvertSpeedToDps(distance) + initialMotorAngle #final desired angle
    
    while ((math.fabs(finalMotorAngle) - math.fabs(BP.get_motor_encoder(BP.PORT_B))) > 0): #keep moving until moved set distance
        BP.set_motor_dps(BP.PORT_B, angularSpeed)
        BP.set_motor_dps(BP.PORT_C, angularSpeed)
        time.sleep(0.02)
        
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)

def DriveCorridor(distance, travelSpeed = 10.0, sideFact = 0.2, angleFact = 0.05): #advanced funtion to maintain wall distance 

    if(distance == 0 or travelSpeed == 0):
        return

    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)
    BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) #reset positions
    BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) #reset positions

    RotateVisorToAngle(8)
    startingAngle = MeasureAngle()
    distanceToTravel = ConvertSpeedToDps(distance)
    currentDistance = 0.0

    while distanceToTravel > 0:
        frontDis = MeasureDistance()

        if(frontDis <= 20):
            break

        #measures
        currentAngle = MeasureAngle() - startingAngle
        sideDistances = MeasureRightLeftSideDis()
        distanceGap = sideDistances[1] - sideDistances[0]

        #clamping distance
        if(distanceGap > 10):
            distanceGap = 10
        elif(distanceGap < -10):
            distanceGap = -10

        #distance calulations
        """
        #print("Dist: " + str(distanceToTravel))
        prevDis = currentDistance
        currentDistance = (BP.get_motor_encoder(BP.PORT_B) + BP.get_motor_encoder(BP.PORT_C)) / 2
        stepDis = (currentDistance - prevDis) * math.cos(abs(currentAngle)) #distance moved in desired direction
        distanceToTravel -= stepDis
        """

        #cm/s calculations
        angleSpeed = currentAngle * angleFact #pos if leaning right
        distSpeed = distanceGap * sideFact #pos if straying right
        driveSpeed = travelSpeed / (1 + math.exp(-0.002 * (distanceToTravel - 600))) #logistic feedback control

        #motor speed calculation
        rightAngSpeed = ConvertSpeedToDps(driveSpeed - angleSpeed + distSpeed)
        leftAngSpeed = ConvertSpeedToDps(driveSpeed + angleSpeed - distSpeed)

        BP.set_motor_dps(BP.PORT_B, rightAngSpeed) #right
        BP.set_motor_dps(BP.PORT_C, leftAngSpeed) #left

        stepDis = ConvertSpeedToDps(driveSpeed) * math.cos(abs(currentAngle * math.pi / 180))
        #print("DS: " + str(driveSpeed))
        #print("ANG: " + str(currentAngle))
        #print("DTT: " + str(distanceToTravel) + " SD: " +str(stepDis))
        distanceToTravel -= stepDis

        time.sleep(0.1)

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
    kP = 0.2
    kL = 3
    angleThreshold = 0.1
    
    while not((currentAngle < desiredAngle + angleThreshold) and (currentAngle > desiredAngle - angleThreshold)):
        
        direction = 0
        
        if(desiredAngle - currentAngle > 0):
            direction = -1
        else:
            direction = 1
        
        angularSpeedLogistic = kL * ((90 - 10) / (1 + math.exp(-0.1 * (abs(desiredAngle - currentAngle) - 37))) + 10)
        angularSpeedPorp = kP * abs(desiredAngle - currentAngle)
        angularSpeedTotal = direction * (angularSpeedPorp + angularSpeedLogistic)

        #print(str(currentAngle) + " current Angle")
        
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
    RotateVisorToAngle(8)
    time.sleep(0.3)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Front")
    return(dis)

def ScanRear():
    RotateVisorToAngle(-222)
    time.sleep(0.3)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Rear")
    return(dis)

def ScanRight():
    RotateVisorToAngle(-310)
    time.sleep(0.3)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Right")
    return(dis)

def ScanLeft():
    RotateVisorToAngle(-125)
    time.sleep(0.3)
    dis = MeasureDistance()
    time.sleep(0.25)
    print("Scanned Left")
    return(dis)

def ReturnVisorToFrontView():
    RotateVisorToAngle(8)
    time.sleep(0.3)
    print("Returned view to front")

def ScanAllSides():
    distanceScans = [0, 0, 0, 0]
    distanceScans[0] = ScanFront()
    distanceScans[1] = ScanLeft()
    distanceScans[2] = ScanRear()
    distanceScans[3] = ScanRight()
    ReturnVisorToFrontView()
    UpdateMapBasedOnScans(distanceScans)

def MoveToAdjacentGrid(direction):
    
    global currentHeading
    print("Current Heading Before Turn: " + currentHeading)
    print("GEARS Current Grid Location Before Drive: " + str(gearsCurrentGridLocation))
    
    if(direction == "North"):
        if(not(currentHeading == "North")):
            TurnToAngle(0)
            time.sleep(0.05)
        currentHeading = "North"
        gearsCurrentGridLocation[1] += 1
        
    elif(direction == "East"):
        if(not(currentHeading == "East")):
            TurnToAngle(-270)
            time.sleep(0.05)
        currentHeading = "East"
        gearsCurrentGridLocation[0] += 1
        
    elif(direction == "South"):
        if(not(currentHeading == "South")):
            TurnToAngle(-180)
            time.sleep(0.05)
        currentHeading = "South"
        gearsCurrentGridLocation[1] += -1
        
    elif(direction == "West"):
        if(not(currentHeading == "West")):
            TurnToAngle(-90)
            time.sleep(0.05)
        currentHeading = "West"
        gearsCurrentGridLocation[0] += -1

    print("Current Heading After Turn: " + currentHeading)
    print("GEARS Current Grid Location After Drive: " + str(gearsCurrentGridLocation))
    time.sleep(0.05)
    DriveCorridor(lengthOfGridUnit)
    SetGridLocationValue(gearsCurrentGridLocation, 2)
    ScanAllSides()

def FollowPath(directions):
    for order in directions:
        MoveToAdjacentGrid(order)



def FindTotalDistanceToTile(start, curLoc, end):

    distanceStart = abs(start[0] - curLoc[0]) + abs(start[1] - curLoc[1])
    distanceEnd = abs(end[0] - curLoc[0]) + abs(end[1] - curLoc[1])
    distanceTotal = distanceStart + distanceEnd
    return(distanceTotal)

def CheckIfGridLocIsPassable(loc):
    try:
        if(loc == gearsCurrentGridLocation):
            temp = False
        elif(loc[0] < 0 or loc[1] < 0):
            temp = False
        elif(mapData[loc[0]][loc[1]] == 1):
            temp = True
        elif(mapData[loc[0]][loc[1]] == 2):
            temp = True
        elif(mapData[loc[0]][loc[1]] == 3):
            temp = True
        elif(mapData[loc[0]][loc[1]] == 6):
            temp = True
        elif(mapData[loc[0]][loc[1]] == 7):
            temp = True
        else:
            temp = False

        return(temp)

    except IndexError:
        return False
    

def TraceBackPath(mapDirections, destination, start):
    path = []

    curCheck = destination
    while curCheck != start:
        path.append(mapDirections[curCheck[0]][curCheck[1]])
        if(path[-1] == "North"):
            curCheck = [curCheck[0], curCheck[1] - 1]
        elif(path[-1] == "South"):
            curCheck = [curCheck[0], curCheck[1] + 1]
        elif(path[-1] == "West"):
            curCheck = [curCheck[0] + 1, curCheck[1]]
        elif(path[-1] == "East"):
            curCheck = [curCheck[0] - 1, curCheck[1]]

    path.reverse()
    return(path)

def AStarPathFinding(dest):

    foundPath = False
    path = [] #final path
    mapCosts = [] #costs of distance to each map location
    mapPathTo = [] #direction from which grid square was moved into
    checkedLocs = [] #Checked Locations

    #initialize Arrays for pathfinding
    for i in range(mapWidth):
        mapCosts.append([])
        for j in range(mapHeight):
            mapCosts[i].append(math.inf)

    for i in range(mapWidth):
        mapPathTo.append([])
        for j in range(mapHeight):
            mapPathTo[i].append(0)

    for i in range(mapWidth):
        checkedLocs.append([])
        for j in range(mapHeight):
            checkedLocs[i].append(False)

    #A* below
    checkLoc = [gearsCurrentGridLocation[0],gearsCurrentGridLocation[1]]
    
    while not(foundPath):

        #update Map
        #CheckAbove
        checkLoc[1] += 1
        if(CheckIfGridLocIsPassable(checkLoc) and checkedLocs[checkLoc[0]][checkLoc[1]] == False):
            mapCosts[checkLoc[0]][checkLoc[1]] = FindTotalDistanceToTile(gearsCurrentGridLocation, checkLoc, dest)
            mapPathTo[checkLoc[0]][checkLoc[1]] = "North"

        #CheckBelow
        checkLoc[1] += -2
        if(CheckIfGridLocIsPassable(checkLoc) and checkedLocs[checkLoc[0]][checkLoc[1]] == False):
            mapCosts[checkLoc[0]][checkLoc[1]] = FindTotalDistanceToTile(gearsCurrentGridLocation, checkLoc, dest)
            mapPathTo[checkLoc[0]][checkLoc[1]] = "South"

        #CheckLeft
        checkLoc[1] += 1
        checkLoc[0] += -1
        if(CheckIfGridLocIsPassable(checkLoc) and checkedLocs[checkLoc[0]][checkLoc[1]] == False):
            mapCosts[checkLoc[0]][checkLoc[1]] = FindTotalDistanceToTile(gearsCurrentGridLocation, checkLoc, dest)
            mapPathTo[checkLoc[0]][checkLoc[1]] = "West"

        #CheckRight
        checkLoc[0] += 2
        if(CheckIfGridLocIsPassable(checkLoc) and checkedLocs[checkLoc[0]][checkLoc[1]] == False):
            mapCosts[checkLoc[0]][checkLoc[1]] = FindTotalDistanceToTile(gearsCurrentGridLocation, checkLoc, dest)
            mapPathTo[checkLoc[0]][checkLoc[1]] = "East"

        #find smallest cost
        minCost = math.inf
        minCostLoc = [-1,-1]
        for i in range(mapWidth):
            for j in range(mapHeight):
                if(mapCosts[i][j] < minCost and checkedLocs[i][j] == False):
                    minCost = mapCosts[i][j]
                    minCostLoc = [i, j]

        checkedLocs[minCostLoc[0]][minCostLoc[1]] = True
        checkLoc = [minCostLoc[0],minCostLoc[1]]
        
        if(minCostLoc == dest):
            foundPath = True
            path = TraceBackPath(mapPathTo, dest, gearsCurrentGridLocation)

    return(path)

def FindLocationToPathTo(travelToExit):

    minLocation = [-1,-1]
    minDistance = math.inf

    numberOfLocationsToTravel = 0

    for i in range(mapWidth):
        for j in range(mapHeight):
            if(travelToExit):
                if(mapData[i][j] == 7):
                    numberOfLocationsToTravel += 1
                    dist = abs(gearsCurrentGridLocation[0] - i) + abs(gearsCurrentGridLocation[1] - j)

                    if(dist < minDistance):
                        minDistance = dist
                        minLocation = [i, j]

            else:    
                if(mapData[i][j] == 1 or mapData[i][j] == 6):
                    numberOfLocationsToTravel += 1
                    dist = abs(gearsCurrentGridLocation[0] - i) + abs(gearsCurrentGridLocation[1] - j)

                    if(dist < minDistance):
                        minDistance = dist
                        minLocation = [i, j]

    if(numberOfLocationsToTravel == 0): #traveled whole map
        return True

    #print("min loc: " + str(minLocation))
    path = AStarPathFinding(minLocation)

    FollowPath(path)
    return False

def CalibrateUltrasonicMotor():
    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #reset positions
    totalDisplacementAngle = 0
    displacementAngle = 0
    print("###Calibration###")
    print("Enter 'done' when calibration is finished")

    while displacementAngle != "done":
        
        displacementAngle = input("Enter angle to displace Ultrasonic Motor: ")

        try:
            displacementAngle = int(displacementAngle)
            totalDisplacementAngle += displacementAngle
            BP.set_motor_position(BP.PORT_D, totalDisplacementAngle)
            
        except ValueError:
            displacementAngle = "done"

    BP.offset_motor_encoder(BP.PORT_D, BP.get_motor_encoder(BP.PORT_D)) #reset positions

def StartMission():
    print("MISSION STARTED")

    ScanAllSides()

    while not(FindLocationToPathTo(False)):
        pass

    FindLocationToPathTo(True)

    WriteUtilityMapData()
    print("MAP TRAVERSED")
    ConvertWorkMapToFinalMap()
    WriteMapData()
    print("MAP WRITTEN")
    
def StartGEARS():
    choice = 0

    while (choice != 1 and choice != 2 and choice != 3):
        print("Starting GEARS Autonomous Vehicle")
        print("Enter 1 to Start Mission")
        print("Enter 2 to Calibrate")
        print("Enter 3 to Calibrate and Start Mission")
        choice = input("Enter Choice: ")

        try:
            choice = int(choice)
            
        except ValueError:
            choice = "Invalid Choice"

        print("") #for spacing

        if(choice == 1):
            StartMission()
        elif(choice == 2):
            CalibrateUltrasonicMotor()
        elif(choice == 3):
            CalibrateUltrasonicMotor()
            StartMission()
        else:
            print("Invalid Choice\n")
     
def Point2PointNav(points):
    #Demonstrate GEARS ability to perform point-to-point navigation in a grid
    global gearsCurrentGridLocation

    for point in points:
        x = point[0]
        y = point[1]


        dis = (y - gearsCurrentGridLocation[1]) * lengthOfGridUnit
        print(dis)
        if(dis >= 0):
            TurnToAngle(0)
        else:
            TurnToAngle(180)

        DriveStraightDistance(dis, dis / 5)

        dis = (x - gearsCurrentGridLocation[0]) * lengthOfGridUnit
        print(dis)
        if(dis >= 0):
            TurnToAngle(90)
        else:
            TurnToAngle(270)
            
        DriveStraightDistance(dis, dis / 5)

        gearsCurrentGridLocation = [x, y]
        
        time.sleep(1)

#______________________________________________________________________________



#TESTING AREA__________________________________________________________________

try:
    #while True:
        #print(MeasureIR())
        #time.sleep(0.05)
    
    #StartGEARS()
    DriveCorridor(40)
        
except KeyboardInterrupt:
    BP.set_motor_dps(BP.PORT_B, 0)
    BP.set_motor_dps(BP.PORT_C, 0)
    ReturnVisorToFrontView()
    WriteUtilityMapData()
    print("MAP WRITTEN")
    ConvertWorkMapToFinalMap()
    WriteMapData()
    BP.reset_all()
