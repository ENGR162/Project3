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

WriteMapData()
#mapData = [[1]]
#mapData[0].append(1)
#print(mapData)