#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
myVariable = 0
#Directions
NORTH = 0
EAST = 90
SOUTH = 180
WEST = 270
#Maze size
mazeWidth = 8
mazeHeight = 8
cellSize = 250
gridOriginX = -900
gridOriginY = -900
#Map walls
UNKNOWN = -1
WALL    = 0
OPEN    = 1
# Distance threshold tweak 
WALL_THRESHOLD_MM = 140


def when_started1():
    
    global myVariable

    #Map Maze 
    stepCount = 0
    passageMarks = createPassageMarks(mazeWidth, mazeHeight)
    discoveredGraph = createDiscoveredGraph(mazeWidth, mazeHeight)
    sealMazeBordersAsWalls(discoveredGraph)

    brain.print("Graph size:")
    brain.print(len(discoveredGraph))
    brain.print(" x ")
    brain.print(len(discoveredGraph[0]))
    brain.new_line()


    (startCellX, startCellY) = getCurrentCell()

    goalCellX = 3
    goalCellY = 7

    #Start Pen
    pen.set_pen_color(BLUE)
    pen.move(DOWN)
    #Store Previous Direction
    previousDirection = None

    while True :
       # wait(1, SECONDS)

        (currentCellX, currentCellY) = getCurrentCell()

        if currentCellX == goalCellX and currentCellY == goalCellY:
            drivetrain.stop()
            pen.move(UP)
            brainPrintLine("Exit reached. Finding shortest path...")

            break

        openDirections = getCandidateDirections(currentCellX, currentCellY)
        allowedDirections = []


        for direction in openDirections:
            index = directionToIndex(direction)
            if discoveredGraph[currentCellX][currentCellY][index] == WALL:
                continue
            if passageMarks[currentCellX][currentCellY][index] < 2:
                allowedDirections.append(direction)
        
        brainPrintLine(allowedDirections)
        unmarkedDirections = []
        markedDirections = []

        for direction in allowedDirections:
            index = directionToIndex(direction)

            if passageMarks[currentCellX][currentCellY][index] == 0:
                unmarkedDirections.append(direction)

            elif passageMarks[currentCellX][currentCellY][index] == 1:
                markedDirections.append(direction)

        #
        # Decision Logic
        #

        if len(unmarkedDirections) > 0:

            chosenDirection =  unmarkedDirections[random.randint(0, len(unmarkedDirections)-1)]

            if chosenDirection == previousDirection and len(unmarkedDirections) > 1:
                chosenDirection =  unmarkedDirections[random.randint(0, len(unmarkedDirections)-1)]
        else:
            for direction in markedDirections:
                if previousDirection == direction:
                    chosenDirection = previousDirection
                else:
                    chosenDirection =  markedDirections[random.randint(0, len(markedDirections)-1)]

        #
        # Move and Mark Passage
        #

        oldCellX = currentCellX
        oldCellY = currentCellY

        moved, newCellX, newCellY = moveOneCellAndMap(discoveredGraph, oldCellX, oldCellY, chosenDirection)

        stepCount += 1
        printMazeMap(discoveredGraph, mazeWidth, mazeHeight,
                    oldCellX if not moved else newCellX,
                    oldCellY if not moved else newCellY)

        if not moved:
            directionIndex = directionToIndex(chosenDirection)
            passageMarks[oldCellX][oldCellY][directionIndex] = 2
            continue

        if (newCellX, newCellY) == (oldCellX, oldCellY):
            directionIndex = directionToIndex(chosenDirection)
            passageMarks[oldCellX][oldCellY][directionIndex] = 2
            continue

        index1 = directionToIndex(chosenDirection)
        passageMarks[oldCellX][oldCellY][index1] += 1

        oppositeDirection = getOppositeDirection(chosenDirection)

        index2 = directionToIndex(oppositeDirection)
        passageMarks[newCellX][newCellY][index2] += 1

        previousDirection = oppositeDirection

        brainPrintLine(passageMarks)
    
vr_thread(when_started1)

def createPassageMarks(mazeWidth, mazeHeight):
    passageMarks = []

    for x in range(mazeWidth):
        column = []
        for y in range(mazeHeight):
            column.append([0, 0, 0, 0])   # NORTH, EAST, SOUTH, WEST
        passageMarks.append(column)

    return passageMarks

def getCurrentCell():

    positionX = location.position(X, MM)
    positionY = location.position(Y, MM)

    # Convert position to cell index by shifting to cell centers
    cellX = int((positionX - gridOriginX + cellSize/2) // cellSize)
    cellY = int((positionY - gridOriginY + cellSize/2) // cellSize)

    # Clamp cell to whole number
    currentCellX = max(0, min(mazeWidth - 1, cellX))
    currentCellY = max(0, min(mazeHeight - 1, cellY))

    brain.print("Cell: ")
    brain.print(currentCellX)
    brain.print(", ")
    brain.print(currentCellY)
    brain.new_line()
    brain.print("Position:")
    brain.print(positionX)
    brain.print(", ")
    brain.print(positionY)
    brain.new_line()


    return (currentCellX, currentCellY)

def getNextCell(currentCellX, currentCellY, direction):
    if direction == NORTH:
        return (currentCellX, currentCellY + 1)
    if direction == SOUTH:
        return (currentCellX, currentCellY - 1)
    if direction == EAST:
        return (currentCellX + 1, currentCellY)
    if direction == WEST:
        return (currentCellX - 1, currentCellY)

def isInBounds(cellX, cellY):
    return (0 <= cellX < mazeWidth) and (0 <= cellY < mazeHeight)

def getDirection():
    direction = location.position_angle(DEGREES)
    
    brainPrintLine(direction)

    return (direction)

def faceDirection(direction):
    if direction == NORTH:
        drivetrain.turn_to_heading(NORTH, DEGREES)
    if direction == EAST:
        drivetrain.turn_to_heading(EAST, DEGREES)
    if direction == SOUTH:
        drivetrain.turn_to_heading(SOUTH, DEGREES)
    if direction == WEST:
        drivetrain.turn_to_heading(WEST, DEGREES)

def getOppositeDirection(direction):
    if direction == NORTH:
        return SOUTH

    if direction == SOUTH:
        return NORTH

    if direction == EAST:
        return WEST

    if direction == WEST:
        return EAST

def isDirectionOpen(direction):
    faceDirection(direction)

    wait(0.1, SECONDS)
    return front_distance.get_distance(MM) > 140

def getOpenDirections(currentCellX, currentCellY):
    originalHeading = location.position_angle(DEGREES)

    openDirections = []
    for direction in [NORTH, EAST, SOUTH, WEST]:
        nextCellX, nextCellY = getNextCell(currentCellX, currentCellY, direction)

        if not isInBounds(nextCellX, nextCellY):
            continue
        if isDirectionOpen(direction):
            openDirections.append(direction)


    drivetrain.turn_to_heading(originalHeading, DEGREES)
    wait(0.05, SECONDS)

    return openDirections

def getCandidateDirections(cellX, cellY):
    directions = []
    for direction in [NORTH, EAST, SOUTH, WEST]:
        nextCellX, nextCellY = getNextCell(cellX, cellY, direction)
        if isInBounds(nextCellX, nextCellY):
            directions.append(direction)
    return directions

def moveOneCell(direction):
    faceDirection(direction)
    wait(0.05, SECONDS)

    drivetrain.drive_for(FORWARD, cellSize, MM)

    wait(0.05, SECONDS)

def moveOneCellAndMap(edgeState, cellX, cellY, direction):

    nextCellX, nextCellY = getNextCell(cellX, cellY, direction)

    # Block move if next cell is outside grid
    if not isInBounds(nextCellX, nextCellY):
        setEdgeState(edgeState, cellX, cellY, direction, WALL)
        return False, cellX, cellY

    # Turn to face chosen direction
    faceDirection(direction)
    wait(0.05, SECONDS)

    # Check for physical wall
    dist = front_distance.get_distance(MM)
    if dist <= WALL_THRESHOLD_MM:
        setEdgeState(edgeState, cellX, cellY, direction, WALL)
        return False, cellX, cellY

    drivetrain.drive_for(FORWARD, cellSize, MM)

    newCellX, newCellY = getCurrentCell()

    if (newCellX, newCellY) == (cellX, cellY):
        setEdgeState(edgeState, cellX, cellY, direction, WALL)
        return False, cellX, cellY

    setEdgeState(edgeState, cellX, cellY, direction, OPEN)

    return True, newCellX, newCellY


def directionToIndex(direction):
    if direction == NORTH: return 0
    if direction == EAST:  return 1
    if direction == SOUTH: return 2
    if direction == WEST:  return 3

def createDiscoveredGraph(mazeWidth, mazeHeight):
    edgeState = []
    for x in range(mazeWidth):
        col = []
        for y in range(mazeHeight):
            col.append([UNKNOWN, UNKNOWN, UNKNOWN, UNKNOWN])  # N,E,S,W
        edgeState.append(col)
    return edgeState

def followPath(pathDirections):
    for direction in pathDirections:
        moveOneCell(direction)


def reversePath(pathDirections):
    reversedDirections = []
    for direction in reversed(pathDirections):
        reversedDirections.append(getOppositeDirection(direction))
    return reversedDirections

def isOpenEdge(discoveredGraph, cellX, cellY, direction):
    if not isInBounds(cellX, cellY):
        return False

    index = directionToIndex(direction)

    if discoveredGraph[cellX][cellY][index] != OPEN:
        return False

    nextCellX, nextCellY = getNextCell(cellX, cellY, direction)
    if not isInBounds(nextCellX, nextCellY):
        return False

    opposite = getOppositeDirection(direction)
    oppIndex = directionToIndex(opposite)

    return discoveredGraph[nextCellX][nextCellY][oppIndex] == OPEN


def setEdgeState(edgeState, cellX, cellY, direction, state):
    cellX = int(cellX); cellY = int(cellY)

    width = len(edgeState)
    if width == 0:
        return
    height = len(edgeState[0])

    if not (0 <= cellX < width and 0 <= cellY < height):
        return
    if len(edgeState[cellX][cellY]) < 4:
        return

    nextCellX, nextCellY = getNextCell(cellX, cellY, direction)
    nextCellX = int(nextCellX); nextCellY = int(nextCellY)

    if not (0 <= nextCellX < width and 0 <= nextCellY < height):
        state = WALL
        idx = directionToIndex(direction)
        edgeState[cellX][cellY][idx] = state
        return

    # Set current edge
    idx = directionToIndex(direction)
    edgeState[cellX][cellY][idx] = state

    # Set neighbour opposite edge
    opposite = getOppositeDirection(direction)
    oppIdx = directionToIndex(opposite)
    edgeState[nextCellX][nextCellY][oppIdx] = state


def brainPrintLine(s):
    brain.print(s)
    brain.new_line()

def edgeToH(state):
    # horizontal wall segment
    if state == WALL:    return "---"
    if state == OPEN:    return "   "
    return " ? "          # UNKNOWN

def edgeToV(state):
    # vertical wall segment
    if state == WALL:    return "|"
    if state == OPEN:    return " "
    return "?"            # UNKNOWN

def printMazeMap(discoveredGraph, width, height, cellX=None, cellY=None):
    brainPrintLine("")
    brainPrintLine("==== DISCOVERED MAZE MAP ====")

    for y in reversed(range(height)):

        # North edges row
        top_row = ""
        for x in range(width):
            top_row += "+"
            top_row += edgeToH(discoveredGraph[x][y][0])
        top_row += "+"
        brainPrintLine(top_row)

        # Middle row with West/East edges 
        mid_row = ""
        for x in range(width):
            mid_row += edgeToV(discoveredGraph[x][y][3])

            if cellX == x and cellY == y:
                mid_row += " R "
            else:
                mid_row += "   "

        # Last east edge
        mid_row += edgeToV(discoveredGraph[width-1][y][1])
        brainPrintLine(mid_row)

    # Bottom border 
    bottom_row = ""
    for x in range(width):
        bottom_row += "+"
        bottom_row += edgeToH(discoveredGraph[x][0][2])
    bottom_row += "+"
    brainPrintLine(bottom_row)

    brainPrintLine("=============================")
    brainPrintLine("")

def sealMazeBordersAsWalls(discoveredGraph):
    # Bottom + top borders
    for x in range(mazeWidth):
        setEdgeState(discoveredGraph, x, 0, SOUTH, WALL)
        setEdgeState(discoveredGraph, x, mazeHeight-1, NORTH, WALL)

    # Left + right borders
    for y in range(mazeHeight):
        setEdgeState(discoveredGraph, 0, y, WEST, WALL)
        setEdgeState(discoveredGraph, mazeWidth-1, y, EAST, WALL)

def onMazeFloor():
    c = down_eye.color()
    return (c == Color.WHITE) or (c == Color.GREEN) or (c == Color.RED)
