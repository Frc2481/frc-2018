import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation
import math
import sys

numArgs = len(sys.argv)
if(numArgs == 1):
    print "Provide a filename."
    sys.exit(1)
elif(numArgs > 1):
    filename = sys.argv[1]
else:
    #This should never happen since the first arg is the script name
    print "HUH?!"
    sys.exit(1)

robotWidth = 22.375
robotLength = 27.0

def drawField(plax):
    #Calculate field length and width in inches
    fieldDim = [12*27, 12*54]
    
    #Draw individual elements
    fieldBoundary = [(29.69, 0), (fieldDim[0]-29.69, 0), (fieldDim[0], 35.0), (fieldDim[0], fieldDim[1]-35.0), (fieldDim[0]-29.69, fieldDim[1]), (29.69, fieldDim[1]), (0, fieldDim[1]-35.0), (0, 35.0), (29.69, 0)]
    plax.plot([p[0] for p in fieldBoundary], [p[1] for p in fieldBoundary], color="black")
    
    switchClose = [(85.25, 140), (fieldDim[0]-85.25, 140), (fieldDim[0]-85.25, 196), (85.25, 196), (85.25, 140)]
    plax.plot([p[0] for p in switchClose], [p[1] for p in switchClose], color="black")
    switchFar = [(85.25, fieldDim[1]-140), (fieldDim[0]-85.25, fieldDim[1]-140), (fieldDim[0]-85.25, fieldDim[1]-196), (85.25, fieldDim[1]-196), (85.25, fieldDim[1]-140)]
    plax.plot([p[0] for p in switchFar], [p[1] for p in switchFar], color="black")
    
    scale = [(71.57, 299.65), (fieldDim[0]-71.57, 299.65), (fieldDim[0]-71.57, 348.35), (71.57, 348.35), (71.57, 299.65)]
    plax.plot([p[0] for p in scale], [p[1] for p in scale], color="black")
    
    platform = [(95.25, 261.47), (fieldDim[0]-95.25, 261.47), (fieldDim[0]-95.25, 386.53), (95.25, 386.53), (95.25, 261.47)]
    plax.plot([p[0] for p in platform], [p[1] for p in platform], color="black")
    
    nullZoneLeft = [(0.0, 288.0), (95.25, 288.0), (95.25, 288.0+72.0), (0, 288.0+72.0)]
    plax.plot([p[0] for p in nullZoneLeft], [p[1] for p in nullZoneLeft], linestyle=':', color="black")
    nullZoneRight = [(fieldDim[0], 288.0), (fieldDim[0]-95.25, 288.0), (fieldDim[0]-95.25, 288.0+72.0), (fieldDim[0], 288.0+72.0)]
    plax.plot([p[0] for p in nullZoneRight], [p[1] for p in nullZoneRight], linestyle=':', color="black")
    
    autoLineClose = [(0, 120.0), (fieldDim[0], 120.0)]
    plax.plot([p[0] for p in autoLineClose], [p[1] for p in autoLineClose], linestyle=':', color="black")
    autoLineFar = [(0, fieldDim[1]-120.0), (fieldDim[0], fieldDim[1]-120.0)]
    plax.plot([p[0] for p in autoLineFar], [p[1] for p in autoLineFar], linestyle=':', color="black")
    
    exchangeZoneClose = [(149.69, 0), (149.69, 36.0), (149.69-48.0, 36.0), (149.69-48.0, 0)]
    plax.plot([p[0] for p in exchangeZoneClose], [p[1] for p in exchangeZoneClose], linestyle=':', color="black")
    exchangeZoneFar = [(fieldDim[0]-149.69, fieldDim[1]), (fieldDim[0]-149.69, fieldDim[1]-36.0), (fieldDim[0]-149.69+48.0, fieldDim[1]-36.0), (fieldDim[0]-149.69+48.0, fieldDim[1])]
    plax.plot([p[0] for p in exchangeZoneFar], [p[1] for p in exchangeZoneFar], linestyle=':', color="black")
    
    powerCubeZoneClose = [(161.69-22.5, 140), (161.69-22.5, 140-42.0), (161.69+22.5, 140-42.0), (161.69+22.5, 140)]
    plax.plot([p[0] for p in powerCubeZoneClose], [p[1] for p in powerCubeZoneClose], linestyle=':', color="black")
    powerCubeZoneFar = [(161.69-22.5, fieldDim[1]-140), (161.69-22.5, fieldDim[1]-140+42.0), (161.69+22.5, fieldDim[1]-140+42.0), (161.69+22.5, fieldDim[1]-140)]
    plax.plot([p[0] for p in powerCubeZoneFar], [p[1] for p in powerCubeZoneFar], linestyle=':', color="black")
    
#Rotate a point p around a second point center by an angle (in degrees)
def rotatePoint(p, center, angle):
    s = math.sin(math.radians(angle))
    c = math.cos(math.radians(angle))
    
    px = p[0]- center[0]
    py = p[1] - center[1]
    pxn = px*c - py*s
    pyn = px*s + py*c
    
    px = pxn + center[0]
    py = pyn + center[1]
    return (px, py)
    
#Generate the array for matplotlib that shows the robot base, rotated by heading
def genRobotSquare(p, heading):
    topLeft = (p[0]-robotWidth/2.0, p[1]+robotLength/2.0)
    topRight = (p[0]+robotWidth/2.0, p[1]+robotLength/2.0)
    bottomLeft = (p[0]-robotWidth/2.0, p[1]-robotLength/2.0)
    bottomRight = (p[0]+robotWidth/2.0, p[1]-robotLength/2.0)
    
    topLeft = rotatePoint(topLeft, p, heading)
    topRight = rotatePoint(topRight, p, heading)
    bottomLeft = rotatePoint(bottomLeft, p, heading)
    bottomRight = rotatePoint(bottomRight, p, heading)
    
    boxArr = [topLeft, topRight, bottomRight, bottomLeft, topLeft]
    return boxArr
    
timestamps = []
timeDeltas = [40.0]
robotX = []
robotY = []
robotYaw = []
pathX = []
pathY = []
pathYaw = []
pathXVel = []
pathYVel = []
pathXAccel = []
pathYAccel = []
robotXVel = []
robotYVel = []
xError = []
yError = []
yawError = []

#Read the csv
f = open(filename, 'r')
linenum = 1
for line in f:
    if linenum == 1:
        #First line is header information.  I'd prefer this line have alliance color and switch/scale positions for coloring
        linenum += 1
        continue
    data = line[:-1].split(",")
        
    timestamps.append(int(float(data[0])))
    if(linenum > 2):
        timeDeltas.append((timestamps[-1]-timestamps[-2])/1000.0)
    robotX.append(float(data[1]))
    robotY.append(float(data[2]))
    robotYaw.append(float(data[3]))
    pathX.append(float(data[4]))
    pathY.append(float(data[5]))
    pathYaw.append(float(data[6]))
    pathXAccel.append(-float(data[7]))
    pathYAccel.append(float(data[8]))
    pathXVel.append(float(data[9]))
    pathYVel.append(float(data[10]))
    actualX = float(data[11])
    if(abs(actualX) < 0.001):
        if len(robotXVel) > 0:
            actualX = robotXVel[-1]
    actualY = float(data[12])
    if(abs(actualY) < 0.001):
        if len(robotYVel) > 0:
            actualY = robotYVel[-1]
    robotXVel.append(actualX)
    robotYVel.append(actualY)
    xError.append(float(data[13]))
    yError.append(float(data[14]))
    yawError.append(float(data[15]))
    
    linenum += 1
    
if len(robotX) == 0:
    print "No valid data in the files.  Either you provided an invalid input file or the file did not have all of the data columns."
    sys.exit(0)

#Generate the figure and draw static elements
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
drawField(ax)
ax.plot(robotX, robotY, color='red', alpha=0.25)
ax.plot(pathX, pathY, color='green', alpha=0.25)

#Draw the path and robot
point, = ax.plot(robotX[0], robotY[0], marker='o', markersize=5, color="red")
startingRobot = genRobotSquare((robotX[0], robotY[0]), 0.0)
robot, = ax.plot([p[0] for p in startingRobot], [p[1] for p in startingRobot], color="black")

fig2 = plt.figure()
ax2 = fig2.add_subplot(221)
ax2.axhline(0, color='black')
ax2MinY = min(min(robotYaw), min(pathYaw))
ax2MaxY = max(max(robotYaw), max(pathYaw))
ax2.set_title('Actual and Path Yaw')
ax2.plot(robotYaw, label='Actual Yaw')
ax2.plot(pathYaw, label='Path Yaw')
timeLine2, = ax2.plot([0, 0], [ax2MinY, ax2MaxY], color='black', alpha=0.25)
ax2.legend()
ax3 = fig2.add_subplot(222)
ax3.axhline(0, color='black')
ax3.set_title('Errors')
ax3.plot(xError, label='X Error')
ax3.plot(yError, label='Y Error')
ax3.plot(yawError, label='Yaw Error')
ax3MinY = min(min(xError), min(yError), min(yawError))
ax3MaxY = max(max(xError), max(yError), max(yawError))
timeLine3, = ax3.plot([0, 0], [ax3MinY, ax3MaxY], color='black', alpha=0.25)
ax3.legend()
ax4 = fig2.add_subplot(223)
ax4.axhline(0, color='black')
ax4.plot(robotXVel, label='Actual X Velocity')
ax4.plot(robotYVel, label='Actual Y Velocity')
ax4.plot(pathXVel, label='Path X Velocity')
ax4.plot(pathYVel, label='Path Y Velocity')
ax4MinY = min(min(robotXVel), min(robotYVel), min(pathXVel), min(pathYVel))
ax4MaxY = max(max(robotXVel), max(robotYVel), max(pathXVel), max(pathYVel))
timeLine4, = ax4.plot([0, 0], [ax4MinY, ax4MaxY], color='black', alpha=0.25)
ax4.legend()
ax5 = fig2.add_subplot(224)
ax5.axhline(0, color='black')
ax5.plot(timeDeltas, label='Time Deltas')
avgTimeDelta = sum(timeDeltas)/float(len(timeDeltas))
ax5.plot([0, len(timeDeltas)], [avgTimeDelta, avgTimeDelta], color='black', alpha=0.25)
ax5MinY = min(timeDeltas)
ax5MaxY = max(timeDeltas)
timeLine5, = ax5.plot([0, 0], [ax5MinY, ax5MaxY], color='black', alpha=0.25)
ax5.legend()

#Set margins so the edge of the field isn't right on the edge of the plot
ax.margins(x=0.1, y=0.1)

#The animation tick, update only what needs to change in the plot
def updateFig1(n):
    point.set_data(np.array([robotX[n], robotY[n]]))
    robotData = genRobotSquare((robotX[n], robotY[n]), robotYaw[n])
    robot.set_data([p[0] for p in robotData], [p[1] for p in robotData])
    
    return [point, robot]

def updateFig2(n):
    timeLine2.set_xdata([n, n])
    timeLine3.set_xdata([n, n])
    timeLine4.set_xdata([n, n])
    timeLine5.set_xdata([n, n])
    return [timeLine2, timeLine3, timeLine4, timeLine5]

#Animate
ani = animation.FuncAnimation(fig, updateFig1, frames=len(robotX), interval=100)
ani2 = animation.FuncAnimation(fig2, updateFig2, frames=len(robotX), interval=100)

plt.show()