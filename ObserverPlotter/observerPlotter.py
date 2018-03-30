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
    filename = sys.argv[1] #"ObserverLog_23.csv"
    if(numArgs > 2):
        outputMode = sys.argv[2][0]
    else:
        outputMode = "p"
else:
    #This should never happen since the first arg is the script name
    print "HUH?!"
    sys.exit(1)

#Because the ObserverLogs on github had negative (and off-field) values, I took a couple paths and added time as the first column to test the field drawing
#filename = "PathLeftStartToLeftCube4.csv"

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
    
def genArmRect(pivot, extension):
    #Purely cosmetic, converts units in inches to pixels
    armScaleFactor = 3.5
    
    extension *= armScaleFactor
    
    #I don't have arm dimensions at all.  I'm just guessing.  Replace with proper values at some point.
    #Numbers are in inches
    outerArmLength = 45.0 * armScaleFactor
    outerArmWidth = 3.0 * armScaleFactor
    innerArmLength = 40.0 * armScaleFactor
    innerArmWidth = 2.0 * armScaleFactor
    
    #Outer Arm
    bottomLeft = (500, 0)
    topLeft = (500, outerArmLength)
    topRight = (500+outerArmWidth, outerArmLength)
    bottomRight = (500+outerArmWidth, 0)
    
    cp = (500+(outerArmWidth/2.0), outerArmLength/2.0) #Pivot point on the arm.  Middle of arm for now (I think this is correct?).  Find where on the arm and update y if needed
    
    bottomLeft = rotatePoint(bottomLeft, cp, pivot)
    topLeft = rotatePoint(topLeft, cp, pivot)
    topRight = rotatePoint(topRight, cp, pivot)
    bottomRight = rotatePoint(bottomRight, cp, pivot)
    
    outerArmArr = [bottomLeft, topLeft, topRight, bottomRight, bottomLeft]
    
    #Inner arm
    xoff = (outerArmWidth-innerArmWidth)/2.0
    yoff = outerArmLength-innerArmLength
    bottomLeft = (500+xoff, yoff+extension)
    topLeft = (500+xoff, yoff+innerArmLength+extension)
    topRight = (500+xoff+innerArmWidth, yoff+innerArmLength+extension)
    bottomRight = (500+xoff+innerArmWidth, yoff+extension)
    
    bottomLeft = rotatePoint(bottomLeft, cp, pivot)
    topLeft = rotatePoint(topLeft, cp, pivot)
    topRight = rotatePoint(topRight, cp, pivot)
    bottomRight = rotatePoint(bottomRight, cp, pivot)
    
    innerArmArr = [bottomLeft, topLeft, topRight, bottomRight, bottomLeft]
    
    #Arm base (triangular structure)
    temp = robotLength/2.0*armScaleFactor
    pivotHeight = 20 * armScaleFactor #Height of pivot (as measured from chassis).  Guesstimate again, find proper value and update
    p0 = (cp[0]-temp, cp[1]-pivotHeight)
    p2 = (cp[0]+temp, cp[1]-pivotHeight)
    armBaseArr = [p0, cp, p2]
    
    return outerArmArr, innerArmArr, armBaseArr
    
def genChassis(robotHeading, flAngle, frAngle, blAngle, brAngle):
    #Purely cosmetic, converts units in inches to pixels (may be different number than arm scale factor, but same idea)
    chassisScaleFactor = 4.5
    
    #The width/length we have are for the wheels, not chassis.  I think we were a half inch short of max size?
    chassisWidth = 27.5 * chassisScaleFactor
    chassisLength = 32.5 * chassisScaleFactor
    
    robotCP = (500, 450)
    bottomLeft = (robotCP[0] - chassisWidth/2.0, robotCP[1] - chassisLength/2.0)
    topLeft = (robotCP[0] - chassisWidth/2.0, robotCP[1] + chassisLength/2.0)
    topRight = (robotCP[0] + chassisWidth/2.0, robotCP[1] + chassisLength/2.0)
    bottomRight = (robotCP[0] + chassisWidth/2.0, robotCP[1] - chassisLength/2.0)
    
    bottomLeft = rotatePoint(bottomLeft, robotCP, robotHeading)
    topLeft = rotatePoint(topLeft, robotCP, robotHeading)
    topRight = rotatePoint(topRight, robotCP, robotHeading)
    bottomRight = rotatePoint(bottomRight, robotCP, robotHeading)
    
    chassisArr = [bottomLeft, topLeft, topRight, bottomRight, bottomLeft]
    
    rwHalf = (robotWidth*chassisScaleFactor)/2.0
    rlHalf = (robotLength*chassisScaleFactor)/2.0
    flArr = genWheel(robotCP[0]-rwHalf, robotCP[1]+rlHalf, chassisScaleFactor, flAngle, robotCP, robotHeading)
    frArr = genWheel(robotCP[0]+rwHalf, robotCP[1]+rlHalf, chassisScaleFactor, frAngle, robotCP, robotHeading)
    blArr = genWheel(robotCP[0]-rwHalf, robotCP[1]-rlHalf, chassisScaleFactor, blAngle, robotCP, robotHeading)
    brArr = genWheel(robotCP[0]+rwHalf, robotCP[1]-rlHalf, chassisScaleFactor, brAngle, robotCP, robotHeading)
    
    fmOffset = rotatePoint((0, 10), (0, 0), robotHeading)
    forwardMarkerArr = [(topLeft[0]+fmOffset[0], topLeft[1]+fmOffset[1]), (topRight[0]+fmOffset[0], topRight[1]+fmOffset[1])]
    
    return chassisArr, flArr, frArr, blArr, brArr, forwardMarkerArr
    
def genWheel(x, y, sf, angle, robotCenter, robotHeading):
    wheelWidth = 1.0 * sf
    wheelLength = 3.0 * sf
    
    bottomLeft = (x-wheelWidth/2.0, y-wheelLength/2.0)
    topLeft = (x-wheelWidth/2.0, y+wheelLength/2.0)
    topRight = (x+wheelWidth/2.0, y+wheelLength/2.0)
    bottomRight = (x+wheelWidth/2.0, y-wheelLength/2.0)
    
    bottomLeft = rotatePoint(bottomLeft, (x,y), angle)
    topLeft = rotatePoint(topLeft, (x,y), angle)
    topRight = rotatePoint(topRight, (x,y), angle)
    bottomRight = rotatePoint(bottomRight, (x,y), angle)
    
    bottomLeft = rotatePoint(bottomLeft, robotCenter, robotHeading)
    topLeft = rotatePoint(topLeft, robotCenter, robotHeading)
    topRight = rotatePoint(topRight, robotCenter, robotHeading)
    bottomRight = rotatePoint(bottomRight, robotCenter, robotHeading)
    
    wheelArr = [bottomLeft, topLeft, topRight, bottomRight, bottomLeft]
    return wheelArr

startTime = 0
ts = []

xdata = []
ydata = []
headings = []

pivots = []
extensions = []

flSteers = []
frSteers = []
blSteers = []
brSteers = []

#Read the csv
f = open(filename, 'r')
linenum = 0
for line in f:
    if linenum == 0:
        #First line is header information.  I'd prefer this line have alliance color and switch/scale positions for coloring
        linenum += 1
        continue
    data = line[:-1].split(",")
    if len(data) < 17:
        continue
    if (len(str(data[0])) == 0) or (len(str(data[1])) == 0) or (len(str(data[2])) == 0) or (len(str(data[3])) == 0):
        continue
    ts.append(int(data[0]))
    xdata.append(float(data[1]))
    ydata.append(float(data[2]))
    headings.append(float(data[3]))
    
    pivots.append(-float(data[16]))
    extensions.append(float(data[17]))
    
    flSteers.append(-float(data[4]))
    frSteers.append(-float(data[5]))
    blSteers.append(-float(data[6]))
    brSteers.append(-float(data[7]))
    
    linenum += 1
    
if len(ts) == 0:
    print "No valid data in the files.  Either you provided an invalid input file or the file did not have all of the data columns."
    sys.exit(0)
    
#Calculate the average time step between lines to we can calculate a frame rate
gaps = []
for v in range(len(ts)):
    if v == 0:
        continue
    gaps.append(int(ts[v]) - int(ts[v-1]))
avgIntervalMs = int(round(sum(gaps)/float(len(gaps))/1000.0))
startTime = ts[0]

#Generate the figure and draw static elements
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
drawField(ax)
ax.plot(xdata, ydata, color='black', alpha=0.25)

#Draw the path and robot
point, = ax.plot(xdata[0], ydata[0], marker='o', markersize=5, color="red")
startingRobot = genRobotSquare((xdata[0], ydata[0]), 0.0)
robot, = ax.plot([p[0] for p in startingRobot], [p[1] for p in startingRobot], color="black")

startingOuterArm, startingInnerArm, startingArmBase = genArmRect(0, 0)
outerArm, = ax.plot([p[0] for p in startingOuterArm], [p[1] for p in startingOuterArm], color="black")
innerArm, = ax.plot([p[0] for p in startingInnerArm], [p[1] for p in startingInnerArm], color="black")
armbase, = ax.plot([p[0] for p in startingArmBase], [p[1] for p in startingArmBase], color="black", alpha=0.5)
#This arrow indicates which way is forward, so we know whether the arm is on the front or back of the robot
armDir = ax.arrow(450, -25, 100, 0, length_includes_head=True, head_width=25, fc='k', ec='k')

startingChassis, startingFL, startingFR, startingBL, startingBR, startingForwardMarker = genChassis(0, 0, 0, 0, 0)
chassis, = ax.plot([p[0] for p in startingChassis], [p[1] for p in startingChassis], color="black")
flWheel, = ax.plot([p[0] for p in startingFL], [p[1] for p in startingFL], color="black")
frWheel, = ax.plot([p[0] for p in startingFR], [p[1] for p in startingFR], color="black")
blWheel, = ax.plot([p[0] for p in startingBL], [p[1] for p in startingBL], color="black")
brWheel, = ax.plot([p[0] for p in startingBR], [p[1] for p in startingBR], color="black")
forwardMarker, = ax.plot([p[0] for p in startingForwardMarker], [p[1] for p in startingForwardMarker], color="red")
wheelAngleSet = [flSteers, frSteers, blSteers, brSteers]
wheelDrawingSet = [flWheel, frWheel, blWheel, brWheel]

time = ax.text(6*27, 12*54+25, "Time: 0.0 sec", ha="center", va="center")

#Set margins so the edge of the field isn't right on the edge of the plot
plt.margins(x=0.1, y=0.1)
xmin, xmax = plt.xlim()
xmax = max(650, xmax)
plt.xlim(xmin, xmax)

#The animation tick, update only what needs to change in the plot
def updatePoint(n):
    point.set_data(np.array([xdata[n], ydata[n]]))
    robotData = genRobotSquare((xdata[n], ydata[n]), headings[n])
    robot.set_data([p[0] for p in robotData], [p[1] for p in robotData])
    
    outerArmData, innerArmData, armBaseData = genArmRect(pivots[n], extensions[n])
    outerArm.set_data([p[0] for p in outerArmData], [p[1] for p in outerArmData])
    innerArm.set_data([p[0] for p in innerArmData], [p[1] for p in innerArmData])
    
    chassisData, flData, frData, blData, brData, fmData = genChassis(headings[n], flSteers[n], frSteers[n], blSteers[n], brSteers[n])
    chassis.set_data([p[0] for p in chassisData], [p[1] for p in chassisData])
    flWheel.set_data([p[0] for p in flData], [p[1] for p in flData])
    frWheel.set_data([p[0] for p in frData], [p[1] for p in frData])
    blWheel.set_data([p[0] for p in blData], [p[1] for p in blData])
    brWheel.set_data([p[0] for p in brData], [p[1] for p in brData])
    forwardMarker.set_data([p[0] for p in fmData], [p[1] for p in fmData])
    
    time.set_text("Time: " + "{0:.2f}".format((ts[n]-ts[0])/1000000.0) + "sec")
    return [point, robot, outerArm, innerArm, chassis, flWheel, frWheel, blWheel, brWheel, forwardMarker, time]

#Animate
ani = animation.FuncAnimation(fig, updatePoint, frames=len(xdata), interval=avgIntervalMs)

#plt.show will quickly show it on a matplotlib window, but is not guaranteed to be realtime (dependent on host machine specs)
#ani.save will produce an mp4 that takes longer to generate but will have proper time scaling
if(outputMode == "p"):
    plt.show()
else:
    ani.save(filename[:-3]+'mp4', writer="ffmpeg")