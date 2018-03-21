from networktables import NetworkTables
import time

NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")

val = 0.0
for i in range(1000):
    sd.putNumber("Robot Pose X", val)
    sd.putNumber("Robot Pose Y", val)
    sd.putNumber("Robot Pose Theta", val)
    val += 0.3
    time.sleep(0.04)
    print i

print "Done"