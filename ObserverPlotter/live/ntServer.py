from networktables import NetworkTables
import time

NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")

ENABLED_FIELD = (1 << 0)
AUTO_FIELD = (1 << 1)

# Start logger
fms = NetworkTables.getTable('FMSInfo')
fms.putNumber('FMSControlData', ENABLED_FIELD | AUTO_FIELD)

val = 0.0
for i in range(200):

    data = [val, val, val, val + 30, val + 30, val + 30, 0, 0, 0, 0, 0, 0, 30, 30, 30]
    sd.putNumberArray("path_log_data", data)

    val += 0.3
    time.sleep(0.04)
    print i

# Stop logger
fms.putNumber('FMSControlData', 0)

time.sleep(10)

print "Done"