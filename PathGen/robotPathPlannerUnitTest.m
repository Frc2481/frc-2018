% paths for unit testing

csvFilename = 'robotPath1.csv';
waypoints = [0, 0, 0, 0;
             25, 50, 45, 0;
             75, 25, 120, 30;
             150, 100, 45, 50;
             200, 120, -30, 10000;
             75, 150, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.9;
sampleRate = 10;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

csvFilename = 'robotPath2.csv';
waypoints = [0, 0, 0, 0;
             25, 50, 45, 0;
             75, 25, 120, 30;
             150, 100, 45, 50;
             200, 120, -30, 10000;
             75, 150, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.1;
sampleRate = 10;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

csvFilename = 'robotPath3.csv';
waypoints = [0, 0, 0, 0;
             25, 50, 0, 0;
             75, 25, 0, 30;
             150, 100, 0, 50;
             200, 120, 0, 10000;
             75, 150, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

csvFilename = 'robotPath4.csv';
waypoints = [0, 0, 0, 0;
             25, 50, 45, 0;
             75, 25, 120, 30;
             150, 100, 45, 50;
             200, 120, -30, 10000;
             75, 150, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.9;
sampleRate = 0.1;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

csvFilename = 'robotPath5.csv';
waypoints = [0, 0, 0, 0;
             50, 0, 0, 10;
             100, 0, 0, 100;
             150, 0, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.9;
sampleRate = 10;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

csvFilename = 'robotPath6.csv';
waypoints = [0, 0, 0, 0;
             50, 0, 0, 10;
             100, 0, 0, 100;
             50, 0, 0, 10;
             150, 0, 0, 0];
maxSpeed = 100;
maxAccel = maxSpeed * 0.9;
sampleRate = 10;
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
