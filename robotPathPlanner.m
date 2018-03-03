% % paths for unit testing
%
% csvFilename = 'robotPath1.csv';
% waypoints = [0, 0, 0, 0;
%              25, 50, 45, 0;
%              75, 25, 120, 30;
%              150, 100, 45, 50;
%              200, 120, -30, 10000;
%              75, 150, 0, 0];
% maxSpeed = (105 - 11) * 0.9;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 10;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% csvFilename = 'robotPath2.csv';
% waypoints = [0, 0, 0, 0;
%              25, 50, 45, 0;
%              75, 25, 120, 30;
%              150, 100, 45, 50;
%              200, 120, -30, 10000;
%              75, 150, 0, 0];
% maxSpeed = (105 - 11) * 0.9;
% maxAccel = maxSpeed * 0.1;
% sampleRate = 10;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% csvFilename = 'robotPath3.csv';
% waypoints = [0, 0, 0, 0;
%              25, 50, 0, 0;
%              75, 25, 0, 30;
%              150, 100, 0, 50;
%              200, 120, 0, 10000;
%              75, 150, 0, 0];
% maxSpeed = (105 - 11) * 50;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% csvFilename = 'robotPath4.csv';
% waypoints = [0, 0, 0, 0;
%              25, 50, 45, 0;
%              75, 25, 120, 30;
%              150, 100, 45, 50;
%              200, 120, -30, 10000;
%              75, 150, 0, 0];
% maxSpeed = (105 - 11) * 0.9;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 0.1;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% csvFilename = 'robotPath5.csv';
% waypoints = [0, 0, 0, 0;
%              50, 0, 0, 10;
%              100, 0, 0, 100;
%              150, 0, 0, 0];
% maxSpeed = (105 - 11) * 0.9;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 10;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% csvFilename = 'robotPath6.csv';
% waypoints = [0, 0, 0, 0;
%              50, 0, 0, 10;
%              100, 0, 0, 100;
%              50, 0, 0, 10;
%              150, 0, 0, 0];
% maxSpeed = (105 - 11) * 0.9;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 10;
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % test robot path
% maxSpeed = 100 * 0.95 * 1.0; % 65 in/s low gear, 100 in/s high gear, 95% for allowing controller to catch up when lagging
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% csvFilename = 'robotPath.csv';
% waypoints = [0, 0, 0, 0;
%              0, 100, 90, 0];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2018 robot and field configuration
robotDim = [33.5, 39]; % including bumpers
fieldDim = 12 * [27, 54];

leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
leftScale = [71.6 + 9, 298 - (robotDim(2) / 2) - 7, -10, 0];
leftSwitchSide = [85.3 - (robotDim(2) / 2) - 7, 168, -90, 0];

laneBtwnSwitchScale = [0, 235, 0, 0];
rightScale = [fieldDim(1) - leftScale(1) + 2, leftScale(2), -leftScale(3), leftScale(4)];

leftCube1 = [85.3 + 6.5 + 6, 196 + 6.5 + (robotDim(2) / 2) + 8, 0, 0];
leftSwitchBack1 = [leftCube1(1), leftCube1(2) - 20, 0, 0];

leftCube4 = [180, leftCube1(2), 0, 0];

leftCube6 = [fieldDim(1) - leftCube1(1) + 14, leftCube1(2) + 1, -leftCube1(3), leftCube1(4)];
leftSwitchBack6 = [leftCube6(1), leftCube6(2) - 20, 0, 0];

leftCube62 = [fieldDim(1) - leftCube1(1) + 2, leftCube1(2) + 1, -leftCube1(3), leftCube1(4)];
leftSwitchBack62 = [leftCube62(1), leftCube62(2) - 20, 0, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2018 auto paths

% left start to left scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftScale.csv';
waypoints = [leftStart;
             leftStart(1), 190, 0, 40;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to left switch
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftSwitch.csv';
waypoints = [leftStart;
             leftStart(1), 100, -45, 100;
             leftSwitchSide];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left switch to left cube 4
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftSwitchToLeftCube4.csv';
waypoints = [leftSwitchSide;
             leftSwitchSide(1) - 25, leftCube4(2) + 15, -60, 10;
             leftCube4(1) + 5, leftCube4(2) + 20, leftCube4(3), 2;
             leftCube4];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left switch to left cube 4
% maxSpeed = 100 * 0.95 * 1.0;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% csvFilename = 'PathLeftCube4ToRightScale.csv';
% waypoints = [leftCube4;
%              leftCube4(1), leftCube4(2) + 10, 0, 2;
%              rightScale(1) + 15, leftCube4(2) + 5, 0, 10;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left scale to left cube 1
% maxSpeed = 100 * 0.95 * 1.0;
% maxAccel = maxSpeed * 1.5;
% csvFilename = 'PathLeftScaleToLeftCube1.csv';
% waypoints = [leftScale;
%              leftScale(1), leftCube1(2) + 10, 0, 2;
%              leftCube1(1), leftCube1(2) + 10, 0, 2;
%              leftCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left cube 1 to switch
% maxSpeed = 100 * 0.95 * 10.0;
% maxAccel = maxSpeed * 10.0;
% sampleRate = 50;
% csvFilename = 'PathLeftCube1ToSwitch.csv';
% waypoints = [leftCube1;
%              leftSwitchBack1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left scale to left cube 6
% maxSpeed = 100 * 0.95 * 1.0;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% csvFilename = 'PathLeftScaleToLeftCube6.csv';
% waypoints = [leftScale;
%              leftScale(1) - 10, leftCube6(2) + 10, 0, 2;
%              leftCube6(1) + 10, leftCube6(2) + 20, 0, 2;
%              leftCube6];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left cube 6 to switch
% maxSpeed = 100 * 0.95 * 10.0;
% maxAccel = maxSpeed * 10.0;
% sampleRate = 50;
% csvFilename = 'PathLeftCube6ToSwitch.csv';
% waypoints = [leftCube6;
%              leftSwitchBack6];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left start to right scale
% maxSpeed = 100 * 0.95 * 0.9;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% csvFilename = 'PathLeftStartToRightScale.csv';
% waypoints = [leftStart;
%              leftStart(1), laneBtwnSwitchScale(2) + 10, 0, 15;
%              rightScale(1), laneBtwnSwitchScale(2) - 10, 0, 10;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % right scale to left cube 6
% maxSpeed = 100 * 0.95 * 1.0;
% maxAccel = maxSpeed * 1.5;
% csvFilename = 'PathRightScaleToLeftCube6.csv';
% waypoints = [rightScale;
%              rightScale(1), leftCube62(2) + 10, 0, 2;
%              leftCube62(1), leftCube62(2) + 10, 0, 2;
%              leftCube62];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % left cube 6 to switch 2
% maxSpeed = 100 * 0.95 * 10.0;
% maxAccel = maxSpeed * 10.0;
% sampleRate = 50;
% csvFilename = 'PathLeftCube6ToSwitch2.csv';
% waypoints = [leftCube62;
%              leftSwitchBack62];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
% % right paths
% maxSpeed = 100 * 0.95 * 1.0;
% maxAccel = maxSpeed * 0.9;
% sampleRate = 50;
% waypoints = [0, 0, 0, 0;
%              0, 100, 90, 0];
% csvFilename = 'PathRightStartToRightScale.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightStartToRightSwitch.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightScaleToRightCube1.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightCube1ToSwitch.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightScaleToRightCube6.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightCube6ToSwitch.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightStartToLeftScale.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathLeftScaleToRightCube6.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightSwitchToRightCube4.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% csvFilename = 'PathRightCube4ToLeftScale.csv';
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% 
