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

% 2018 robot and field configuration
maxSpeed = 100 * 0.95 * 0.80; % 65 in/s low gear, 100 in/s high gear, 95% for allowing controller to catch up when lagging, 80% to compensate for arm current draw
maxAccel = maxSpeed;
sampleRate = 50;

robotDim = [33.5, 39]; % including bumpers
fieldDim = 12 * [27, 54];

leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
leftScale = [71.6 + 7, 298 - (robotDim(2) / 2), -10, 0];
leftSwitchSide = [85.3 - 4 - (robotDim(2) / 2), 168, -90, 0];
leftSwitchBack = [85.3 + 1 + (robotDim(1) / 2), 196 + 1 + (robotDim(2) / 2), 0, 0];
leftCube1 = [85.3 + 6.5, 196 + 12 + 6.5 + (robotDim(2) / 2), 0, 0];
leftCube2 = leftCube1 + [28.1, 0, 0, 0];
leftCube3 = leftCube2 + [28.1, 0, 0, 0];

rightStart = [fieldDim(1) - leftStart(1), leftStart(2), -leftStart(3), leftStart(4)];
rightScale = [fieldDim(1) - leftScale(1), leftScale(2), -leftScale(3), leftScale(4)];
rightSwitchSide = [fieldDim(1) - leftSwitchSide(1), leftSwitchSide(2), -leftSwitchSide(3), leftSwitchSide(4)];
rightSwitchBack = [fieldDim(1) - leftSwitchBack(1), leftSwitchBack(2), -leftSwitchBack(3), leftSwitchBack(4)];
rightCube1 = [fieldDim(1) - leftCube1(1), leftCube1(2), -leftCube1(3), leftCube1(4)];
rightCube2 = [fieldDim(1) - leftCube2(1), leftCube2(2), -leftCube2(3), leftCube2(4)];
rightCube3 = [fieldDim(1) - leftCube3(1), leftCube3(2), -leftCube3(3), leftCube3(4)];

laneBtwnSwitchScale = [0, 240, 0, 0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2018 auto paths

% % test robot path
% csvFilename = 'robotPath.csv';
% waypoints = [0, 0, 0, 0;
%              0, 100, 90, 0];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to left scale
csvFilename = 'PathLeftStartToLeftScale.csv';
waypoints = [leftStart;
             leftStart(1), 180, 0, 40;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to left switch
csvFilename = 'PathLeftStartToLeftSwitch.csv';
waypoints = [leftStart;
             leftStart(1), 100, -45, 100;
             leftSwitchSide];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to right scale
csvFilename = 'PathLeftStartToRightScale.csv';
waypoints = [leftStart;
             leftStart(1), laneBtwnSwitchScale(2), 0, 20;
             rightScale(1), laneBtwnSwitchScale(2), 0, 5;
             rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to left cube 1
csvFilename = 'PathLeftScaleToLeftCube1.csv';
waypoints = [leftScale;
             leftScale(1), laneBtwnSwitchScale(2), 0, 5;
             leftCube1(1), laneBtwnSwitchScale(2), 0, 5;
             leftCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 1 to left switch
csvFilename = 'PathLeftCube1ToLeftSwitch.csv';
waypoints = [leftCube1;
             leftSwitchBack];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to right cube 1
csvFilename = 'PathLeftScaleToRightCube1.csv';
waypoints = [leftScale;
             leftScale(1), laneBtwnSwitchScale(2), 0, 2;
             rightCube1(1), laneBtwnSwitchScale(2), 0, 2;
             rightCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% right start to right scale
csvFilename = 'PathRightStartToRightScale.csv';
waypoints = [rightStart;
             rightStart(1), 180, 0, 40;
             rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right start to right switch
csvFilename = 'PathRightStartToRightSwitch.csv';
waypoints = [rightStart;
             rightStart(1), 100, 45, 100;
             rightSwitchSide];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right start to left scale
csvFilename = 'PathRightStartToLeftScale.csv';
waypoints = [rightStart;
             rightStart(1), laneBtwnSwitchScale(2), 0, 20;
             leftScale(1), laneBtwnSwitchScale(2), 0, 5;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to right cube 1
csvFilename = 'PathRightScaleToRightCube1.csv';
waypoints = [rightScale;
             rightScale(1), laneBtwnSwitchScale(2), 0, 5;
             rightCube1(1), laneBtwnSwitchScale(2), 0, 5;
             rightCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right cube 1 to right switch
csvFilename = 'PathRightCube1ToRightSwitch.csv';
waypoints = [rightCube1;
             rightSwitchBack];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to left cube 1
csvFilename = 'PathRightScaleToLeftCube1.csv';
waypoints = [rightScale;
             rightScale(1), laneBtwnSwitchScale(2), 0, 2;
             leftCube1(1), laneBtwnSwitchScale(2), 0, 2;
             leftCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
