% % paths for unit testing
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

maxSpeed = 100 * 0.95 * 0.80; % 65 in/s low gear, 100 in/s high gear, 95% for allowing controller to catch up when lagging, 80% to compensate for arm current draw
maxAccel = maxSpeed;
sampleRate = 50;
robotDimensionsWithBumpers = [33.5, 39];
fieldDimensions = 12 * [27, 54];

% % test robot path
% csvFilename = 'robotPath.csv';
% waypoints = [0, 0, 0, 0;
%              0, 100, 90, 0];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to left scale
csvFilename = 'PathLeftStartToLeftScale.csv';
waypoints = [46.4, 19.5, 0, 0;
             46.4, 180, 0, 40;
             77, 280, -10, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to left switch
csvFilename = 'PathLeftStartToLeftSwitch.csv';
waypoints = [46.4, 19.5, 0, 0;
             46.4, 100, -45, 40;
             61, 168, -90, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left start to right scale
csvFilename = 'PathLeftStartToRightScale.csv';
waypoints = [46.4, 19.5, 0, 0;
             46.4, 235, 0, 10;
             fieldDimensions(1) - 77, 235, 0, 5;
             fieldDimensions(1) - 77, 280, 10, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale back up
csvFilename = 'PathLeftScaleBackUp.csv';
waypoints = [77, 280, -10, 0;
             77, 270, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to left cube 1
csvFilename = 'PathLeftScaleToLeftCube1.csv';
waypoints = [77, 270, 0, 0;
             77, 245, 0, 5;
             98, 245, 0, 5;
             98, 240, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 1 to left switch
csvFilename = 'PathLeftCube1ToLeftSwitch.csv';
waypoints = [98, 240, 0, 0;
             103, 216, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to right cube 1
csvFilename = 'PathLeftScaleToRightCube1.csv';
waypoints = [77, 270, 0, 0;
             77, 245, 0, 2;
             fieldDimensions(1) - 88, 250, 0, 2;
             fieldDimensions(1) - 88, 245, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right cube 1 to right switch 2
csvFilename = 'PathRightCube1ToRightSwitch2.csv';
waypoints = [fieldDimensions(1) - 88, 245, 0, 0;
             fieldDimensions(1) - 93, 221, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right start to right scale
csvFilename = 'PathRightStartToRightScale.csv';
waypoints = [fieldDimensions(1) - 46.4, 19.5, 0, 0;
             fieldDimensions(1) - 46.4, 180, 0, 40;
             fieldDimensions(1) - 77, 280, 10, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right start to right switch
csvFilename = 'PathRightStartToRightSwitch.csv';
waypoints = [fieldDimensions(1) - 46.4, 19.5, 0, 0;
             fieldDimensions(1) - 46.4, 100, 45, 40;
             fieldDimensions(1) - 61, 168, 90, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right start to left scale
csvFilename = 'PathRightStartToLeftScale.csv';
waypoints = [fieldDimensions(1) - 46.4, 19.5, 0, 0;
             fieldDimensions(1) - 46.4, 235, 0, 10;
             77, 235, 0, 5;
             77, 280, -10, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale back up
csvFilename = 'PathRightScaleBackUp.csv';
waypoints = [fieldDimensions(1) - 77, 280, 10, 0;
             fieldDimensions(1) - 77, 270, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to right cube 1
csvFilename = 'PathRightScaleToRightCube1.csv';
waypoints = [fieldDimensions(1) - 77, 270, 0, 0;
             fieldDimensions(1) - 77, 245, 0, 5;
             fieldDimensions(1) - 98, 245, 0, 5;
             fieldDimensions(1) - 98, 240, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right cube 1 to right switch
csvFilename = 'PathRightCube1ToRightSwitch.csv';
waypoints = [fieldDimensions(1) - 98, 240, 0, 0;
             fieldDimensions(1) - 103, 216, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to left cube 1
csvFilename = 'PathRightScaleToLeftCube1.csv';
waypoints = [fieldDimensions(1) - 77, 270, 0, 0;
             fieldDimensions(1) - 77, 245, 0, 2;
             88, 250, 0, 2;
             88, 245, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
      
% left cube 1 to left switch 2
csvFilename = 'PathLeftCube1ToLeftSwitch2.csv';
waypoints = [88, 245, 0, 0;
             93, 221, 0, 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

