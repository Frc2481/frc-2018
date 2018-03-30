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

% test robot path
maxSpeed = 100 * 0.95; % 65 in/s low gear, 100 in/s high gear, 95% for allowing controller to catch up when lagging
maxAccel = 90;
sampleRate = 20;
csvFilename = 'robotPath.csv';
waypoints = [0, 0, 0, 0;
             0, 150, 0 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 2018 FRC PowerUp
robotDim = [33.5, 39]; % including bumpers
fieldDim = 12 * [27, 54];
cubeDim = 13;

%%%%%%%%%%%% left start, left switch %%%%%%%%%%%%
LL_leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
LL_leftSwitchSide = [85.3 - (robotDim(2) / 2) - 8, 168, -90, 0];

maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftSwitch.csv';
waypoints = [LL_leftStart;
             LL_leftStart(1), 60, -90, 100;
             LL_leftSwitchSide];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%% left start, left scale, left cube 1, left switch, left cube 2, left scale %%%%%%%%%%%%
LLL_leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
LLL_leftScale = [71.6 + 9, 298 - (robotDim(2) / 2) - 7, -10, 0];
LLL_leftCube1 = [85.3 + (cubeDim / 2) + 9, 196 + (cubeDim / 2) + (robotDim(2) / 2) + 10, 0, 0];
LLL_leftSwitchBack = [LLL_leftCube1(1) + 2, LLL_leftCube1(2) - 20, 0, 0];
LLL_leftCube2 = [LLL_leftCube1(1) + 28, LLL_leftCube1(2), 0, 0];
LLL_leftScale2 = [LLL_leftScale(1), LLL_leftScale(2) + 9, -40, 0];

% left start to left scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftScale.csv';
waypoints = [LLL_leftStart;
             LLL_leftStart(1), 190, 0, 40;
             LLL_leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to left cube 1
maxSpeed = 100 * 0.95 * 0.9;
maxAccel = maxSpeed * 1.0;
csvFilename = 'PathLeftScaleToLeftCube1.csv';
waypoints = [LLL_leftScale;
             LLL_leftScale(1), LLL_leftCube1(2) + 10, 0, 2;
             LLL_leftCube1(1), LLL_leftCube1(2) + 10, 0, 2;
             LLL_leftCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 1 to switch
maxSpeed = 100 * 0.95 * 10.0;
maxAccel = maxSpeed * 10.0;
sampleRate = 50;
csvFilename = 'PathLeftCube1ToSwitch.csv';
waypoints = [LLL_leftCube1;
             LLL_leftSwitchBack];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% switch to left cube 2
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathSwitchToLeftCube2.csv';
waypoints = [LLL_leftSwitchBack;
             LLL_leftSwitchBack(1) - 10, LLL_leftCube2(2) + 25, 0, 2;
             LLL_leftCube2(1) + 5, LLL_leftCube2(2) + 25, 0, 2;
             LLL_leftCube2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 2 to left scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftCube2ToLeftScale.csv';
waypoints = [LLL_leftCube2;
             LLL_leftScale2(1) - 5, LLL_leftCube2(2) + 5, 0, 5;
             LLL_leftScale2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%% left start, right scale, left cube 6, right switch, left cube 5, right scale %%%%%%%%%%%%
LRR_leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
LRR_rightScale = [fieldDim(1) - 71.6 - 5, 298 - (robotDim(2) / 2) - 4, 10, 0];
LRR_leftCube6 = [fieldDim(1) - 85.3 - (cubeDim / 2) - 3, 196 + (cubeDim / 2) + (robotDim(2) / 2) + 11, 0, 0];
LRR_rightSwitchBack = [LRR_leftCube6(1) - 2, LRR_leftCube6(2) - 20, 0, 0];
LRR_leftCube5 = [LRR_leftCube6(1) - 27, LRR_leftCube6(2), 0, 0];
LRR_rightScale2 = [LRR_rightScale(1) + 4, LRR_rightScale(2) + 9, 40, 0];

% left start to right scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToRightScale.csv';
waypoints = [LRR_leftStart;
             LRR_leftStart(1), 245, 0, 15;
             LRR_rightScale(1) + 15, 225, 0, 2;
             LRR_rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to left cube 6
maxSpeed = 100 * 0.95 * 0.8;
maxAccel = maxSpeed * 1.0;
csvFilename = 'PathRightScaleToLeftCube6.csv';
waypoints = [LRR_rightScale;
             LRR_rightScale(1), LRR_leftCube6(2) + 10, 0, 2;
             LRR_leftCube6(1), LRR_leftCube6(2) + 10, 0, 2;
             LRR_leftCube6];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 6 to right switch
maxSpeed = 100 * 0.95 * 10.0;
maxAccel = maxSpeed * 10.0;
sampleRate = 50;
csvFilename = 'PathLeftCube6ToSwitch2.csv';
waypoints = [LRR_leftCube6;
             LRR_rightSwitchBack];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right switch to left cube 5
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathSwitchToLeftCube52.csv';
waypoints = [LRR_rightSwitchBack
             LRR_rightSwitchBack(1) + 10, LRR_leftCube5(2) + 20, 0, 2;
             LRR_leftCube5(1) - 5, LRR_leftCube5(2) + 20, 0, 2;
             LRR_leftCube5];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 5 to right scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftCube5ToRightScale.csv';
waypoints = [LRR_leftCube5;
             LRR_rightScale2(1) + 5, LRR_leftCube5(2) + 5, 0, 5;
             LRR_rightScale2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%% left start, left scale, left cube 6, right switch, left cube 5, left scale %%%%%%%%%%%%
LRL_leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
LRL_leftScale = [71.6 + 9, 298 - (robotDim(2) / 2) - 7, -10, 0];
LRL_leftCube6 = [fieldDim(1) - 85.3 - (cubeDim / 2) + 6, 196 + (cubeDim / 2) + (robotDim(2) / 2) + 11, 0, 0];
LRL_rightSwitchBack = [LRL_leftCube6(1) - 2, LRL_leftCube6(2) - 20, 0, 0];
LRL_leftCube5 = [LRL_leftCube6(1) - 27, LRL_leftCube6(2) - 1, 0, 0];
LRL_leftScale2 = [LRL_leftScale(1), LRL_leftScale(2) + 9, -40, 0];

% left start to left scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftScale.csv';
waypoints = [LRL_leftStart;
             LRL_leftStart(1), 190, 0, 40;
             LRL_leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left scale to left cube 6
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftScaleToLeftCube6.csv';
waypoints = [LRL_leftScale;
             LRL_leftScale(1) - 10, LRL_leftCube6(2) + 7, 0, 2;
             LRL_leftCube6(1) + 15, LRL_leftCube6(2) + 28, 0, 2;
             LRL_leftCube6];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 6 to right switch
maxSpeed = 100 * 0.95 * 10.0;
maxAccel = maxSpeed * 10.0;
sampleRate = 50;
csvFilename = 'PathLeftCube6ToSwitch.csv';
waypoints = [LRL_leftCube6;
             LRL_rightSwitchBack];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right switch to left cube 5
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathSwitchToLeftCube5.csv';
waypoints = [LRL_rightSwitchBack
             LRL_rightSwitchBack(1) + 10, LRL_leftCube5(2) + 25, 0, 2;
             LRL_leftCube5(1) - 5, LRL_leftCube5(2) + 25, 0, 2;
             LRL_leftCube5];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 5 to left scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftCube5ToLeftScale.csv';
waypoints = [LRL_leftCube5;
             LRL_leftCube5(1), LRL_leftCube5(2) + 10, 0, 2;
             LRL_leftScale2(1), LRL_leftCube5(2) + 10, 0, 2;
             LRL_leftScale2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

%%%%%%%%%%%% left start, left switch, left cube 4, right scale, left cube 6, right scale %%%%%%%%%%%%
LLR_leftStart = [29.7 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
LLR_leftSwitchSide = [85.3 - (robotDim(2) / 2) - 8, 175, -90, 0];
LLR_leftCube4 = [183, 196 + (cubeDim / 2) + (robotDim(2) / 2) + 11, 0, 0];
LLR_rightScale = [fieldDim(1) - 71.6 - 6, 298 - (robotDim(2) / 2) - 6, 10, 0];
LLR_leftCube6 = [LLR_leftCube4(1) + 50, LLR_leftCube4(2) + 2, 0, 0];
LLR_rightScale2 = [LLR_rightScale(1) + 5, LLR_rightScale(2) + 9, 40, 0];

% leftCube62 = [fieldDim(1) - leftCube1(1) + 5, leftCube1(2) + 1, 0, 0];
% leftCube1 = [85.3 + 6.5 + 8, 196 + 6.5 + (robotDim(2) / 2) + 10, 0, 0];

% left start to left cube 4
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftStartToLeftCube4.csv';
waypoints = [LLR_leftStart;
             LLR_leftSwitchSide(1), 60, -90, 100;
             LLR_leftSwitchSide(1), LLR_leftSwitchSide(2), LLR_leftSwitchSide(3), 0;
             LLR_leftSwitchSide(1) - 20, LLR_leftCube4(2) + 15, -60, 10;
             LLR_leftCube4(1) + 5, LLR_leftCube4(2) + 10, LLR_leftCube4(3), 2;
             LLR_leftCube4];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 4 to right scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftCube4ToRightScale.csv';
waypoints = [LLR_leftCube4;
             LLR_leftCube4(1), LLR_leftCube4(2) + 10, 0, 2;
             LLR_rightScale(1) + 35, LLR_leftCube4(2) - 5, 0, 2;
             LLR_rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% right scale to left cube 6
maxSpeed = 100 * 0.95 * 0.7;
maxAccel = maxSpeed * 0.8;
sampleRate = 50;
csvFilename = 'PathRightScaleToLeftCube62.csv';
waypoints = [LLR_rightScale;
             LLR_rightScale(1), LLR_leftCube6(2) + 10, 0, 2;
             LLR_leftCube6(1), LLR_leftCube6(2) + 5, 0, 2;
             LLR_leftCube6];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)

% left cube 6 to right scale
maxSpeed = 100 * 0.95 * 1.0;
maxAccel = maxSpeed * 0.9;
sampleRate = 50;
csvFilename = 'PathLeftCube6ToRightScale.csv';
waypoints = [LLR_leftCube6;
             LLR_leftCube6(1), LLR_leftCube6(2) + 10, 0, 2;
             LLR_rightScale2(1), LLR_leftCube6(2) + 10, 0, 2;
             LLR_rightScale2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)