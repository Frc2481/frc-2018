close all

%%%%%%%%%%% 2018 FRC Power Up %%%%%%%%%%%%
robotDim = [33.5, 39]; % including bumpers
fieldDim = 12 * [27, 54];
cubeDim = 13;
cubeGrabDist = 17;
cubeSpacing = 28.1;
scaleApproachDist = 246;
cubeApproachDistScale = 246;
platformAlleyScale = 238;
cubeApproachDistSwitch = 240;
maxSpeed = 90; % 90
maxAccel = 100; % 100
maxSpeed2 = 105; % 105
maxAccel2 = 20; % 20
maxDeccel = -140; % -140
sampleRate = 25; % 25

%%%%%%%%%%%% left start, left scale %%%%%%%%%%%%
leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
leftScaleWP1 = [leftStart(1), 190, 0, 40];
leftScale = [71.57 + 1, 288 - (robotDim(2) / 2) + 6, -15, 0];
leftScaleWP2 = [71.57 + 1, scaleApproachDist, -8, 30];
leftScaleWP3 = [71.57 + 1, scaleApproachDist, 0, 30];
leftCube1Approach = [85.25 + (cubeDim / 2), cubeApproachDistScale, 0, 5];
leftCube1 = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
leftCube2Approach = [85.25 + (cubeDim / 2) + cubeSpacing, cubeApproachDistScale, 0, 5];
leftCube2 = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
leftCube3Approach = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, cubeApproachDistScale + 5, 0, 5];
leftCube3 = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];

csvFilename = 'Path_LL_scale1.csv';
waypoints = [leftStart;
             leftScaleWP1;
             leftScale(1), leftScale(2) - 2, leftScale(3), leftScale(4)];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale2.csv';
waypoints = [leftScale(1), leftScale(2) + 2, leftScale(3), leftScale(4);
             leftScaleWP3;
             leftCube1Approach;
             leftCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, -100, sampleRate)

csvFilename = 'Path_LL_scale3.csv';
waypoints = [leftCube1;
             leftScaleWP2;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale4.csv';
waypoints = [leftScale;
             leftScaleWP3;
             leftCube2Approach;
             leftCube2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, -100, sampleRate)

csvFilename = 'Path_LL_scale5.csv';
waypoints = [leftCube2;
             leftScaleWP2(1), leftScaleWP2(2), -15, leftScaleWP2(4);
             leftScale(1), leftScale(2), -25, leftScale(4)];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale6.csv';
waypoints = [leftScale;
             leftScaleWP3;
             leftCube3Approach;
             leftCube3];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, 95, maxAccel2, -80, sampleRate)

csvFilename = 'Path_LL_scale7.csv';
waypoints = [leftCube3;
             leftScaleWP2(1), leftScaleWP2(2), -15, leftScaleWP2(4);
             leftScale(1), leftScale(2) + 2, -25, leftScale(4)];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

%%%%%%%%%%% left start, right scale %%%%%%%%%%%%
leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
rightScaleWP1 = [leftStart(1), platformAlleyScale, 0, 15];
rightScaleWP2 = [fieldDim(1) - 71.57 - 1, platformAlleyScale, 0, 60];
rightScale = [fieldDim(1) - 71.57 - 1, 288 - (robotDim(2) / 2) + 6, 15, 0];
rightScaleWP3 = [fieldDim(1) - 71.57 - 1, scaleApproachDist, 8, 30];
rightScaleWP4 = [fieldDim(1) - 71.57 - 1, scaleApproachDist, 0, 30];
rightCube1Approach = [fieldDim(1) - 85.25 - (cubeDim / 2), cubeApproachDistScale, 0, 5];
rightCube1 = [fieldDim(1) - 85.25 - (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
rightCube2Approach = [fieldDim(1) - 85.25 - (cubeDim / 2) - cubeSpacing, cubeApproachDistScale, 0, 5];
rightCube2 = [fieldDim(1) - 85.25 - (cubeDim / 2) - cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
rightCube3Approach = [fieldDim(1) - 85.25 - (cubeDim / 2) - 2 * cubeSpacing, cubeApproachDistScale + 5, 0, 5];
rightCube3 = [fieldDim(1) - 85.25 - (cubeDim / 2) - 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];

csvFilename = 'Path_LR_scale1.csv';
waypoints = [leftStart;
             rightScaleWP1;
             rightScaleWP2;
             rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LR_scale2.csv';
waypoints = [rightScale;
             rightScaleWP4;
             rightCube1Approach;
             rightCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, -100, sampleRate)

csvFilename = 'Path_LR_scale3.csv';
waypoints = [rightCube1;
             rightScaleWP3;
             rightScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LR_scale4.csv';
waypoints = [rightScale;
             rightScaleWP4;
             rightCube2Approach;
             rightCube2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, -100, sampleRate)

csvFilename = 'Path_LR_scale5.csv';
waypoints = [rightCube2;
             rightScaleWP3(1), rightScaleWP3(2), 15, rightScaleWP3(4);
             rightScale(1), rightScale(2), 25, rightScale(4)];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LR_scale6.csv';
waypoints = [rightScale;
             rightScaleWP4;
             rightCube3Approach;
             rightCube3];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, 95, maxAccel2, -80, sampleRate)

csvFilename = 'Path_LR_scale7.csv';
waypoints = [rightCube3;
             rightScaleWP3(1), rightScaleWP3(2), 15, rightScaleWP3(4);
             rightScale(1), rightScale(2) + 2, 25, rightScale(4)];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% %%%%%%%%%%% left start, left switch %%%%%%%%%%%%
% leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
% leftSwitchWP1 = [29.69 + (robotDim(1) / 2), 60, -90, 40];
% leftSwitchWP2 = [85.25 - (robotDim(2) / 2) - 8, 160, -90, 0];
% leftSwitchWP3 = [85.25 - (robotDim(2) / 2) - 8, cubeApproachDistSwitch, 0, 40];
% leftCube1Approach = [85.25 + (cubeDim / 2), cubeApproachDistSwitch, 0, 5];
% leftCube1 = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftSwitch = [85.25 + 30, 196 + (robotDim(2) / 2), 0, 0];
% leftCube2Approach = [85.25 + (cubeDim / 2) + cubeSpacing, cubeApproachDistSwitch, 0, 5];
% leftCube2 = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftCube3Approach = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, cubeApproachDistSwitch, 0, 5];
% leftCube3 = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_LL_switch1.csv';
% waypoints = [leftStart;
%              leftSwitchWP1;
%              leftSwitchWP2;
%              leftSwitchWP3;
%              leftCube2Approach;
%              leftCube2];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LL_switch2.csv';
% waypoints = [leftCube2;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LL_switch3.csv';
% waypoints = [leftSwitch;
%              leftCube1Approach;
%              leftCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LL_switch4.csv';
% waypoints = [leftCube1;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LL_switch5.csv';
% waypoints = [leftSwitch;
%              leftCube3Approach;
%              leftCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LL_switch6.csv';
% waypoints = [leftCube3;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% %%%%%%%%%%% left start, right switch %%%%%%%%%%%%
% leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
% rightSwitchWP1 = [leftStart(1), cubeApproachDistSwitch, 0, 20];
% rightSwitch = [fieldDim(1) - 85.25 - 30, 196 + (robotDim(2) / 2) + 15, 0, 0];
% leftCube2Approach = [85.25 + (cubeDim / 2) + cubeSpacing, cubeApproachDistSwitch, 0, 5];
% leftCube2 = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftCube3Approach = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, cubeApproachDistSwitch, 0, 5];
% leftCube3 = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftCube4Approach = [85.25 + (cubeDim / 2) + 3 * cubeSpacing, cubeApproachDistSwitch, 0, 5];
% leftCube4 = [85.25 + (cubeDim / 2) + 3 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_LR_switch1.csv';
% waypoints = [leftStart;
%              rightSwitchWP1;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch2.csv';
% waypoints = [rightSwitch;
%              leftCube4Approach;
%              leftCube4];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch3.csv';
% waypoints = [leftCube4;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch4.csv';
% waypoints = [rightSwitch;
%              leftCube3Approach;
%              leftCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch5.csv';
% waypoints = [leftCube3;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch5.csv';
% waypoints = [rightSwitch;
%              leftCube2Approach;
%              leftCube2];s
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_switch6.csv';
% waypoints = [leftCube2;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)