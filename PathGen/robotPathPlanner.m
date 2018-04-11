%%%%%%%%%%%% test path %%%%%%%%%%%%
maxSpeed = 90;
maxAccel = 100;
maxSpeed2 = 110;
maxAccel2 = 20;
maxDeccel = -200;
sampleRate = 50;
csvFilename = 'robotPath.csv';
waypoints = [0, 0, 0, 0;
             0, 100, 0 0];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

%%%%%%%%%%%% 2018 FRC Power Up %%%%%%%%%%%%
robotDim = [33.5, 39]; % including bumpers
fieldDim = 12 * [27, 54];
cubeDim = 13;
cubeGrabDist = 10;
cubeSpacing = 28.1;
sampleRate = 50;
cubeApproachDist = 10;

%%%%%%%%%%%% left start, left scale %%%%%%%%%%%%
leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
leftScaleWP1 = [leftStart(1), 190, 0, 40];
leftScale = [71.57 + 4, 292 - (robotDim(2) / 2) - 6, -20, 0];
leftScaleWP2 = [71.57 + 4, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, -10, 20];
leftCube1Approach = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
leftCube1 = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
leftCube2Approach = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
leftCube2 = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
leftCube3Approach = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
leftCube3 = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];

csvFilename = 'Path_LL_scale1.csv';
waypoints = [leftStart;
             leftScaleWP1;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale2.csv';
waypoints = [leftScale;
             leftScaleWP2;
             leftCube1Approach;
             leftCube1];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale3.csv';
waypoints = [leftCube1;
             leftScaleWP2;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale4.csv';
waypoints = [leftScale;
             leftScaleWP2;
             leftCube2Approach;
             leftCube2];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale5.csv';
waypoints = [leftCube2;
             leftScaleWP2;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale6.csv';
waypoints = [leftScale;
             leftScaleWP2;
             leftCube3Approach;
             leftCube3];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

csvFilename = 'Path_LL_scale7.csv';
waypoints = [leftCube3;
             leftScaleWP2;
             leftScale];
generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)

% %%%%%%%%%%%% left start, right scale %%%%%%%%%%%%
% leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
% rightScaleWP1 = [leftStart(1), 240, 0, 20];
% rightScaleWP2 = [fieldDim(1) - 71.57 - 4, 240, 0, 20];
% rightScale = [fieldDim(1) - 71.57 - 4, 292 - (robotDim(2) / 2) - 6, 20, 0];
% rightScaleWP3 = [fieldDim(1) - 71.57 - 4, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 10, 20];
% rightCube1Approach = [fieldDim(1) - 85.25 - (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% rightCube1 = [fieldDim(1) - 85.25 - (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% rightCube2Approach = [fieldDim(1) - 85.25 - (cubeDim / 2) - cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% rightCube2 = [fieldDim(1) - 85.25 - (cubeDim / 2) - cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% rightCube3Approach = [fieldDim(1) - 85.25 - (cubeDim / 2) - 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% rightCube3 = [fieldDim(1) - 85.25 - (cubeDim / 2) - 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_LR_scale1.csv';
% waypoints = [leftStart;
%              rightScaleWP1;
%              rightScaleWP2;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_scale2.csv';
% waypoints = [rightScale;
%              rightScaleWP3;
%              rightCube1Approach;
%              rightCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_scale3.csv';
% waypoints = [rightCube1;
%              rightScaleWP3;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_scale4.csv';
% waypoints = [rightScale;
%              rightScaleWP3;
%              rightCube2Approach;
%              rightCube2];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate)
% 
% csvFilename = 'Path_LR_scale5.csv';
% waypoints = [rightCube2;
%              rightScaleWP3;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LR_scale6.csv';
% waypoints = [rightScale;
%              rightScaleWP3;
%              rightCube3Approach;
%              rightCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LR_scale7.csv';
% waypoints = [rightCube3;
%              rightScaleWP3;
%              rightScale];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% %%%%%%%%%%%% left start, left switch %%%%%%%%%%%%
% leftStart = [29.69 + (robotDim(1) / 2), (robotDim(2) / 2), 0, 0];
% leftSwitchWP1 = [29.69 + (robotDim(1) / 2), 60, -90, 40];
% leftSwitchWP2 = [85.25 - (robotDim(2) / 2) - 8, 168, -90, 0];
% leftSwitchWP3 = [85.25 - (robotDim(2) / 2) - 8, 240, 0, 40];
% leftCube1Approach = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% leftCube1 = [85.25 + (cubeDim / 2), 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftSwitch = [85.25 + 10, 196 + (robotDim(2) / 2), 0, 0];
% leftU1 = [85.25 + 20, 240, 0, 20];
% leftCube2Approach = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% leftCube2 = [85.25 + (cubeDim / 2) + cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% leftU2 = [85.25 + 50, 240, 0, 20];
% leftCube3Approach = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist + cubeApproachDist, 0, 10];
% leftCube3 = [85.25 + (cubeDim / 2) + 2 * cubeSpacing, 196 + (cubeDim / 2) + (robotDim(2) / 2) + cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_LL_switch1.csv';
% waypoints = [leftStart;
%              leftSwitchWP1;
%              leftSwitchWP2;
%              leftSwitchWP3;
%              leftCube1Approach;
%              leftCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_switch2.csv';
% waypoints = [leftCube1;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_switch3.csv';
% waypoints = [leftSwitch;
%              leftU1;
%              leftCube2Approach;
%              leftCube2];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_switch4.csv';
% waypoints = [leftCube2;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_switch5.csv';
% waypoints = [leftSwitch;
%              leftU2;
%              leftCube3Approach;
%              leftCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_switch6.csv';
% waypoints = [leftCube3;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% %%%%%%%%%%%% center start, left switch %%%%%%%%%%%%
% centerStart = [(fieldDim(1) / 2), (robotDim(2) / 2), 0, 0];
% leftSwitch = [85.25 + 10, 140 - (robotDim(2) / 2), 0, 0];
% centerCube1Approach = [(fieldDim(1) / 2), 100 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube1 = [(fieldDim(1) / 2), 100 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% centerCube2Approach = [(fieldDim(1) / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube2 = [(fieldDim(1) / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% centerCube3Approach = [(fieldDim(1) / 2) - (cubeDim / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube3 = [(fieldDim(1) / 2) - (cubeDim / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_CL_switch1.csv';
% waypoints = [centerStart;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch2.csv';
% waypoints = [leftSwitch;
%              centerCube1Approach;
%              centerCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch3.csv';
% waypoints = [centerCube1;
%              centerCube1Approach;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch4.csv';
% waypoints = [leftSwitch;
%              centerCube2Approach;
%              centerCube2];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch5.csv';
% waypoints = [centerCube2;
%              centerCube2Approach;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch6.csv';
% waypoints = [leftSwitch;
%              centerCube3Approach;
%              centerCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CL_switch7.csv';
% waypoints = [centerCube3;
%              centerCube3Approach;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% %%%%%%%%%%%% center start, right switch %%%%%%%%%%%%
% centerStart = [(fieldDim(1) / 2), (robotDim(2) / 2), 0, 0];
% rightSwitch = [fieldDim(1) - 85.25 - 10, 140 - (robotDim(2) / 2), 0, 0];
% centerCube1Approach = [(fieldDim(1) / 2), 100 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube1 = [(fieldDim(1) / 2), 100 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% centerCube2Approach = [(fieldDim(1) / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube2 = [(fieldDim(1) / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% centerCube3Approach = [(fieldDim(1) / 2) + (cubeDim / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist - cubeApproachDist, 0, 10];
% centerCube3 = [(fieldDim(1) / 2) + (cubeDim / 2), 120 + (cubeDim / 2) - (robotDim(2) / 2) - cubeGrabDist, 0, 0];
% 
% csvFilename = 'Path_CR_switch1.csv';
% waypoints = [centerStart;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch2.csv';
% waypoints = [rightSwitch;
%              centerCube1Approach;
%              centerCube1];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch3.csv';
% waypoints = [centerCube1;
%              centerCube1Approach;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch4.csv';
% waypoints = [rightSwitch;
%              centerCube2Approach;
%              centerCube2];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch5.csv';
% waypoints = [centerCube2;
%              centerCube2Approach;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch6.csv';
% waypoints = [rightSwitch;
%              centerCube3Approach;
%              centerCube3];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_CR_switch7.csv';
% waypoints = [centerCube3;
%              centerCube3Approach;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% %%%%%%%%%%%% left portal, left switch %%%%%%%%%%%%
% leftPortal = [30, fieldDim(2) - 30, 45, 0];
% leftSwitch = [85.25 - (robotDim(2) / 2) - 8, fieldDim(2) - 168, 90, 0];
% 
% csvFilename = 'Path_LL_portal1.csv';
% waypoints = [leftPortal;
%              leftSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_LL_portal2.csv';
% waypoints = [leftSwitch;
%              leftPortal];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% % 
% %%%%%%%%%%%% right portal, right switch %%%%%%%%%%%%
% rightPortal = [fieldDim(1) - 30, fieldDim(2) - 30, -45, 0];
% rightSwitch = [fieldDim(1) - 85.25 + (robotDim(2) / 2) + 8, fieldDim(2) - 168, -90, 0];
% 
% csvFilename = 'Path_RR_portal1.csv';
% waypoints = [rightPortal;
%              rightSwitch];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 
% 
% csvFilename = 'Path_RR_portal2.csv';
% waypoints = [rightSwitch;
%              rightPortal];
% generatePath(waypoints, csvFilename, maxSpeed, maxAccel, maxSpeed2, maxAccel2, maxDeccel, sampleRate) 