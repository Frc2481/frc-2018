heading = heading - heading(1);
for i = 1:length(heading)
    if(heading(i) > 180)
        heading(i) = heading(i) - 360;
    elseif(heading(i) < -180)
        heading(i) = heading(i) + 360;
    end
end

timeSec = time / 1000000;

xStart = 46.66;
yStart = 19.5;
yawStart = 0;

robotWidth = 22.5; % 22.375;
robotLength = 27;

robotYaw = heading + yawStart + 0;
for i = 1:length(robotYaw)
    if(robotYaw(i) > 180)
        robotYaw(i) = robotYaw(i) - 360;
    elseif(heading(i) < -180)
        robotYaw(i) = robotYaw(i) + 360;
    end
end

robotPos = [xStart;
            yStart];
robotPosHist = robotPos;

deltaFLDriveCenterHist = zeros(2, 1);
deltaFRDriveCenterHist = zeros(2, 1);
deltaBLDriveCenterHist = zeros(2, 1);
deltaBRDriveCenterHist = zeros(2, 1);
projAccelHist = zeros(2, 1);
latSlipDeltaHist = 0;

deltaMeanHist = zeros(2, 1);
deltaRmsHist = zeros(2, 1);

windowSize = 5;
accel = [-movingmean(rawAccelX, windowSize), -movingmean(rawAccelY, windowSize)];

for i = 2:length(time)
    tempI = i - 0;
    if(tempI > length(time))
        tempI = length(time);
    elseif(tempI < 2)
        tempI = 2;
    end
    
    deltaYaw = robotYaw(tempI) - robotYaw(tempI - 1);
    if deltaYaw > 180
        deltaYaw = 360 - deltaYaw;
    elseif deltaYaw < -180
        deltaYaw = 360 + deltaYaw;
    end
    
    tempAccel = accel(i, :)';
    R90 = -[cosd(90) -sind(90);
            sind(90) cosd(90)];
    deltaTime = timeSec(i) - timeSec(i - 1);
    
    radPercent = 0.99;
    cornerStiff = 0; % 0.7
    RSteer = [cosd(-flSteer(i)) -sind(-flSteer(i));
              sind(-flSteer(i)) cosd(-flSteer(i))];
    deltaDriveRaw = (flDrive(i) - flDrive(i - 1)) * radPercent;
    projAccel = RSteer * tempAccel * sign(deltaDriveRaw);
    projAccelHist(:, end + 1) = projAccel;
    latSlipDelta = -deltaDriveRaw * projAccel(1) * cornerStiff;
    latSlipDeltaHist(:, end + 1) = latSlipDelta;
    deltaDrive = RSteer * R90 * [deltaDriveRaw; latSlipDelta];
    deltaFLDriveCenter = deltaDrive - deltaYaw * pi / 180 * [-robotLength / 2; -robotWidth / 2];
    deltaFLDriveCenterHist(:, end + 1) = deltaFLDriveCenter;

    RSteer = [cosd(-frSteer(i)) -sind(-frSteer(i));
              sind(-frSteer(i)) cosd(-frSteer(i))];
    deltaDriveRaw = (frDrive(i) - frDrive(i - 1)) * radPercent;
    projAccel = RSteer * tempAccel * sign(deltaDriveRaw);
    latSlipDelta = -deltaDriveRaw * projAccel(1) * cornerStiff;
    deltaDrive = RSteer * R90 * [deltaDriveRaw; latSlipDelta];
    deltaFRDriveCenter = deltaDrive - deltaYaw * pi / 180 * [-robotLength / 2; -robotWidth / 2];
    deltaFRDriveCenterHist(:, end + 1) = deltaFRDriveCenter;

    RSteer = [cosd(-blSteer(i)) -sind(-blSteer(i));
              sind(-blSteer(i)) cosd(-blSteer(i))];
    deltaDriveRaw = (blDrive(i) - blDrive(i - 1)) * radPercent;
    projAccel = RSteer * tempAccel * sign(deltaDriveRaw);
    latSlipDelta = -deltaDriveRaw * projAccel(1) * cornerStiff;
    deltaDrive = RSteer * R90 * [deltaDriveRaw; latSlipDelta];
    deltaBLDriveCenter = deltaDrive - deltaYaw * pi / 180 * [-robotLength / 2; -robotWidth / 2];
    deltaBLDriveCenterHist(:, end + 1) = deltaBLDriveCenter;
    
    RSteer = [cosd(-brSteer(i)) -sind(-brSteer(i));
              sind(-brSteer(i)) cosd(-brSteer(i))];
    deltaDriveRaw = (brDrive(i) - brDrive(i - 1)) * radPercent;
    projAccel = RSteer * tempAccel * sign(deltaDriveRaw);
    latSlipDelta = -deltaDriveRaw * projAccel(1) * cornerStiff;
    deltaDrive = RSteer * R90 * [deltaDriveRaw; latSlipDelta];
    deltaBRDriveCenter = deltaDrive - deltaYaw * pi / 180 * [-robotLength / 2; -robotWidth / 2];
    deltaBRDriveCenterHist(:, end + 1) = deltaBRDriveCenter;
    
%     deltaCenter = (deltaFLDriveCenter * 1 + deltaFRDriveCenter * 0 + deltaBLDriveCenter * 0 + deltaBRDriveCenter * 0) / (4 - 3);
%     deltaCenter = (deltaFLDriveCenter * 0 + deltaFRDriveCenter * 1 + deltaBLDriveCenter * 0 + deltaBRDriveCenter * 0) / (4 - 3);
%     deltaCenter = (deltaFLDriveCenter * 0 + deltaFRDriveCenter * 0 + deltaBLDriveCenter * 1 + deltaBRDriveCenter * 0) / (4 - 3);
%     deltaCenter = (deltaFLDriveCenter * 0 + deltaFRDriveCenter * 0 + deltaBLDriveCenter * 0 + deltaBRDriveCenter * 1) / (4 - 3);
%     deltaCenter = (deltaFLDriveCenter * 1 + deltaFRDriveCenter * 1 + deltaBLDriveCenter * 1 + deltaBRDriveCenter * 1) / (4 - 0);
%     deltaCenter = (deltaFLDriveCenter * 1 + deltaFRDriveCenter * 1 + deltaBLDriveCenter * 0 + deltaBRDriveCenter * 0) / (4 - 2);
    
    deltaMean = mean([deltaFLDriveCenter, deltaFRDriveCenter, deltaBLDriveCenter, deltaBRDriveCenter], 2);
    deltaMeanHist(:, end + 1) = deltaMean;
    deltaRms = std([deltaFLDriveCenter, deltaFRDriveCenter, deltaBLDriveCenter, deltaBRDriveCenter], 1, 2);
    deltaRmsHist(:, end + 1) = deltaRms;
    
    rmsGain = 1000; % 0.5;
    isFLSlip = norm(deltaFLDriveCenter) > (norm(deltaMean) + rmsGain * norm(deltaRms));
    isFRSlip = norm(deltaFRDriveCenter) > (norm(deltaMean) + rmsGain * norm(deltaRms));
    isBLSlip = norm(deltaBLDriveCenter) > (norm(deltaMean) + rmsGain * norm(deltaRms));
    isBRSlip = norm(deltaBRDriveCenter) > (norm(deltaMean) + rmsGain * norm(deltaRms));
    
    deltaCenter = (deltaFLDriveCenter * ~isFLSlip + deltaFRDriveCenter * ~isFRSlip + deltaBLDriveCenter * ~isBLSlip + deltaBRDriveCenter * ~isBRSlip) / (~isFLSlip + ~isFRSlip + ~isBLSlip + ~isBRSlip);

    RField = [cosd(robotYaw(tempI)) -sind(robotYaw(tempI));
              sind(robotYaw(tempI)) cosd(robotYaw(tempI))];
     
%     R = [cosd((robotYaw(i) + robotYaw(i - 1)) / 2) -sind((robotYaw(i) + robotYaw(i - 1)) / 2);
%          sind((robotYaw(i) + robotYaw(i - 1)) / 2) cosd((robotYaw(i) + robotYaw(i - 1)) / 2)];
    
    robotPos = robotPos + RField * deltaCenter;
    robotPosHist(:, end + 1) = robotPos;
end

% draw field
fieldDim = 12 * [27, 54];
fieldDraw = [0, 0;
             fieldDim(1), 0;
             fieldDim;
             0, fieldDim(2);
             0, 0];

% draw switch
switchDraw = [85.25, 140;
              fieldDim(1) - 85.25, 140;
              fieldDim(1) - 85.25, 196;
              85.25, 196;
              85.25, 140];

% draw scale
scaleDraw = [71.57, 299.65;
             fieldDim(1) - 71.57, 299.65;
             fieldDim(1) - 71.57, 348.35;
             71.57, 348.35;
             71.57, 299.65];

% draw platform
platformDraw = [95.25, 261.47;
                fieldDim(1) - 95.25, 261.47;
                fieldDim(1) - 95.25, 386.53;
                95.25, 386.53;
                95.25, 261.47];

% draw cubes
cubeDim = 13;
cubeSpacing = 28.1;
cube1 = [85.25 + 0 * cubeSpacing, 196, cubeDim, cubeDim];
cube2 = [85.25 + 1 * cubeSpacing, 196, cubeDim, cubeDim];
cube3 = [85.25 + 2 * cubeSpacing, 196, cubeDim, cubeDim];
cube4 = [85.25 + 3 * cubeSpacing, 196, cubeDim, cubeDim];
cube5 = [85.25 + 4 * cubeSpacing, 196, cubeDim, cubeDim];
cube6 = [85.25 + 5 * cubeSpacing, 196, cubeDim, cubeDim];
cube7 = [(fieldDim(1) / 2) - (cubeDim / 2), 100, cubeDim, cubeDim];
cube8 = [(fieldDim(1) / 2) - cubeDim, 120 - (cubeDim / 2), cubeDim, cubeDim];
cube9 = [(fieldDim(1) / 2), 120 - (cubeDim / 2), cubeDim, cubeDim];
cube10 = [(fieldDim(1) / 2) - (cubeDim / 2), 120 + (cubeDim / 2), cubeDim, cubeDim];
cube11 = [(fieldDim(1) / 2) - 1.5 * cubeDim, 120 + (cubeDim / 2), cubeDim, cubeDim];
cube12 = [(fieldDim(1) / 2) + (cubeDim / 2), 120 + (cubeDim / 2), cubeDim, cubeDim];

% plot pose
close all
figure
plot(robotPosHist(1, :), robotPosHist(2, :))
hold on
plot(x, y)
plot(fieldDraw(:, 1), fieldDraw(:, 2), 'b')
plot(switchDraw(:, 1), switchDraw(:, 2), 'b')
plot(scaleDraw(:, 1), scaleDraw(:, 2), 'b')
plot(platformDraw(:, 1), platformDraw(:, 2), 'b')
rectangle('Position', cube1, 'EdgeColor', 'b')
rectangle('Position', cube2, 'EdgeColor', 'b')
rectangle('Position', cube3, 'EdgeColor', 'b')
rectangle('Position', cube4, 'EdgeColor', 'b')
rectangle('Position', cube5, 'EdgeColor', 'b')
rectangle('Position', cube6, 'EdgeColor', 'b')
rectangle('Position', cube7, 'EdgeColor', 'b')
rectangle('Position', cube8, 'EdgeColor', 'b')
rectangle('Position', cube9, 'EdgeColor', 'b')
rectangle('Position', cube10, 'EdgeColor', 'b')
rectangle('Position', cube11, 'EdgeColor', 'b')
rectangle('Position', cube12, 'EdgeColor', 'b')
hold off
axis([-5 700 -5 700])

figure
plot(timeSec, robotPosHist(1, :))
hold on
plot(timeSec, x)
hold off

figure
plot(timeSec, robotPosHist(2, :))
hold on
plot(timeSec, y)
hold off

% angCorrect = 90 - atan2d(robotPos(2), robotPos(1))
% endX = robotPos(1)
% endY = robotPos(2)

windowSize = 1;
figure
plot(timeSec(2:end), movingmean(deltaFLDriveCenterHist(1, 2:end)' ./ diff(timeSec), windowSize))
hold on
plot(timeSec(2:end), movingmean(deltaFRDriveCenterHist(1, 2:end)' ./ diff(timeSec), windowSize))
plot(timeSec(2:end), movingmean(deltaBLDriveCenterHist(1, 2:end)' ./ diff(timeSec), windowSize))
plot(timeSec(2:end), movingmean(deltaBRDriveCenterHist(1, 2:end)' ./ diff(timeSec), windowSize))
hold off

figure
plot(timeSec(2:end), movingmean(deltaFLDriveCenterHist(2, 2:end)' ./ diff(timeSec), windowSize))
hold on
plot(timeSec(2:end), movingmean(deltaFRDriveCenterHist(2, 2:end)' ./ diff(timeSec), windowSize))
plot(timeSec(2:end), movingmean(deltaBLDriveCenterHist(2, 2:end)' ./ diff(timeSec), windowSize))
plot(timeSec(2:end), movingmean(deltaBRDriveCenterHist(2, 2:end)' ./ diff(timeSec), windowSize))
hold off

% figure
% plot(timeSec, deltaMeanHist(1, :))
% 
% figure
% plot(timeSec, deltaRmsHist(1, :))

% figure
% plot(timeSec, projAccelHist(1, :))
% 
% figure
% plot(timeSec, projAccelHist(2, :))

figure
plot(timeSec, latSlipDeltaHist(1, :))

figure
plot(timeSec, heading)