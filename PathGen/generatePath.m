function [] = generatePath(waypoints, csvFilename, maxSpeed, maxAccel, sampleRate)
% waypoints = [x position (in),
%              y position (in),
%              yaw (deg),
%              maximum distance away from point (in)],
%
%              generatePath() adds the following to waypoints...
%              temp path index,
%              distance traveled (in),
%              final path index];
%
% csvFilename = file name to output finalPath to .csv
%
% maxSpeed = max speed (in)
%
% sampleRate = sample rate (Hz)
%
% finalPath = [time (s),
%              x position (in),
%              y position (in),
%              yaw (deg),
%              x vel (in/s),
%              y vel (in/s),
%              yaw rate (deg/s),
%              x accel (in/s^2),
%              y accel (in/s^2),
%              yaw accel (deg/s^2)]

    % input error checking
    if(isempty(waypoints))
        display('*****waypoints is empty*****')
        return
    end

    if(~any(maxSpeed > 0))
        display('*****max speed is not positive real number*****')
        return
    end

    if(~any(maxAccel > 0))
        display('*****max accel is not positive real number*****')
        return
    end

    if(~any(sampleRate > 0))
        display('*****sample rate is not positive real number*****')
        return
    end

    % add start point to path
    tempPath = zeros(1, 3);
    tempPath(1, 1:2) = waypoints(1, 1:2);
    waypoints(1, 5) = 1;

    % loop through waypoints and round corners
    sizeWaypoints = size(waypoints);
    j = 2;
    i = 1;
    while(i < (sizeWaypoints(1) - 1))
        % get 3 consecutive points
        p1 = waypoints(i, 1:2);
        p2 = waypoints(i + 1, 1:2);
        p3 = waypoints(i + 2, 1:2);

        % get vectors between points
        v21 = p1 - p2; % vector between second and first points
        v23 = p3 - p2; % vector between second and third points

        % get max distance away from point
        D = waypoints(i + 1, 4);

        % check if vectors have zero length
        if (norm(v21) == 0) || (norm(v23) == 0)
            theta = 0; % skip waypoint
        else
            theta = acos(dot(v21, v23) / (norm(v21) * norm(v23))); % angle between v21 and v23
        end

        % check if points lie on straight line
        if (theta == pi)
            % do not insert rounded corner
            tempPath(end + 1, 1:2) = p2;
            sizeTempPath = size(tempPath);
            waypoints(j, 5) = sizeTempPath(1);
            j = j + 1;
        % check if redundant points
        elseif (theta == 0)
            % skip waypoint
            waypoints(j, :) = [];
            sizeWaypoints = size(waypoints);
            i = i - 1;
        else
            % calculate arc between points
            R = D * sin(theta / 2) / (1 - sin(theta / 2)); % arc radius
            H = R * (1 - cos((pi - theta) / 2)); % arc height
            C = 2 * R * sin((pi - theta) / 2); % arc chord length
            L = sqrt((D + H) * (D + H) + (C / 2) * (C / 2)); % distance between second point and where arc is tangent to straight line

            % limit arc radius if bigger than distance between points and update other parameters
            limitL = min(norm(v21), norm(v23)) / 2;
            if (L > limitL)
                L = limitL;
                C = 2 * L * sin(theta / 2);
                R = C * sin(theta / 2) / sin(pi - theta);
                H = R * (1 - cos((pi - theta) / 2));
                D = (L * cos(theta / 2)) - H;
            end

            % calculate points on arc tangent to vectors between points 1, 2, 3
            v24 = L * v21 / norm(v21);
            v25 = L * v23 / norm(v23);
            p4 = p2 + v24;
            p5 = p2 + v25;

            % add point 4 to path
            tempPath(end + 1, 1:2) = p4;

            % calculate center point of arc
            v26 = (v21 / norm(v21) + v23 / norm(v23)) / 2;
            p6 = p2 + (D + R) * v26 / norm(v26); % center point of arc
            crossV21V23 = cross([v21, 0], [v23, 0]);

            % generate points along arc
            for phi = linspace(sign(crossV21V23(3)) * (pi - theta) / 2, -sign(crossV21V23(3)) * (pi - theta) / 2, 51)
                p7(1) = p6(1) - R * (cos(phi) * v26(1) - sin(phi) * v26(2)) / norm(v26);
                p7(2) = p6(2) - R * (sin(phi) * v26(1) + cos(phi) * v26(2)) / norm(v26);

                % add p7 to path
                tempPath(end + 1, 1:2) = p7;
                if (-0.0001 < phi) && (phi < 0.001)
                    sizeTempPath = size(tempPath);
                    waypoints(j, 5) = sizeTempPath(1);
                    j = j + 1;
                end
            end

            % add p5 to path
            tempPath(end + 1, 1:2) = p5;
        end

        i = i + 1;
    end

    % add end point to path
    tempPath(end + 1, 1:2) = waypoints(end, 1:2);
    sizeTempPath = size(tempPath);
    waypoints(end, 5) = sizeTempPath(1);

    % calculate total path length
    sizeTempPath = size(tempPath);
    tempPath(1, 3) = 0;
    for i = 2:sizeTempPath(1)
        tempPath(i, 3) = tempPath(i - 1, 3) + norm(tempPath(i, 1:2) - tempPath(i - 1, 1:2));
    end

    % calculate acceleration time and distance
    accelTime = maxSpeed / maxAccel;
    accelDist = 0.5 * maxAccel * accelTime^2;

    % recalculate max speed if total path length is too small for max acceleration
    if (2 * accelDist > tempPath(end, 3))
        accelTime = sqrt(tempPath(end, 3) / 2 * 2 / maxAccel);
        accelDist = 0.5 * maxAccel * accelTime^2;
        maxSpeed = accelTime * maxAccel;
    end

    % calculate times during path
    totalTime = 2 * accelTime + (tempPath(end, 3) - 2 * accelDist) / maxSpeed;
    startMaxSpeedTime = accelTime;
    stopMaxSpeedTime = totalTime - accelTime;
    time = 0:(1 / sampleRate):(floor(totalTime * sampleRate) / sampleRate);

    % calculate speed with respect to time
    speed = 0;
    for i = time(2:end)
        if(i < startMaxSpeedTime)
            speed(end + 1) = maxAccel * i;
        elseif(i > stopMaxSpeedTime)
            speed(end + 1) = maxAccel * (totalTime - i);
        else
            speed(end + 1) = maxSpeed;
        end
    end

    % calculate distance traveled with respect to time
    dist = 0;
    for i = 2:length(time)
        if((startMaxSpeedTime < time(i)) && (time(i) < (startMaxSpeedTime + 1 / sampleRate)))
            dist(end + 1) = dist(end) + (time(i) - startMaxSpeedTime) * maxSpeed + (startMaxSpeedTime - time(i - 1)) * speed(i - 1);
        elseif((stopMaxSpeedTime < time(i)) && (time(i) < (stopMaxSpeedTime + 1 / sampleRate)))
            dist(end + 1) = dist(end) + (stopMaxSpeedTime - time(i - 1)) * maxSpeed + (time(i) - stopMaxSpeedTime) * speed(i);
        else
            dist(end + 1) = dist(end) + (speed(i) + speed(i - 1)) / (2 * sampleRate);
        end
    end

    % calculate distance traveled during last point
    dist(end + 1) = tempPath(end, 3);
    speed(end + 1) = 0;
    time(end + 1) = totalTime;

    % limit distance to total path length
    dist(dist > tempPath(end, 3)) = tempPath(end, 3);
    
    % add time to final path
    finalPath(:, 1) = time;
    lengthFinalPath = length(finalPath);

    % interpolate distance traveled to get final path positions
    finalPath(:, 2:3) = interparc(dist / tempPath(end, 3), tempPath(:, 1)', tempPath(:, 2)', 'linear');

    % get distance traveled corresponding to waypoints
    waypoints(:, 6) = tempPath(waypoints(:, 5), 3);

    % get final path index corresponding to waypoints
    j = 2;
    waypoints(1, 7) = 1;
    for i = 2:lengthFinalPath
        if (dist(i) > waypoints(j, 6))
            waypoints(j, 7) = i - 1;
            j = j + 1;
        end
    end
    waypoints(end, 7) = length(dist);

    % get yaw rate corresponding to final path
    for i = 2:sizeWaypoints(1)
        deltaYaw = waypoints(i, 3) - waypoints(i - 1, 3);

        % angle wraparound
        if (deltaYaw > 180)
            deltaYaw = 360 - deltaYaw;
        elseif (deltaYaw < -180)
            deltaYaw = 360 + deltaYaw;
        end

        deltaIdx = (waypoints(i, 7) - waypoints(i - 1, 7));
        yawRate(i - 1) = deltaYaw / deltaIdx;
    end

    % add yaw to final path
    j = 1;
    finalPath(1, 4) = waypoints(1, 3);
    for i = 2:lengthFinalPath
        if (i > waypoints(j, 7))
            j = j + 1;
        end

        newYaw = finalPath(i - 1, 4) + yawRate(j - 1);

        % angle wraparound
        if (newYaw > 180)
            newYaw = newYaw - 360;
        elseif (newYaw < -180)
            newYaw = newYaw + 360;
        end

        finalPath(i, 4) = newYaw;
    end
    
    % calculate final path velocity
    velX = diff(finalPath(:, 2)) ./ diff(finalPath(:, 1));
    velX(end) = 0;
    velX = [0; velX];
    velY = diff(finalPath(:, 3)) ./ diff(finalPath(:, 1));
    velY(end) = 0;
    velY = [0; velY];
    yawRate = diff(finalPath(:, 4)) ./ diff(finalPath(:, 1));
    yawRate(end) = 0;
    yawRate = [0; yawRate];
    
    % calculate final path acceleration
    accelX = diff(velX) ./ diff(finalPath(:, 1));
    accelX(end) = 0;
    accelX = [0; accelX];
    accelY = diff(velY) ./ diff(finalPath(:, 1));
    accelY(end) = 0;
    accelY = [0; accelY];
    yawAccel = diff(yawRate) ./ diff(finalPath(:, 1));
    yawAccel(end) = 0;
    yawAccel = [0; yawAccel];
    
    finalPath(:, 5) = velX;
    finalPath(:, 6) = velY;
    finalPath(:, 7) = yawRate;
    finalPath(:, 8) = accelX;
    finalPath(:, 9) = accelY;
    finalPath(:, 10) = yawAccel;
        
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
    switchDraw2 = [85.25, fieldDim(2) - 140;
                  fieldDim(1) - 85.25, fieldDim(2) - 140;
                  fieldDim(1) - 85.25, fieldDim(2) - 196;
                  85.25, fieldDim(2) - 196;
                  85.25, fieldDim(2) - 140];
    
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
    
    % plot path
%     close all
    figure
    plot(waypoints(:, 1), waypoints(:, 2), 'bo-')
    % quiver(waypoints(:, 1), waypoints(:, 2), cosd(waypoints(:, 3) + 90), sind(waypoints(:, 3) + 90), 'b');
    hold on
    plot(tempPath(:, 1), tempPath(:, 2), 'gx-')
    plot(finalPath(:, 2), finalPath(:, 3), 'rx-')
    % quiver(finalPath(:, 1), finalPath(:, 2), cosd(finalPath(:, 3) + 90), sind(finalPath(:, 3) + 90), 'r');
    plot(fieldDraw(:, 1), fieldDraw(:, 2), 'b')
    plot(switchDraw(:, 1), switchDraw(:, 2), 'b')
    plot(switchDraw2(:, 1), switchDraw2(:, 2), 'b')
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

    % write final path to .csv file
    csvwrite(csvFilename, finalPath);
    
    disp(csvFilename)
    disp(finalPath(end, 1))
end