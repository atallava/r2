function [meas,velLeft_ms,velRight_ms] = parseHuskyLog(filename,dispFlag)
    
    if nargin < 2
        dispFlag = false;
    end
    
    % load the file
    load(filename)
    
    % Interpret the linear and angular velocity from the encoder?
    % distance per encoder tick (m)
    distPerTick_m = 0.000244;
    
    % delta time difference between successive ticks (smoothed over all the data)
    deltaT_s = mean(diff(Encoder.Time));
    
    % the left and right wheel velocities
    velLeft_ms = zeros(1,length(Encoder.Time));
    velRight_ms = zeros(1,length(Encoder.Time));
    
    % the left wheel velocity (m/s) from encoder 1 - notice the sign flip
    velLeft_ms(1,2:end) = double(diff(-1.0*Encoder.Counts(1,:))) * distPerTick_m/deltaT_s;
    
    % the right wheel velocity (m/s) from encoder 2
    velRight_ms(1,2:end) = double(diff(Encoder.Counts(2,:))) * distPerTick_m/deltaT_s;
    
    
    % extract the "pose" fields from Ins(1,3)
    % Time, X, Y, Z, Roll, Pitch, Yaw, Forward Velocity, (Instantaneous path) Curvature
    
    
    %% populate meas structure
    pose = zeros(length(Ins(3).Time), 9);
    pose(:,1) = Ins(3).Time;
    
    % position is in Latitude, Longitude and Height
    pose(:,2:4) = Ins(3).Position';
    
    % convert from radians to degrees for the latitude and longitude
    pose(:,2:3) = rad2deg(pose(:,2:3));
    pose(:,5:7) = Ins(3).Orientation';
    
    % compute the cosine and sine of the heading/yaw
    c_theta = cos(Ins(3).Orientation(3,:));
    s_theta = sin(Ins(3).Orientation(3,:));
    
    % iterate through and compute the body velocity
    for i= 1:length(c_theta)
        
        % compute the body velocity in the forward direction
        % from velocity in N, E
        % [g_x] =  [ct -st 0]   [xi_x]
        % [g_y] =  [st  ct 0] * [xi_y]
        % [g_t] =  [0   0  1]   [xi_theta]
        % what we want are xi's - invert the matrix and multiply
        % Heading is measured from North hence theta = pi/2 - theta
        pose(i,8) = c_theta(i) * Ins(3).Velocity(1,i) + s_theta(i) * Ins(3).Velocity(2,i);
        
        % compute the instantaneous curvature
        if(0.0 ~= pose(i,8))
            pose(i,9) = Ins(3).Velocity(3,i)/pose(i,8);
        else % instead of infinite curvature, set it to something large...
            pose(i,9) = 1e6;
        end
    end
    
    % figure; hold on;
    % plot(rad2deg(Ins(3).Orientation(1,:)), 'r');
    % plot(rad2deg(Ins(3).Orientation(2,:)), 'g');
    % plot(rad2deg(Ins(3).Orientation(3,:)), 'b');
    
    % Solution type
    solType = Ins(3).SolutionType;
    
    % Solution status
    solStatus = Ins(3).SolutionStatus;
    
    % first the timestamp t
    meas.t = Encoder.Time;
    
    % the INS data is currently generated at 1KHz and the Encoder data is
    % coming in at 100Hz - need to interpolate/match the timestamps
    matchIdx = interp1(Ins(3).Time, 1:length(Ins(3).Time), meas.t, 'nearest');
    
    matchIdx(isnan(matchIdx)) = [];
    
    % just consider the elements of pose whose timestamp is close to the times
    % of the encoders
    pose = pose(matchIdx,:);
    
    % prune solType and solStatus
    solType = solType(matchIdx);
    solStatus = solStatus(matchIdx);
    
    % figure; hold on;
    % plot(0.5 * (velLeft_ms + velRight_ms), 'r');
    % plot(pose(:,8), 'g');
    
    % Here, we consider only a subset of the data where solType is 9 (RTK)
    % and solStatus is 2 (Navigating)
    matchIdx = ((solType == 9) & (solStatus == 2));
    
    % prune the data further
    pose = pose(matchIdx,:);
    meas.t = meas.t(matchIdx);
    meas.lat = pose(:,2); meas.lon = pose(:,3);
    solType = solType(matchIdx);
    solStatus = solStatus(matchIdx);
    velLeft_ms = velLeft_ms(matchIdx);
    velRight_ms = velRight_ms(matchIdx);
    
    % conversion from Lat-Lon to UTM
    p1 = [pose(1,2), pose(1,3)];
    z1 = utmzone(p1);
    [ellipsoid, estr] = utmgeoid(z1);
    
    utmstruct = defaultm('utm');
    utmstruct.zone = z1;
    utmstruct.geoid = ellipsoid;
    utmstruct = defaultm(utmstruct);
    
    % convert from Lat-Lon to UTM
    [pose(:,2) pose(:,3)] = mfwdtran(utmstruct, pose(:,2), pose(:,3));
    
    % the number of measurements
    nm=length(meas.t);
    
    % update meas.pos
    meas.pos = pose(:,2:4);
    
    %start at [0 0 0]. numerical error in animation otherwise
    gps_start = meas.pos(1,:);
    
    % offset all the measurements to be zero indexed
    meas.pos = meas.pos - ones(nm,1)*gps_start;
    
    if dispFlag
        figure; plot(meas.pos(:,1), meas.pos(:,2));
        xlabel('Relative Easting (m)'); ylabel('Relative Northing (m)'); grid on;
        title('Husky slip data collection path');
        %figure; plot(0.5 * (velLeft_ms + velRight_ms));
    end
    inpt = [];
    
end